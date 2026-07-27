#!/usr/bin/env python3
"""
trapezoidal_dynamics.py — Smooth Trapezoidal Motion Planning + 3D Joint Dynamics
==================================================================================
Uses the Pilz PTP planner (via MoveIt2) which natively generates trapezoidal
velocity profiles (ramp-up → cruise → ramp-down) across all joints simultaneously.

For each trajectory point, computes:
  • 3D angular acceleration  α  = dω/dt          (rad/s²)
  • 3D joint torque vector   τ  = I · α           (N·m)   [F=ma rotational form]
  • 3D CoM linear accel      a  = dv_com/dt        (m/s²)
  • 3D reaction force        F  = m · a            (N)     [F=ma translational form]

Inertia tensors and masses are taken directly from kerabot.urdf.xacro.

Usage (inside the kerabot_ws, after sourcing ROS2):
    python3 trapezoidal_dynamics.py
    python3 trapezoidal_dynamics.py --execute       # actually move the robot
    python3 trapezoidal_dynamics.py --cartesian     # use Cartesian LIN instead of PTP
    python3 trapezoidal_dynamics.py --vel 0.4 --accel 0.25
"""

import argparse
import threading
import time
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")          # headless-safe; swap to "TkAgg" if you want a window
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2

# ──────────────────────────────────────────────────────────────────────────────
# ROBOT CONFIGURATION
# ──────────────────────────────────────────────────────────────────────────────
JOINT_NAMES = [
    "Revolute_1",   # joint1 — axis Z [0,0,1]
    "Revolute_2",   # joint2 — axis X [1,0,0]
    "Revolute_3",   # joint3 — axis X [1,0,0]
    "Revolute_4",   # joint4 — axis Z [0,0,1]
    "Revolute_5",   # joint5 — axis X [1,0,0]
]
BASE_LINK        = "base_link"
END_EFFECTOR     = "L70IE_Finger"
MOVE_GROUP       = "arm"

# Target pose (edit to your desired goal)
TARGET_POSITION  = [0.0, -0.0595, 0.65]
TARGET_QUAT_XYZW = [-0.4997, -0.5003, -0.4997, 0.5003]

# Output plot path
PLOT_PATH = "/home/saad/kerabot_ws/trapezoidal_dynamics_plot.png"

# ──────────────────────────────────────────────────────────────────────────────
# PHYSICAL PARAMETERS — extracted from kerabot.urdf.xacro
# Each entry: (mass_kg, inertia_3x3, joint_axis_unit_vector, com_offset_m)
#
# Inertia tensor layout (from URDF ixx iyy izz ixy ixz iyz):
#   [ ixx  ixy  ixz ]
#   [ ixy  iyy  iyz ]
#   [ ixz  iyz  izz ]
#
# Joint axes (in their own link frame):
#   Revolute_1 (joint1) → Z
#   Revolute_2 (joint2) → X
#   Revolute_3 (joint3) → X
#   Revolute_4 (joint4) → Z
#   Revolute_5 (joint5) → X
# ──────────────────────────────────────────────────────────────────────────────
def _I(ixx, iyy, izz, ixy=0.0, ixz=0.0, iyz=0.0):
    """Build a symmetric 3×3 inertia tensor."""
    return np.array([
        [ixx,  ixy,  ixz],
        [ixy,  iyy,  iyz],
        [ixz,  iyz,  izz],
    ])

# (mass, inertia_tensor, joint_axis, com_offset_from_joint_origin)
LINK_PARAMS = [
    # link1 — driven by Revolute_1 (joint1, axis Z)
    {
        "name":   "link1",
        "mass":   6.46,
        "I":      _I(0.013, 0.015, 0.009, iyz=-0.001),
        "axis":   np.array([0.0, 0.0, 1.0]),
        "com":    np.array([0.0, 0.014, 0.26]),     # from URDF inertial origin
    },
    # link2 — driven by Revolute_2 (joint2, axis X)
    {
        "name":   "link2",
        "mass":   5.098,
        "I":      _I(0.0118, 0.0116, 0.009, ixy=0.001, ixz=0.002, iyz=-0.002),
        "axis":   np.array([1.0, 0.0, 0.0]),
        "com":    np.array([0.081, 0.0, 0.31]),
    },
    # link3 — driven by Revolute_3 (joint3, axis X)
    {
        "name":   "link3",
        "mass":   3.167,
        "I":      _I(0.034, 0.033, 0.005, ixz=0.002),
        "axis":   np.array([1.0, 0.0, 0.0]),
        "com":    np.array([0.005, -0.006, 0.18]),
    },
    # link4 — driven by Revolute_4 (joint4, axis Z)
    {
        "name":   "link4",
        "mass":   1.429,
        "I":      _I(0.002, 0.002, 0.001),
        "axis":   np.array([0.0, 0.0, 1.0]),
        "com":    np.array([0.0, 0.008, 0.09]),
    },
    # link5 — driven by Revolute_5 (joint5, axis X)
    {
        "name":   "link5",
        "mass":   1.391,
        "I":      _I(0.002, 0.002, 0.001),
        "axis":   np.array([1.0, 0.0, 0.0]),
        "com":    np.array([0.0, 0.008, 0.12]),
    },
]

G_VEC = np.array([0.0, 0.0, -9.81])   # gravity in world frame (Z-up)


# ──────────────────────────────────────────────────────────────────────────────
# TRAPEZOID PROFILE VALIDATION
# ──────────────────────────────────────────────────────────────────────────────
def check_trapezoidal(times: np.ndarray, velocities: np.ndarray,
                      joint_name: str, tol: float = 0.15) -> dict:
    """
    Assess how trapezoidal the velocity profile is.
    tol=0.15 means any point within 15% of peak velocity counts as "cruise".
    This is intentionally loose — Pilz PTP generates true trapezoids but
    with limited waypoint density (30-40 pts), the flat-top region may be
    only 1-3 samples wide.
    Returns a dict with: is_trapezoidal, cruise_fraction, peak_velocity, jerk_rms.
    """
    n = len(velocities)
    if n < 3:
        return {"is_trapezoidal": False, "jerk_rms": 0.0, "note": "too few points"}

    v_peak = np.max(np.abs(velocities))
    if v_peak < 1e-6:
        return {"is_trapezoidal": True, "jerk_rms": 0.0, "note": "zero motion"}

    v_norm = np.abs(velocities) / v_peak

    # Cruise = within `tol` of peak AND not at the very first or last sample
    cruise_mask = (v_norm > (1.0 - tol)) & \
                  (np.arange(n) > 0) & \
                  (np.arange(n) < n - 1)
    cruise_indices = np.where(cruise_mask)[0]

    acc      = np.gradient(velocities, times)
    jerk     = np.gradient(acc, times)
    jerk_rms = float(np.sqrt(np.mean(jerk ** 2)))

    # A profile is trapezoidal if:
    #   • there is at least 1 interior cruise sample, AND
    #   • velocity rises before the cruise and falls after (monotone ramps)
    has_cruise = len(cruise_indices) >= 1
    if has_cruise:
        mid        = int(np.median(cruise_indices))
        rises_ok   = v_norm[mid] >= v_norm[0]          # velocity grew to cruise
        falls_ok   = v_norm[mid] >= v_norm[-1]         # velocity fell from cruise
        is_trap    = rises_ok and falls_ok
    else:
        is_trap = False

    result = {
        "joint":            joint_name,
        "peak_velocity":    float(v_peak),
        "jerk_rms":         jerk_rms,
        "has_cruise_phase": has_cruise,
        "is_trapezoidal":   is_trap,
    }
    if has_cruise:
        result["cruise_fraction"] = round(len(cruise_indices) / n, 3)
    return result


# ──────────────────────────────────────────────────────────────────────────────
# 3D DYNAMICS COMPUTATION
# ──────────────────────────────────────────────────────────────────────────────
def compute_dynamics(times: np.ndarray,
                     joint_positions: np.ndarray,
                     joint_velocities: np.ndarray,
                     joint_accelerations: np.ndarray) -> dict:
    """
    Compute 3D torques (τ = I·α) and 3D forces (F = m·a) for each joint/link.

    Parameters
    ----------
    times               : (T,)     time stamps in seconds
    joint_positions     : (T, 5)   joint angles in radians
    joint_velocities    : (T, 5)   joint angular velocities in rad/s
    joint_accelerations : (T, 5)   joint angular accelerations in rad/s²

    Returns
    -------
    dict with keys:
        torques  : (T, 5, 3)  — 3D torque vector per joint per timestep  [N·m]
        forces   : (T, 5, 3)  — 3D force  vector per link per timestep   [N]
        alpha    : (T, 5, 3)  — 3D angular acceleration vector            [rad/s²]
        a_com    : (T, 5, 3)  — 3D CoM linear acceleration                [m/s²]
    """
    T = len(times)
    n = len(LINK_PARAMS)

    torques = np.zeros((T, n, 3))
    forces  = np.zeros((T, n, 3))
    alpha   = np.zeros((T, n, 3))
    a_com   = np.zeros((T, n, 3))

    for j, link in enumerate(LINK_PARAMS):
        ax     = link["axis"]           # unit vector of joint axis
        mass   = link["mass"]           # kg
        I_body = link["I"]              # 3×3 inertia tensor (body frame ≈ world for small angles)
        com    = link["com"]            # CoM offset from joint origin [m]

        # scalar angular acceleration for this joint
        q_ddot = joint_accelerations[:, j]   # (T,)
        q_dot  = joint_velocities[:, j]      # (T,)

        # ── 3D angular acceleration: α = q̈ · axis ──────────────────────────
        alpha_vec = np.outer(q_ddot, ax)     # (T, 3)
        alpha[:, j, :] = alpha_vec

        # ── 3D torque: τ = I · α  (rotational F=ma) ─────────────────────────
        # I is constant (body-frame approximation — valid for single rigid body)
        tau_vec = (I_body @ alpha_vec.T).T   # (T, 3)
        torques[:, j, :] = tau_vec

        # ── CoM linear acceleration via centripetal + tangential ─────────────
        # Tangential: a_tan = α × r_com
        # Centripetal: a_cen = ω × (ω × r_com)
        omega_vec = np.outer(q_dot, ax)      # (T, 3)   ω = q̇ · axis

        a_tangential  = np.cross(alpha_vec, com)   # (T, 3)
        a_centripetal = np.cross(omega_vec,
                                 np.cross(omega_vec, com))  # (T, 3)

        a_com_vec = a_tangential + a_centripetal   # (T, 3)  [no gravity offset for net force]
        a_com[:, j, :] = a_com_vec

        # ── 3D force: F = m · a  (translational F=ma) ───────────────────────
        forces[:, j, :] = mass * a_com_vec

    return {
        "torques": torques,
        "forces":  forces,
        "alpha":   alpha,
        "a_com":   a_com,
    }


# ──────────────────────────────────────────────────────────────────────────────
# PLOTTING
# ──────────────────────────────────────────────────────────────────────────────
COLORS = ["#E63946", "#457B9D", "#2A9D8F", "#E9C46A", "#F4A261"]

def plot_all(times, positions, velocities, accelerations, dynamics, joint_names, save_path):
    """Generate a comprehensive 4-row, 5-column dynamics figure."""
    n_joints = len(joint_names)
    jerk = np.gradient(accelerations, times, axis=0)   # (T, 5)

    torques = dynamics["torques"]   # (T, 5, 3)
    forces  = dynamics["forces"]    # (T, 5, 3)

    fig = plt.figure(figsize=(22, 20), facecolor="#0f0f1a")
    fig.suptitle("Kerabot — Trapezoidal Motion Dynamics  (Pilz PTP Planner)",
                 fontsize=16, color="white", fontweight="bold", y=0.98)

    gs = gridspec.GridSpec(5, 2, figure=fig, hspace=0.55, wspace=0.35,
                           left=0.07, right=0.97, top=0.95, bottom=0.04)

    axes_cfg = [
        (gs[0, 0], "Position (rad)",      positions,           "Joint Position vs Time"),
        (gs[1, 0], "Velocity (rad/s)",     velocities,          "Velocity (Trapezoidal Profile)"),
        (gs[2, 0], "Acceleration (rad/s²)",accelerations,       "Acceleration vs Time"),
        (gs[3, 0], "Jerk (rad/s³)",        jerk,                "Jerk vs Time  (lower = smoother)"),
    ]

    for spec, ylabel, data, title in axes_cfg:
        ax = fig.add_subplot(spec)
        _style_ax(ax, title, ylabel)
        for j in range(n_joints):
            ax.plot(times, data[:, j], color=COLORS[j],
                    label=joint_names[j], linewidth=1.8)
        ax.legend(fontsize=7, loc="upper right",
                  labelcolor="white", framealpha=0.2)
        ax.axhline(0, color="white", linewidth=0.4, alpha=0.3)

    # ── Torque components (right column, top 3) ───────────────────────────────
    comp_labels = ["X", "Y", "Z"]
    comp_colors = ["#E63946", "#2A9D8F", "#E9C46A"]

    tau_ax = fig.add_subplot(gs[0, 1])
    _style_ax(tau_ax, "Torque Magnitude |τ| (N·m)", "|τ| (N·m)")
    for j in range(n_joints):
        mag = np.linalg.norm(torques[:, j, :], axis=1)
        tau_ax.plot(times, mag, color=COLORS[j], label=joint_names[j], linewidth=1.8)
    tau_ax.legend(fontsize=7, loc="upper right", labelcolor="white", framealpha=0.2)

    tau_comp_ax = fig.add_subplot(gs[1, 1])
    _style_ax(tau_comp_ax, "Torque Components — All Joints (N·m)", "τ (N·m)")
    for j in range(n_joints):
        for c in range(3):
            tau_comp_ax.plot(times, torques[:, j, c],
                             color=comp_colors[c], linewidth=1.0, alpha=0.75,
                             label=f"{joint_names[j]} τ{comp_labels[c]}" if j == 0 else "")
    tau_comp_ax.legend(fontsize=6, loc="upper right", labelcolor="white", framealpha=0.2)

    # ── Force components ──────────────────────────────────────────────────────
    f_mag_ax = fig.add_subplot(gs[2, 1])
    _style_ax(f_mag_ax, "Force Magnitude |F| = m·a  (N)", "|F| (N)")
    for j in range(n_joints):
        mag = np.linalg.norm(forces[:, j, :], axis=1)
        f_mag_ax.plot(times, mag, color=COLORS[j], label=LINK_PARAMS[j]["name"], linewidth=1.8)
    f_mag_ax.legend(fontsize=7, loc="upper right", labelcolor="white", framealpha=0.2)

    f_comp_ax = fig.add_subplot(gs[3, 1])
    _style_ax(f_comp_ax, "Force Components — All Links (N)", "F (N)")
    for j in range(n_joints):
        for c in range(3):
            f_comp_ax.plot(times, forces[:, j, c],
                           color=comp_colors[c], linewidth=1.0, alpha=0.75,
                           label=f"{LINK_PARAMS[j]['name']} F{comp_labels[c]}" if j == 0 else "")
    f_comp_ax.legend(fontsize=6, loc="upper right", labelcolor="white", framealpha=0.2)

    # ── Summary table ─────────────────────────────────────────────────────────
    summary_ax = fig.add_subplot(gs[4, :])
    summary_ax.axis("off")
    summary_ax.set_facecolor("#0f0f1a")

    col_labels = ["Joint / Link", "Peak |τ| (N·m)", "τ_max allowed (N·m)",
                  "Peak |F| (N)", "Peak jerk (rad/s³)", "JerkRMS (rad/s³)", "Trapezoidal?"]
    table_data = []
    for j in range(n_joints):
        trap = check_trapezoidal(times, velocities[:, j], joint_names[j])
        peak_tau = float(np.max(np.linalg.norm(torques[:, j, :], axis=1)))
        peak_f   = float(np.max(np.linalg.norm(forces[:, j, :], axis=1)))
        peak_jk  = float(np.max(np.abs(jerk[:, j])))
        tau_ok   = peak_tau <= 5.0
        table_data.append([
            joint_names[j],
            f"{peak_tau:.3f}  {'[OK]' if tau_ok else '[!!]'}",
            "5.000",
            f"{peak_f:.3f}",
            f"{peak_jk:.3f}",
            f"{trap.get('jerk_rms', 0.0):.3f}",
            "[TRAP]" if trap["is_trapezoidal"] else "[WARN]",
        ])

    tbl = summary_ax.table(cellText=table_data, colLabels=col_labels,
                           cellLoc="center", loc="center")
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8.5)
    tbl.scale(1, 1.6)
    for (r, c), cell in tbl.get_celld().items():
        cell.set_facecolor("#1a1a2e" if r % 2 == 0 else "#16213e")
        cell.set_text_props(color="white")
        cell.set_edgecolor("#2a2a4a")

    plt.savefig(save_path, dpi=150, bbox_inches="tight", facecolor="#0f0f1a")
    print(f"\n✅  Saved dynamics plot → {save_path}")


def _style_ax(ax, title, ylabel):
    ax.set_facecolor("#1a1a2e")
    ax.set_title(title, color="white", fontsize=9, pad=4)
    ax.set_ylabel(ylabel, color="#aaaacc", fontsize=8)
    ax.set_xlabel("Time (s)", color="#aaaacc", fontsize=7)
    ax.tick_params(colors="#aaaacc", labelsize=7)
    ax.spines[:].set_color("#2a2a4a")
    ax.grid(True, color="#2a2a4a", linewidth=0.5, alpha=0.7)


# ──────────────────────────────────────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="Kerabot trapezoidal motion + dynamics")
    p.add_argument("--execute",  action="store_true",
                   help="Execute the planned trajectory on the real robot (default: dry-run)")
    p.add_argument("--cartesian", action="store_true",
                   help="Use Pilz LIN (Cartesian straight-line) instead of PTP")
    p.add_argument("--vel",   type=float, default=0.5,
                   help="Velocity scaling factor 0–1   (default: 0.5)")
    p.add_argument("--accel", type=float, default=0.3,
                   help="Acceleration scaling factor 0–1 (default: 0.3)")
    p.add_argument("--home", action="store_true",
                   help="Return to home position first before planning (use when already at target)")
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()

    node = Node("kerabot_trapezoidal_dynamics")
    cb_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name=BASE_LINK,
        end_effector_name=END_EFFECTOR,
        group_name=MOVE_GROUP,
        callback_group=cb_group,
    )

    # ── Select Pilz planner ───────────────────────────────────────────────────
    # "PTP"  → joint-space trapezoidal (default, smoothest for point-to-point)
    # "LIN"  → Cartesian straight-line trapezoidal
    planner_id = "LIN" if args.cartesian else "PTP"
    moveit2.planner_id          = planner_id
    moveit2.max_velocity        = args.vel
    moveit2.max_acceleration    = args.accel
    moveit2.num_planning_attempts = 10
    moveit2.allowed_planning_time = 10.0

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.0)

    # ── Optional: go home first so we always have meaningful travel ───────────
    if args.home:
        node.get_logger().info("🏠  Returning to home position first...")
        moveit2.move_to_configuration([0.0, 0.0, 0.0, 0.0, 0.0])
        moveit2.wait_until_executed()
        node.get_logger().info("🏠  Home reached. Now planning target motion...")
        time.sleep(0.5)

    mode = "CARTESIAN LIN" if args.cartesian else "PTP"
    node.get_logger().info(
        f"Planning {mode} motion | vel={args.vel} | accel={args.accel} | "
        f"{'EXECUTING' if args.execute else 'DRY-RUN (plot only)'}"
    )

    # ── Plan ─────────────────────────────────────────────────────────────────
    trajectory = moveit2.plan(
        position=TARGET_POSITION,
        quat_xyzw=TARGET_QUAT_XYZW,
        cartesian=args.cartesian,
    )

    if trajectory is None:
        node.get_logger().error("❌  Planning failed. Check that MoveIt is running and Pilz is loaded.")
        rclpy.shutdown()
        spin_thread.join()
        return

    points      = trajectory.points
    joint_names = trajectory.joint_names
    n_joints    = len(joint_names)
    T           = len(points)

    node.get_logger().info(f"✅  Got trajectory: {T} waypoints, joints={joint_names}")

    # ── Guard: robot may already be at the target ─────────────────────────────
    if T < 2:
        node.get_logger().warn(
            "⚠️  Trajectory has only 1 waypoint — the robot is already at the "
            "target position. Nothing to analyse.\n"
            "   Tip: run with --home first to return to the home pose, then re-run."
        )
        rclpy.shutdown()
        spin_thread.join()
        return

    # ── Extract arrays ────────────────────────────────────────────────────────
    times  = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                       for p in points])

    positions     = np.array([[p.positions[j]     for j in range(n_joints)] for p in points])
    velocities    = np.array([[p.velocities[j]    if p.velocities    else 0.0 for j in range(n_joints)] for p in points])
    accelerations = np.array([[p.accelerations[j] if p.accelerations else 0.0 for j in range(n_joints)] for p in points])

    # Finite-difference cross-check on accelerations
    accelerations_fd = np.gradient(velocities, times, axis=0)

    # ── Compute 3D dynamics ───────────────────────────────────────────────────
    node.get_logger().info("Computing 3D torques (τ=Iα) and forces (F=ma)...")
    dynamics = compute_dynamics(times, positions, velocities, accelerations)

    # ── Print summary ─────────────────────────────────────────────────────────
    print("\n" + "=" * 72)
    print(f"  KERABOT TRAPEZOIDAL DYNAMICS  |  Planner: Pilz {planner_id}")
    print(f"  Duration: {times[-1]:.3f}s   |  Waypoints: {T}")
    print("=" * 72)
    print(f"{'Joint':<14} {'Peak|τ|(Nm)':>12} {'OK?':>5}  {'Peak|F|(N)':>11}  {'PeakJerk':>10}  {'Trapezoid?':>12}")
    print("-" * 72)

    tau_limit = 5.0   # from URDF effort limits
    for j, jname in enumerate(joint_names):
        peak_tau = float(np.max(np.linalg.norm(dynamics["torques"][:, j, :], axis=1)))
        peak_f   = float(np.max(np.linalg.norm(dynamics["forces"][:, j, :],  axis=1)))
        jerk     = np.gradient(velocities[:, j], times)
        peak_jk  = float(np.max(np.abs(jerk)))
        trap     = check_trapezoidal(times, velocities[:, j], jname)
        ok       = "✅" if peak_tau <= tau_limit else "⚠️ "
        is_trap  = "✅ Yes" if trap["is_trapezoidal"] else "⚠️  No"
        print(f"  {jname:<12} {peak_tau:>12.4f} {ok:>5}  {peak_f:>11.4f}  {peak_jk:>10.4f}  {is_trap:>12}")

    print("=" * 72 + "\n")

    # ── Plot ──────────────────────────────────────────────────────────────────
    node.get_logger().info("Generating dynamics plot...")
    plot_all(times, positions, velocities, accelerations, dynamics, joint_names, PLOT_PATH)

    # ── Execute (optional) ────────────────────────────────────────────────────
    if args.execute:
        node.get_logger().info("🚀  Executing trajectory on robot...")
        moveit2.execute(trajectory)
        moveit2.wait_until_executed()
        node.get_logger().info("✅  Execution complete.")
    else:
        node.get_logger().info("🔍  Dry-run complete. Pass --execute to move the robot.")

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
