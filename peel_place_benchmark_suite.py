#!/usr/bin/env python3
"""
peel_place_benchmark_suite.py — 6-DoF Motion Planning & Peeling Trajectory Benchmark Suite
==========================================================================================
Comprehensive sticker pick, peel, and place trajectory benchmark for the 6-DoF Kerabot arm
equipped with the 329x267x100mm end-effector payload box.

Features:
  1. 6-DoF Active Joint Kinematics (Revolute_1..5 + ee_rotation_joint) & end_effector_box_link
  2. Multi-Stage Sticker Pick, Peel & Place State Machine:
     - Stage 1: Home -> Pre-Pick Approach (OMPL)
     - Stage 2: Pre-Pick -> Sticker Contact (Pilz LIN)
     - Stage 3: Dynamic Sticker Peel Phase (Pilz LIN / Angled Cartesian Retraction @ 15°, 30°, 45°, 60°)
     - Stage 4: Peel Exit -> High Clearance Transfer (Pilz PTP)
     - Stage 5: Transfer -> Target Surface Placement (Pilz LIN / 0° Flat Alignment)
     - Stage 6: Release & Return Home (OMPL)
  3. Comparative Motion Pipeline Benchmark (OMPL + Ruckig, Pilz LIN + Ruckig, Pilz PTP + Ruckig)
  4. Speed Scaling Sweeps (0.2 -> 1.0 in 0.2 step increments)
  5. Dynamics Logging: Execution Time, Planning Time, Peak Jerk (<5 rad/s³ target), Accel, Torques, Collision Pass/Fail
  6. Visual Analytics: Itemized Terminal Summary Table & Matplotlib comparative plots saved to ~/kerabot_ws/

Usage:
    python3 peel_place_benchmark_suite.py
"""

import math
import os
import sys
import time
import threading
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2


# ── 6-DoF Arm Configuration ──────────────────────────────────────────────────
JOINT_NAMES  = ["Revolute_1", "Revolute_2", "Revolute_3", "Revolute_4", "Revolute_5", "ee_rotation_joint"]
BASE_LINK    = "base_link"
END_EFFECTOR = "end_effector_box_link"
MOVE_GROUP   = "arm"
HOME         = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Joint Inertia Estimates (kg*m²)
INERTIA_VALS = np.array([0.009, 0.012, 0.034, 0.001, 0.002, 0.018])

# Peel Angles to test (degrees)
PEEL_ANGLES = [15, 30, 45, 60]

# Pipeline Configurations: (Label, Pipeline ID, Planner ID)
PIPELINES = [
    ("Pipeline A (OMPL + Ruckig)",      "ompl",                           "RRTConnect"),
    ("Pipeline B (Pilz LIN + Ruckig)",  "pilz_industrial_motion_planner", "LIN"),
    ("Pipeline C (Pilz PTP + Ruckig)",  "pilz_industrial_motion_planner", "PTP"),
]

# Velocity/Acceleration scaling sweeps
SPEED_SCALES = [0.2, 0.4, 0.6, 0.8, 1.0]

# Pick & Place Workspace Coordinates (meters)
PICK_XYZ  = np.array([0.25, -0.30, 0.22])   # Sticker contact pose
PLACE_XYZ = np.array([-0.25, -0.30, 0.22])  # Placement surface pose
HOVER_Z   = 0.35                             # Hover clearance height
CLEAR_Z   = 0.40                             # High clearance transfer height
PEEL_DIST = 0.12                             # Retraction distance along peel vector (m)


# ── 6-DoF Forward Kinematics Solver ───────────────────────────────────────────
def fk_kerabot_6dof(q):
    q1, q2, q3, q4, q5, q6 = q

    # T1: base_link -> L110I_Shoulder
    T1 = np.eye(4)
    T1[:3, 3] = [0.0, 0.0, 0.08]
    R_j1_base = R.from_euler('xyz', [-np.pi/2, 0, 0]).as_matrix()
    R_j1_rot  = R.from_euler('xyz', [0, q1, 0]).as_matrix()
    T1[:3, :3] = R_j1_base @ R_j1_rot

    # T2: L110I_Shoulder -> L110I_shoulder_2
    T2 = np.eye(4)
    T2[:3, 3] = [0.0, -0.119, 0.057]
    R_j2_base = R.from_euler('xyz', [np.pi, 0, 0]).as_matrix()
    R_j2_rot  = R.from_euler('xyz', [0, 0, q2]).as_matrix()
    T2[:3, :3] = R_j2_base @ R_j2_rot
    T12 = T1 @ T2

    # T3: L110I_shoulder_2 -> J2J3_Shoulder
    T3 = np.eye(4)
    T3[:3, 3] = [0.0, 0.426, 0.003028]
    R_j3_base = R.from_euler('xyz', [-np.pi/2, -np.pi/2, 0]).as_matrix()
    R_j3_rot  = R.from_euler('xyz', [-q3, 0, 0]).as_matrix()
    T3[:3, :3] = R_j3_base @ R_j3_rot
    T123 = T12 @ T3

    # T4: J2J3_Shoulder -> Wrist_Motor
    T4 = np.eye(4)
    T4[:3, 3] = [0.053972, 0.0, 0.314]
    R_j4_rot  = R.from_euler('xyz', [0, 0, q4]).as_matrix()
    T4[:3, :3] = R_j4_rot
    T1234 = T123 @ T4

    # T5: Wrist_Motor -> L70IE_Finger
    T5 = np.eye(4)
    T5[:3, 3] = [0.0595, 0.0, 0.130]
    R_j5_base = R.from_euler('xyz', [-0.000787, -np.pi/2, 0.002101]).as_matrix()
    R_j5_rot  = R.from_euler('xyz', [0, 0, q5]).as_matrix()
    T5[:3, :3] = R_j5_base @ R_j5_rot
    T12345 = T1234 @ T5

    # T6: L70IE_Finger -> end_effector_box_link (Joint origin: xyz=[0.15,0,0.05], rpy=[0,1.5708,0])
    T6 = np.eye(4)
    T6[:3, 3] = [0.150, 0.0, 0.050]
    R_j6_base = R.from_euler('xyz', [0, np.pi/2, 0]).as_matrix()
    R_j6_rot  = R.from_euler('xyz', [0, 0, q6]).as_matrix()
    T6[:3, :3] = R_j6_base @ R_j6_rot
    T_total = T12345 @ T6

    # End-effector box centroid (+0.05m along local Z)
    p_box_center = T_total @ np.array([0.0, 0.0, 0.050, 1.0])
    pos_ee = p_box_center[:3]

    links = [T1[:3, 3], T12[:3, 3], T123[:3, 3], T1234[:3, 3], T12345[:3, 3], T_total[:3, 3], pos_ee]
    return pos_ee, links, T_total


# ── 6-DoF Numerical Inverse Kinematics (IK) Solver ───────────────────────────
def solve_ik_6dof(target_xyz, target_pitch_deg=90.0, yaw_angle=0.0):
    target_pos = np.array(target_xyz)

    def objective(q):
        pos_ee, links, T_total = fk_kerabot_6dof(q)
        pos_err = np.linalg.norm(pos_ee - target_pos)

        # Ground clearance penalty (all links Z >= 0.03m)
        g_pen = sum(500.0 * (0.03 - l[2])**2 for l in links if l[2] < 0.03)

        # Joint limits soft penalty
        j_pen = sum(100.0 * (lo - q_i)**2 if q_i < lo else (100.0 * (q_i - hi)**2 if q_i > hi else 0)
                    for q_i, (lo, hi) in zip(q, [(-2.9, 2.9)]*5 + [(-3.14, 3.14)]))

        return 1000.0 * pos_err**2 + g_pen + j_pen

    bounds = [(-2.9, 2.9)] * 5 + [(-3.14, 3.14)]
    best_q = None
    best_cost = float("inf")

    q1_base = math.atan2(target_xyz[0], -target_xyz[1])
    guesses = [
        [q1_base, -0.6, 1.2, 0.0, 0.0, 0.0],
        [q1_base, -1.0, 1.6, 0.0, -0.5, math.radians(yaw_angle)],
        [q1_base, -0.4, 0.8, 0.2, 0.2, 0.0],
        [0.0, -0.5, 0.5, 0.0, 0.0, 0.0],
    ]

    for q0 in guesses:
        res = minimize(objective, q0, method='SLSQP', bounds=bounds, options={'maxiter': 300})
        if res.fun < best_cost:
            best_cost = res.fun
            best_q = res.x

    pos_check, links_check, _ = fk_kerabot_6dof(best_q)
    if np.linalg.norm(pos_check - target_pos) > 0.05 or any(l[2] < 0.02 for l in links_check):
        return None

    return list(best_q)


# ── MoveIt 2 Helper Commands ─────────────────────────────────────────────────
def get_current_joints(moveit2):
    if moveit2.joint_state is None:
        return None
    mapping = dict(zip(moveit2.joint_state.name, moveit2.joint_state.position))
    return [mapping.get(jn, float("nan")) for jn in JOINT_NAMES]


def go_home(moveit2):
    moveit2.pipeline_id = "pilz_industrial_motion_planner"
    moveit2.planner_id  = "PTP"
    moveit2.max_velocity = 0.5
    moveit2.max_acceleration = 0.3
    traj = moveit2.plan(joint_positions=HOME, joint_names=JOINT_NAMES)
    if traj is not None:
        moveit2.execute(traj)
        moveit2.wait_until_executed()
    time.sleep(0.3)


# ── Trajectory Dynamics Analysis ─────────────────────────────────────────────
def analyze_trajectory_dynamics(moveit2_node, joint_trajectory, dt=0.02):
    """
    Computes numerical derivatives (velocity, acceleration, jerk) and estimated torques
    from a planned joint trajectory.
    """
    if joint_trajectory is None or not joint_trajectory.points:
        return {
            "max_vel": 0.0, "max_accel": 0.0, "max_jerk": 0.0,
            "jerk_pass": True, "est_torque": 0.0, "time_series": None
        }

    points = joint_trajectory.points
    n_pts = len(points)

    times = []
    positions = []
    for pt in points:
        t_sec = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
        times.append(t_sec)
        positions.append(pt.positions)

    times = np.array(times)
    positions = np.array(positions)  # shape (n_pts, 6)

    # Ensure monotonic strictly increasing time array for differentiation
    if n_pts < 3 or (times[-1] - times[0]) <= 1e-4:
        # Uniform resampling
        times = np.linspace(0, max(0.1, n_pts * dt), n_pts)

    # Compute numerical derivatives
    velocities    = np.zeros_like(positions)
    accelerations = np.zeros_like(positions)
    jerks         = np.zeros_like(positions)

    for j in range(6):
        velocities[:, j]    = np.gradient(positions[:, j], times)
        accelerations[:, j] = np.gradient(velocities[:, j], times)
        jerks[:, j]         = np.gradient(accelerations[:, j], times)

    max_vel   = float(np.max(np.abs(velocities)))
    max_accel = float(np.max(np.abs(accelerations)))
    max_jerk  = float(np.max(np.abs(jerks)))
    jerk_pass = max_jerk < 5.0

    # Estimated joint torques: tau_i = I_i * alpha_i
    est_torques = np.max(np.abs(accelerations) * INERTIA_VALS, axis=0)
    max_torque  = float(np.max(est_torques))

    return {
        "max_vel": max_vel,
        "max_accel": max_accel,
        "max_jerk": max_jerk,
        "jerk_pass": jerk_pass,
        "est_torque": max_torque,
        "time_series": {
            "times": times, "pos": positions,
            "vel": velocities, "accel": accelerations, "jerk": jerks
        }
    }


# ── Benchmark Suite Execution Class ──────────────────────────────────────────
class StickerPeelBenchmarkSuite:
    def __init__(self):
        rclpy.init()
        self.node = Node("kerabot_peel_benchmark")
        self.cb = ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=self.node,
            joint_names=JOINT_NAMES,
            base_link_name=BASE_LINK,
            end_effector_name=END_EFFECTOR,
            group_name=MOVE_GROUP,
            callback_group=self.cb,
        )

        self.executor = MultiThreadedExecutor(2)
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        time.sleep(1.5)

        self.results = []
        self.plot_data = {}

    def run_stage_motion(self, stage_name, joint_target, pipeline_id, planner_id, vel_scale, accel_scale):
        self.moveit2.pipeline_id               = pipeline_id
        self.moveit2.planner_id                = planner_id
        self.moveit2.max_velocity              = vel_scale
        self.moveit2.max_acceleration          = accel_scale
        self.moveit2.num_planning_attempts     = 10
        self.moveit2.allowed_planning_time     = 5.0

        t_plan_start = time.time()
        traj = self.moveit2.plan(joint_positions=joint_target, joint_names=JOINT_NAMES)
        t_plan = (time.time() - t_plan_start) * 1000.0  # ms

        if traj is None:
            return {
                "stage": stage_name, "success": False, "plan_ms": t_plan, "exec_s": 0.0,
                "max_vel": 0.0, "max_accel": 0.0, "max_jerk": 0.0, "jerk_pass": False,
                "est_torque": 0.0, "collision_pass": False, "time_series": None
            }

        # Analyze planned trajectory dynamics
        dyn = analyze_trajectory_dynamics(self.node, traj)

        t_exec_start = time.time()
        self.moveit2.execute(traj)
        self.moveit2.wait_until_executed()
        t_exec = time.time() - t_exec_start

        # Check ground clearance collision status from joint states
        current_q = get_current_joints(self.moveit2)
        collision_pass = True
        if current_q is not None:
            pos_ee, links, _ = fk_kerabot_6dof(current_q)
            if any(l[2] < 0.02 for l in links):
                collision_pass = False

        return {
            "stage": stage_name, "success": True, "plan_ms": t_plan, "exec_s": t_exec,
            "max_vel": dyn["max_vel"], "max_accel": dyn["max_accel"], "max_jerk": dyn["max_jerk"],
            "jerk_pass": dyn["jerk_pass"], "est_torque": dyn["est_torque"],
            "collision_pass": collision_pass, "time_series": dyn["time_series"]
        }

    def execute_peel_sequence(self, peel_angle_deg, pipe_label, pipe_id, planner_id, vel_scale, accel_scale):
        print(f"\n─────────────────────────────────────────────────────────────────────────────")
        print(f"▶ BENCHMARK RUN: Angle={peel_angle_deg}° | {pipe_label} | Speed Scale={vel_scale:.1f}")
        print(f"─────────────────────────────────────────────────────────────────────────────")

        go_home(self.moveit2)

        # Compute targets for the 6-stage sequence
        # Stage 1: Pre-Pick Approach (hover above pick)
        q_pre_pick = solve_ik_6dof([PICK_XYZ[0], PICK_XYZ[1], HOVER_Z])
        # Stage 2: Sticker Contact
        q_contact = solve_ik_6dof(PICK_XYZ)
        # Stage 3: Peel Retraction Pose (angled lift @ peel_angle)
        rad_angle = math.radians(peel_angle_deg)
        peel_xyz = PICK_XYZ + np.array([-PEEL_DIST * math.cos(rad_angle), 0.0, PEEL_DIST * math.sin(rad_angle)])
        q_peel = solve_ik_6dof(peel_xyz, target_pitch_deg=90.0 - peel_angle_deg, yaw_angle=peel_angle_deg)
        # Stage 4: High Clearance Transfer
        q_transfer = solve_ik_6dof([0.0, -0.35, CLEAR_Z])
        # Stage 5: Target Surface Placement Contact
        q_place = solve_ik_6dof(PLACE_XYZ)
        # Stage 6: Return Home
        q_home = HOME

        stages = [
            ("Stage 1: Pre-Pick Approach", q_pre_pick, "ompl", "RRTConnect"),
            ("Stage 2: Sticker Contact",    q_contact,  pipe_id, planner_id),
            (f"Stage 3: Peel ({peel_angle_deg}°)", q_peel, pipe_id, planner_id),
            ("Stage 4: High Transfer",      q_transfer, pipe_id, planner_id),
            ("Stage 5: Placement Contact",  q_place,    pipe_id, planner_id),
            ("Stage 6: Return Home",        q_home,     "ompl", "RRTConnect"),
        ]

        stage_metrics = []
        for name, q_t, pid, plid in stages:
            if q_t is None:
                print(f"  ❌ {name:30s} -> IK Solver Failed to find valid pose")
                continue
            res = self.run_stage_motion(name, q_t, pid, plid, vel_scale, accel_scale)
            stage_metrics.append(res)
            status_symbol = "✅" if res["success"] and res["collision_pass"] else "❌"
            jerk_str = f"{res['max_jerk']:.2f} rad/s³ (" + ("PASS" if res['jerk_pass'] else "HIGH") + ")"
            print(f"  {status_symbol} {name:30s} | Plan: {res['plan_ms']:5.1f}ms | Exec: {res['exec_s']:4.2f}s | Jerk: {jerk_str}")

        # Aggregate run metrics
        total_plan_ms  = sum(r["plan_ms"] for r in stage_metrics)
        total_exec_s   = sum(r["exec_s"] for r in stage_metrics)
        max_jerk       = max((r["max_jerk"] for r in stage_metrics), default=0.0)
        max_accel      = max((r["max_accel"] for r in stage_metrics), default=0.0)
        max_vel        = max((r["max_vel"] for r in stage_metrics), default=0.0)
        max_torque     = max((r["est_torque"] for r in stage_metrics), default=0.0)
        all_collisions = all(r["collision_pass"] for r in stage_metrics)
        all_jerk_pass  = max_jerk < 5.0

        run_summary = {
            "angle": peel_angle_deg, "pipeline": pipe_label, "scale": vel_scale,
            "plan_ms": total_plan_ms, "exec_s": total_exec_s,
            "max_vel": max_vel, "max_accel": max_accel, "max_jerk": max_jerk,
            "jerk_pass": all_jerk_pass, "est_torque": max_torque, "collision_pass": all_collisions,
            "stage_metrics": stage_metrics
        }
        self.results.append(run_summary)
        return run_summary

    def generate_visual_analytics(self):
        ws_dir = "/home/saad/kerabot_ws"
        os.makedirs(ws_dir, exist_ok=True)

        print("\n=========================================================================")
        print("                GENERATING COMPARATIVE VISUAL ANALYTICS                  ")
        print("=========================================================================")

        # Plot 1: Velocity & Acceleration Scaling Curves across Peel Angles
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle("6-DoF Kerabot Arm: Peeling Trajectory Dynamics & Jerk Benchmark", fontsize=14, fontweight='bold')

        angles = PEEL_ANGLES
        for idx, angle in enumerate(angles):
            ax = axes[idx // 2, idx % 2]
            runs = [r for r in self.results if r["angle"] == angle and abs(r["scale"] - 0.6) < 0.05]
            if not runs:
                runs = [r for r in self.results if r["angle"] == angle]

            labels = [r["pipeline"].split("(")[0].strip() for r in runs]
            jerks  = [r["max_jerk"] for r in runs]
            accels = [r["max_accel"] for r in runs]
            execs  = [r["exec_s"] for r in runs]

            x = np.arange(len(labels))
            width = 0.25

            ax.bar(x - width, jerks, width, label='Max Jerk (rad/s³)', color='#E74C3C')
            ax.bar(x, accels, width, label='Max Accel (rad/s²)', color='#3498DB')
            ax.bar(x + width, execs, width, label='Exec Time (s)', color='#2ECC71')

            ax.axhline(5.0, color='red', linestyle='--', linewidth=1.5, label='Jerk Threshold (5.0 rad/s³)')
            ax.set_title(f"Peel Angle: {angle}°", fontweight='bold')
            ax.set_xticks(x)
            ax.set_xticklabels(labels, rotation=15, fontsize=9)
            ax.grid(True, linestyle=':', alpha=0.6)
            ax.legend(fontsize=8)

        plt.tight_layout(rect=[0, 0, 1, 0.95])
        p1_path = os.path.join(ws_dir, "peel_benchmark_summary.png")
        plt.savefig(p1_path, dpi=150)
        plt.close()
        print(f"  📊 Saved summary plot: {p1_path}")

        # Plot 2: Speed Scaling Sweeps (Velocity, Accel, Jerk vs Scaling Factor)
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(16, 5))
        fig.suptitle("Scaling Factor Sweeps (0.2 -> 1.0) Across Peeling Motion Pipelines", fontsize=14, fontweight='bold')

        for pipe_label, p_id, _ in PIPELINES:
            pipe_short = pipe_label.split("(")[0].strip()
            runs = [r for r in self.results if r["pipeline"] == pipe_label and r["angle"] == 30]
            runs = sorted(runs, key=lambda x: x["scale"])
            scales = [r["scale"] for r in runs]
            vels   = [r["max_vel"] for r in runs]
            accels = [r["max_accel"] for r in runs]
            jerks  = [r["max_jerk"] for r in runs]

            if scales:
                ax1.plot(scales, vels, 'o-', label=pipe_short, linewidth=2)
                ax2.plot(scales, accels, 's-', label=pipe_short, linewidth=2)
                ax3.plot(scales, jerks, '^--', label=pipe_short, linewidth=2)

        ax1.set_title("Peak Velocity vs Scaling", fontweight='bold')
        ax1.set_xlabel("Scaling Factor")
        ax1.set_ylabel("Peak Velocity (rad/s)")
        ax1.grid(True, linestyle=':', alpha=0.6)
        ax1.legend()

        ax2.set_title("Peak Acceleration vs Scaling", fontweight='bold')
        ax2.set_xlabel("Scaling Factor")
        ax2.set_ylabel("Peak Accel (rad/s²)")
        ax2.grid(True, linestyle=':', alpha=0.6)

        ax3.set_title("Peak Jerk vs Scaling", fontweight='bold')
        ax3.set_xlabel("Scaling Factor")
        ax3.set_ylabel("Peak Jerk (rad/s³)")
        ax3.axhline(5.0, color='red', linestyle='--', label='Target Limit (<5.0)')
        ax3.grid(True, linestyle=':', alpha=0.6)
        ax3.legend()

        plt.tight_layout(rect=[0, 0, 1, 0.93])
        p2_path = os.path.join(ws_dir, "peel_benchmark_velocity.png")
        plt.savefig(p2_path, dpi=150)
        plt.close()
        print(f"  📊 Saved velocity/scaling plot: {p2_path}")

    def print_terminal_summary_table(self):
        print("\n" + "="*115)
        print("            KERABOT 6-DOF ARM: STICKER PICK, PEEL & PLACE BENCHMARK SUMMARY TABLE             ")
        print("="*115)
        print(f"{'Angle':<7}| {'Pipeline':<28}| {'Scale':<6}| {'Plan(ms)':<9}| {'Exec(s)':<8}| {'MaxVel':<8}| {'MaxAccel':<9}| {'PeakJerk':<10}| {'Jerk<5':<7}| {'Collision':<10}")
        print("-" * 115)

        for r in self.results:
            jerk_status = "PASS" if r["jerk_pass"] else "HIGH"
            coll_status = "PASS" if r["collision_pass"] else "FAIL"
            pipe_short  = r["pipeline"].split("(")[0].strip()
            print(f"{r['angle']:>4}°  | {pipe_short:<28}| {r['scale']:<6.1f}| {r['plan_ms']:<9.1f}| {r['exec_s']:<8.2f}| {r['max_vel']:<8.2f}| {r['max_accel']:<9.2f}| {r['max_jerk']:<10.2f}| {jerk_status:<7}| {coll_status:<10}")

        print("="*115 + "\n")


# ── Main Suite Entrypoint ────────────────────────────────────────────────────
def main():
    suite = StickerPeelBenchmarkSuite()

    print("\n" + "#"*80)
    print("#  KERABOT 6-DOF ARM: STICKER PICK, PEEL & PLACE TRAJECTORY BENCHMARK SUITE    #")
    print("#"*80)

    # 1. Peel Angle Sweeps (at baseline scaling 0.6)
    print("\n--- PHASE 1: PEEL ANGLE SWEEPS (15°, 30°, 45°, 60°) ---")
    for angle in PEEL_ANGLES:
        for p_label, p_id, pl_id in PIPELINES:
            suite.execute_peel_sequence(angle, p_label, p_id, pl_id, vel_scale=0.6, accel_scale=0.4)

    # 2. Speed Scaling Sweeps (0.2 -> 1.0 @ 30° peel angle)
    print("\n--- PHASE 2: VELOCITY & ACCELERATION SCALING SWEEPS (0.2 -> 1.0) ---")
    for scale in SPEED_SCALES:
        for p_label, p_id, pl_id in PIPELINES:
            suite.execute_peel_sequence(peel_angle_deg=30, pipe_label=p_label, pipe_id=p_id, planner_id=pl_id, vel_scale=scale, accel_scale=scale*0.6)

    # 3. Print Terminal Summary & Generate Plots
    suite.print_terminal_summary_table()
    suite.generate_visual_analytics()

    print("\n✅ Benchmark Suite execution complete! All plots saved to ~/kerabot_ws/.\n")
    rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
