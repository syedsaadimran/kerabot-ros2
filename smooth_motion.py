#!/usr/bin/env python3
"""
smooth_motion.py — Pilz PTP + Ruckig Smoothing: Before/After Comparison
=========================================================================
Plans the same motion at multiple velocity/acceleration combinations and
generates a detailed 4-panel comparison plot showing the effect of Ruckig
jerk-limiting on the trapezoidal profile.

With Ruckig enabled in ompl_planning.yaml, MoveIt automatically applies
Ruckig smoothing to every Pilz plan. This script sweeps multiple scaling
combos so you can see how jerk behaves across the speed/smoothness tradeoff.

Output files:
  ~/kerabot_ws/ruckig_comparison.png   — 4-panel per-joint view
  ~/kerabot_ws/ruckig_stats.png        — summary: jerk reduction bar chart

Usage:
    # Dry-run (plan only, no movement):
    python3 smooth_motion.py

    # Execute the balanced combo on the robot:
    python3 smooth_motion.py --execute

    # Go home first (use when robot is already at target):
    python3 smooth_motion.py --home

    # Slower/safer:
    python3 smooth_motion.py --vel 0.3 --accel 0.2
"""

import argparse
import threading
import time

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2


# ─────────────────────────────────────────────────────────────────────────────
# ROBOT CONFIG
# ─────────────────────────────────────────────────────────────────────────────
JOINT_NAMES      = ["Revolute_1", "Revolute_2", "Revolute_3", "Revolute_4", "Revolute_5"]
BASE_LINK        = "base_link"
END_EFFECTOR     = "L70IE_Finger"
MOVE_GROUP       = "arm"
TARGET_POSITION  = [0.0, -0.0595, 0.65]
TARGET_QUAT_XYZW = [-0.4997, -0.5003, -0.4997, 0.5003]

OUT_COMPARISON = "/home/saad/kerabot_ws/ruckig_comparison.png"
OUT_STATS      = "/home/saad/kerabot_ws/ruckig_stats.png"

# Sweep combos for the multi-speed analysis
COMBOS = [
    (1.0, 0.8, "Max Speed\n(vel=1.0, accel=0.8)",      "#E63946"),
    (0.7, 0.5, "Fast\n(vel=0.7, accel=0.5)",           "#F4A261"),
    (0.5, 0.3, "Balanced — default\n(vel=0.5, accel=0.3)", "#2A9D8F"),
    (0.3, 0.2, "Smooth\n(vel=0.3, accel=0.2)",         "#457B9D"),
    (0.2, 0.1, "Ultra Smooth\n(vel=0.2, accel=0.1)",   "#A8DADC"),
]

BG, PANEL, PANEL2, GRID = "#0f0f1a", "#1a1a2e", "#16213e", "#2a2a4a"
TCOL, TCOL2             = "#d0d0f0", "#8888aa"
JOINT_COLORS = ["#E63946", "#F4A261", "#2A9D8F", "#457B9D", "#A8DADC"]


# ─────────────────────────────────────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────────────────────────────────────
def extract_arrays(trajectory):
    """Return (times, positions, velocities, accelerations, jerk) as np arrays."""
    pts   = trajectory.points
    n     = len(trajectory.joint_names)
    times = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                      for p in pts])
    pos   = np.array([[p.positions[j]     for j in range(n)] for p in pts])
    vel   = np.array([[p.velocities[j]    if p.velocities    else 0.0
                       for j in range(n)] for p in pts])
    acc   = np.array([[p.accelerations[j] if p.accelerations else 0.0
                       for j in range(n)] for p in pts])
    jerk  = np.gradient(vel, times, axis=0)
    return times, pos, vel, acc, jerk


def style_ax(ax, title, ylabel):
    ax.set_facecolor(PANEL)
    ax.set_title(title, color=TCOL, fontsize=9, pad=4, fontweight="bold")
    ax.set_ylabel(ylabel, color=TCOL2, fontsize=8)
    ax.tick_params(colors=TCOL2, labelsize=7)
    for sp in ax.spines.values():
        sp.set_color(GRID)
    ax.grid(True, color=GRID, linewidth=0.5, alpha=0.8)


# ─────────────────────────────────────────────────────────────────────────────
# FIGURE 1 — 4-panel per-joint velocity/jerk comparison
# ─────────────────────────────────────────────────────────────────────────────
def plot_comparison(results, joint_names):
    """
    For each of the 5 joints:
      Row A: velocity profile (trapezoidal shape + Ruckig-smoothed corners)
      Row B: jerk profile (lower = smoother)
    All combos overlaid in different colours.
    """
    n_joints = len(joint_names)
    fig = plt.figure(figsize=(20, 22), facecolor=BG)
    fig.suptitle(
        "Kerabot — Pilz PTP + Ruckig Smoothing\n"
        "Velocity Profiles & Jerk per Joint across Speed Combos",
        fontsize=14, color=TCOL, fontweight="bold", y=0.995
    )

    # 5 joints × 2 rows (vel + jerk)  →  10 subplot rows
    gs = gridspec.GridSpec(n_joints * 2, 1, figure=fig,
                           hspace=0.65, left=0.09, right=0.97,
                           top=0.96, bottom=0.03)

    for j, jname in enumerate(joint_names):
        ax_vel  = fig.add_subplot(gs[j * 2])
        ax_jerk = fig.add_subplot(gs[j * 2 + 1])

        style_ax(ax_vel,  f"{jname} — Velocity (rad/s)",  "Velocity (rad/s)")
        style_ax(ax_jerk, f"{jname} — Jerk (rad/s³)  [lower = smoother]", "Jerk (rad/s³)")

        for (vel, accel, label, color), res in zip(COMBOS, results):
            if res is None:
                continue
            times, _, vel_arr, _, jerk_arr = res
            lbl = label.replace("\n", "  ")
            ax_vel.plot( times, vel_arr[:, j],  color=color, lw=2.0, label=lbl)
            ax_jerk.plot(times, jerk_arr[:, j], color=color, lw=2.0, label=lbl)
            ax_jerk.axhline(0, color=GRID, lw=0.6)

        ax_vel.legend( loc="upper right", fontsize=7, labelcolor=TCOL, framealpha=0.2, ncol=2)
        ax_jerk.legend(loc="upper right", fontsize=7, labelcolor=TCOL, framealpha=0.2, ncol=2)

        if j == n_joints - 1:
            ax_jerk.set_xlabel("Time (s)", color=TCOL2, fontsize=8)

    plt.savefig(OUT_COMPARISON, dpi=150, bbox_inches="tight", facecolor=BG)
    print(f"[1/2] Saved velocity/jerk comparison → {OUT_COMPARISON}")


# ─────────────────────────────────────────────────────────────────────────────
# FIGURE 2 — Stats: duration, avg jerk, peak jerk
# ─────────────────────────────────────────────────────────────────────────────
def plot_stats(results, joint_names):
    valid = [(c, r) for c, r in zip(COMBOS, results) if r is not None]
    if not valid:
        return

    labels   = [c[2].replace("\n", " ") for c, _ in valid]
    colors   = [c[3] for c, _ in valid]
    n        = len(valid)
    bar_w    = 0.14
    joint_x  = np.arange(len(joint_names))

    fig = plt.figure(figsize=(20, 12), facecolor=BG)
    fig.suptitle("Kerabot — Pilz PTP + Ruckig  |  Stats Comparison",
                 fontsize=14, color=TCOL, fontweight="bold", y=0.99)
    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.55, wspace=0.38,
                           left=0.07, right=0.98, top=0.93, bottom=0.23)

    # Duration
    ax_dur = fig.add_subplot(gs[0, 0])
    style_ax(ax_dur, "Motion Duration", "Duration (s)")
    durations = [r[0][-1] for _, r in valid]
    bars = ax_dur.bar(range(n), durations, color=colors, width=0.6, edgecolor=GRID)
    ax_dur.set_xticks(range(n))
    ax_dur.set_xticklabels([f"C{i+1}" for i in range(n)], color=TCOL2, fontsize=8)
    for bar, dur in zip(bars, durations):
        ax_dur.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.03,
                    f"{dur:.2f}s", ha="center", va="bottom",
                    color=TCOL, fontsize=8, fontweight="bold")
    ax_dur.set_ylim(0, max(durations) * 1.2)

    # Peak jerk per joint
    ax_pk = fig.add_subplot(gs[0, 1])
    style_ax(ax_pk, "Peak Jerk per Joint  [lower = smoother]", "Peak Jerk (rad/s³)")
    for i, ((c, res), color) in enumerate(zip(valid, colors)):
        _, _, _, _, jerk_arr = res
        peak_j = [float(np.max(np.abs(jerk_arr[:, j]))) for j in range(len(joint_names))]
        off = (i - n / 2 + 0.5) * bar_w
        ax_pk.bar(joint_x + off, peak_j, width=bar_w, color=color,
                  edgecolor=GRID, alpha=0.9, label=f"C{i+1}")
    ax_pk.set_xticks(joint_x)
    ax_pk.set_xticklabels(joint_names, color=TCOL2, fontsize=7, rotation=15)
    ax_pk.legend(fontsize=7, labelcolor=TCOL, framealpha=0.2)

    # Avg jerk per joint
    ax_avg = fig.add_subplot(gs[0, 2])
    style_ax(ax_avg, "Avg Jerk per Joint  (RMS)", "Avg Jerk (rad/s³)")
    for i, ((c, res), color) in enumerate(zip(valid, colors)):
        _, _, _, _, jerk_arr = res
        avg_j = [float(np.sqrt(np.mean(jerk_arr[:, j] ** 2)))
                 for j in range(len(joint_names))]
        off = (i - n / 2 + 0.5) * bar_w
        ax_avg.bar(joint_x + off, avg_j, width=bar_w, color=color,
                   edgecolor=GRID, alpha=0.9, label=f"C{i+1}")
    ax_avg.set_xticks(joint_x)
    ax_avg.set_xticklabels(joint_names, color=TCOL2, fontsize=7, rotation=15)
    ax_avg.legend(fontsize=7, labelcolor=TCOL, framealpha=0.2)

    # Speed vs smoothness scatter
    ax_sc = fig.add_subplot(gs[1, 0])
    style_ax(ax_sc, "Speed vs Smoothness Trade-off\n(ideal = top-left)", "Avg RMS Jerk")
    for i, ((c, res), color) in enumerate(zip(valid, colors)):
        _, _, _, _, jerk_arr = res
        avg_rms = float(np.mean([np.sqrt(np.mean(jerk_arr[:, j] ** 2))
                                 for j in range(len(joint_names))]))
        ax_sc.scatter(res[0][-1], avg_rms, s=160, color=color,
                      edgecolors="white", linewidth=1.0, zorder=5)
        ax_sc.annotate(f"C{i+1}", (res[0][-1], avg_rms),
                       textcoords="offset points", xytext=(6, 4),
                       color=color, fontsize=9, fontweight="bold")
    ax_sc.set_xlabel("Duration (s)", color=TCOL2, fontsize=8)

    # Max velocity per joint
    ax_vel = fig.add_subplot(gs[1, 1])
    style_ax(ax_vel, "Peak Velocity per Joint", "Velocity (rad/s)")
    for i, ((c, res), color) in enumerate(zip(valid, colors)):
        _, _, vel_arr, _, _ = res
        peak_v = [float(np.max(np.abs(vel_arr[:, j]))) for j in range(len(joint_names))]
        off = (i - n / 2 + 0.5) * bar_w
        ax_vel.bar(joint_x + off, peak_v, width=bar_w, color=color,
                   edgecolor=GRID, alpha=0.9, label=f"C{i+1}")
    ax_vel.set_xticks(joint_x)
    ax_vel.set_xticklabels(joint_names, color=TCOL2, fontsize=7, rotation=15)
    ax_vel.legend(fontsize=7, labelcolor=TCOL, framealpha=0.2)

    # Summary table
    scores = []
    max_dur = max(r[0][-1] for _, r in valid)
    max_jrk = max(float(np.mean([np.sqrt(np.mean(r[4][:, j]**2))
                                  for j in range(len(joint_names))]))
                  for _, r in valid)
    for _, res in valid:
        nd = res[0][-1] / max_dur
        nj = float(np.mean([np.sqrt(np.mean(res[4][:, j]**2))
                             for j in range(len(joint_names))])) / max_jrk
        scores.append(round(0.4 * nd + 0.6 * nj, 4))

    best = int(np.argmin(scores))
    ax_leg = fig.add_subplot(gs[1, 2])
    ax_leg.set_facecolor(PANEL2)
    ax_leg.axis("off")
    ax_leg.set_title("Combo Legend + Recommendation", color=TCOL,
                     fontsize=9, pad=6, fontweight="bold")
    for i, ((c, _), color) in enumerate(zip(valid, colors)):
        tag  = " <-- RECOMMENDED" if i == best else ""
        text = f"C{i+1}: {c[2].replace(chr(10), '  ')}{tag}"
        ax_leg.text(0.04, 0.88 - i * 0.16, text,
                    transform=ax_leg.transAxes,
                    color=color if i != best else "#00ff88",
                    fontsize=8,
                    fontweight="bold" if i == best else "normal")

    # Bottom table
    col_labels = ["Combo", "vel", "accel", "Duration", "Avg RMS Jerk", "Score"]
    rows = []
    for i, ((c, res), score) in enumerate(zip(valid, scores)):
        avg_rms = float(np.mean([np.sqrt(np.mean(res[4][:, j]**2))
                                  for j in range(len(joint_names))]))
        rows.append([
            f"C{i+1} — {c[2].split(chr(10))[0]}",
            f"{c[0]:.1f}", f"{c[1]:.1f}",
            f"{res[0][-1]:.3f}s", f"{avg_rms:.4f}",
            f"{score}  {'<<' if i == best else ''}",
        ])

    tbl_ax = fig.add_axes([0.04, 0.01, 0.92, 0.16])
    tbl_ax.set_facecolor(BG)
    tbl_ax.axis("off")
    tbl = tbl_ax.table(cellText=rows, colLabels=col_labels,
                       cellLoc="center", loc="center")
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8.5)
    tbl.scale(1, 1.7)
    for (r, c), cell in tbl.get_celld().items():
        is_best = (r > 0 and r - 1 == best)
        cell.set_facecolor("#2a3a2a" if is_best else (PANEL if r % 2 == 0 else PANEL2))
        cell.set_text_props(color="#00ff88" if is_best else TCOL)
        cell.set_edgecolor(GRID)

    plt.savefig(OUT_STATS, dpi=150, bbox_inches="tight", facecolor=BG)
    print(f"[2/2] Saved stats summary       → {OUT_STATS}")


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="Kerabot Pilz PTP + Ruckig comparison plotter")
    p.add_argument("--execute",   action="store_true",
                   help="Execute the balanced (vel=0.5, accel=0.3) combo on the robot")
    p.add_argument("--home",      action="store_true",
                   help="Return to home position before planning")
    p.add_argument("--vel",       type=float, default=None,
                   help="Override: run a single combo at this vel scale")
    p.add_argument("--accel",     type=float, default=None,
                   help="Override: run a single combo at this accel scale")
    p.add_argument("--cartesian", action="store_true",
                   help="Use Pilz LIN (Cartesian straight-line) instead of PTP")
    return p.parse_args()


def main():
    args  = parse_args()
    rclpy.init()

    node = Node("kerabot_ruckig_comparison")
    cb   = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name=BASE_LINK,
        end_effector_name=END_EFFECTOR,
        group_name=MOVE_GROUP,
        callback_group=cb,
    )
    moveit2.pipeline_id           = "pilz_industrial_motion_planner"
    moveit2.planner_id            = "LIN" if args.cartesian else "PTP"
    moveit2.num_planning_attempts = 10
    moveit2.allowed_planning_time = 10.0

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.0)

    if args.home:
        node.get_logger().info("Returning to home position first...")
        moveit2.move_to_configuration([0.0, 0.0, 0.0, 0.0, 0.0])
        moveit2.wait_until_executed()
        node.get_logger().info("Home reached.")
        time.sleep(0.5)

    # Single override or full sweep
    combos = COMBOS
    if args.vel is not None and args.accel is not None:
        combos = [(args.vel, args.accel,
                   f"Custom (vel={args.vel}, accel={args.accel})", "#E63946")]

    planner_name = "LIN" if args.cartesian else "PTP"
    print(f"\nPlanner : Pilz {planner_name} + Ruckig Smoothing")
    print(f"Target  : {TARGET_POSITION}")
    print(f"Combos  : {len(combos)}\n")
    print(f"{'#':<4} {'vel':<6} {'accel':<6} {'Pts':>5} {'Duration':>9}  "
          f"{'PeakJerk(J3)':>13}  Status")
    print("-" * 60)

    results = []
    exec_traj = None

    for i, (vel, accel, label, color) in enumerate(combos):
        moveit2.max_velocity    = vel
        moveit2.max_acceleration = accel

        traj = moveit2.plan(
            position=TARGET_POSITION,
            quat_xyzw=TARGET_QUAT_XYZW,
            cartesian=args.cartesian,
        )

        if traj is None or len(traj.points) < 2:
            print(f"  C{i+1}  {vel:<6} {accel:<6} {'--':>5}  -- FAILED / already at target")
            results.append(None)
            continue

        times, pos, vel_arr, acc_arr, jerk_arr = extract_arrays(traj)

        # J3 (Revolute_3) is typically most active — report its jerk
        j3_idx   = 2
        peak_jk3 = float(np.max(np.abs(jerk_arr[:, j3_idx])))
        print(f"  C{i+1}  {vel:<6} {accel:<6} {len(traj.points):>5}  "
              f"{times[-1]:>9.3f}  {peak_jk3:>13.3f}  OK")

        results.append((times, pos, vel_arr, acc_arr, jerk_arr))

        # Save balanced combo trajectory for optional execution
        if abs(vel - 0.5) < 0.01 and abs(accel - 0.3) < 0.01:
            exec_traj = traj

        time.sleep(0.3)

    valid = [r for r in results if r is not None]
    if not valid:
        print("\nNo valid trajectories. Is the robot already at the target?")
        print("Run: python3 smooth_motion.py --home")
        rclpy.shutdown()
        spin_thread.join()
        return

    print("\nGenerating plots...")
    plot_comparison(results, JOINT_NAMES)
    plot_stats(results, JOINT_NAMES)

    print(f"\nView plots on Windows:")
    print(f"  \\\\wsl$\\Ubuntu-22.04{OUT_COMPARISON}")
    print(f"  \\\\wsl$\\Ubuntu-22.04{OUT_STATS}")

    if args.execute and exec_traj is not None:
        node.get_logger().info("Executing balanced combo (vel=0.5, accel=0.3)...")
        moveit2.execute(exec_traj)
        moveit2.wait_until_executed()
        node.get_logger().info("Execution complete.")
    elif args.execute and exec_traj is None:
        node.get_logger().warn(
            "Could not find balanced combo trajectory to execute. "
            "Add (0.5, 0.3) to COMBOS or use --vel 0.5 --accel 0.3 --execute."
        )

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
