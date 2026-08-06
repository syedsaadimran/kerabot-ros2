#!/usr/bin/env python3
"""
compare_profiles.py — Visual Comparison of Velocity/Acceleration Scaling Combos
=================================================================================
Plans the same motion using Pilz PTP at several (vel_scale, accel_scale) pairs
and generates two output figures:

  1. profile_comparison.png  — velocity trapezoid shape for every joint × combo
  2. stats_comparison.png    — bar charts: duration, peak jerk, peak force per combo

Usage:
    python3 compare_profiles.py               # uses built-in COMBOS list
    python3 compare_profiles.py --cartesian   # use Pilz LIN instead of PTP

The robot does NOT move — all planning is done offline (dry-run only).
"""

import argparse
import threading
import time

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2


# ──────────────────────────────────────────────────────────────────────────────
# ROBOT CONFIG
# ──────────────────────────────────────────────────────────────────────────────
JOINT_NAMES      = ["Revolute_1", "Revolute_2", "Revolute_3", "Revolute_4", "Revolute_5"]
BASE_LINK        = "base_link"
END_EFFECTOR     = "L70IE_Finger"
MOVE_GROUP       = "arm"
TARGET_POSITION  = [0.0, -0.0595, 0.65]
TARGET_QUAT_XYZW = [-0.4997, -0.5003, -0.4997, 0.5003]

# ──────────────────────────────────────────────────────────────────────────────
# COMBINATIONS TO COMPARE
# (vel_scale, accel_scale, label, hex_colour)
# ──────────────────────────────────────────────────────────────────────────────
COMBOS = [
    (1.0, 0.8, "Max Speed\n(vel=1.0, accel=0.8)",   "#E63946"),
    (0.7, 0.5, "Fast\n(vel=0.7, accel=0.5)",         "#F4A261"),
    (0.5, 0.3, "Balanced — default\n(vel=0.5, accel=0.3)", "#2A9D8F"),
    (0.3, 0.2, "Smooth\n(vel=0.3, accel=0.2)",       "#457B9D"),
    (0.2, 0.1, "Ultra Smooth\n(vel=0.2, accel=0.1)", "#A8DADC"),
]

OUT_PROFILES = "/home/saad/kerabot_ws/profile_comparison.png"
OUT_STATS    = "/home/saad/kerabot_ws/stats_comparison.png"

BG      = "#0f0f1a"
PANEL   = "#1a1a2e"
PANEL2  = "#16213e"
GRID    = "#2a2a4a"
TCOL    = "#d0d0f0"
TCOL2   = "#8888aa"


# ──────────────────────────────────────────────────────────────────────────────
# HELPERS
# ──────────────────────────────────────────────────────────────────────────────
def style(ax, title, ylabel, xlabel="Time (s)"):
    ax.set_facecolor(PANEL)
    ax.set_title(title, color=TCOL, fontsize=9, pad=5, fontweight="bold")
    ax.set_ylabel(ylabel, color=TCOL2, fontsize=8)
    ax.set_xlabel(xlabel, color=TCOL2, fontsize=7)
    ax.tick_params(colors=TCOL2, labelsize=7)
    for sp in ax.spines.values():
        sp.set_color(GRID)
    ax.grid(True, color=GRID, linewidth=0.5, alpha=0.8)


def shade_cruise(ax, times, velocities, color, tol=0.15):
    """Shade the cruise (flat-top) region of the trapezoidal profile."""
    v_peak = np.max(np.abs(velocities))
    if v_peak < 1e-6:
        return
    v_norm = np.abs(velocities) / v_peak
    n = len(v_norm)
    cruise = (v_norm > (1.0 - tol)) & (np.arange(n) > 0) & (np.arange(n) < n - 1)
    if not np.any(cruise):
        return
    idx = np.where(cruise)[0]
    ax.axvspan(times[idx[0]], times[idx[-1]],
               alpha=0.12, color=color, linewidth=0)


def compute_stats(times, velocities, accelerations):
    """Return per-joint stats dict."""
    jerk = np.gradient(velocities, times, axis=0)   # (T, 5)
    return {
        "duration":   times[-1],
        "peak_jerk":  [float(np.max(np.abs(jerk[:, j]))) for j in range(5)],
        "rms_jerk":   [float(np.sqrt(np.mean(jerk[:, j]**2))) for j in range(5)],
        "peak_vel":   [float(np.max(np.abs(velocities[:, j]))) for j in range(5)],
        "peak_accel": [float(np.max(np.abs(accelerations[:, j]))) for j in range(5)],
    }


# ──────────────────────────────────────────────────────────────────────────────
# FIGURE 1 — Velocity trapezoidal profiles, one subplot per joint
# ──────────────────────────────────────────────────────────────────────────────
def plot_velocity_comparison(results):
    """
    5 rows (one per joint) × 1 column.
    Each subplot has one coloured line per (vel, accel) combo.
    Cruise phase is shaded for the first combo only (to avoid clutter).
    """
    fig = plt.figure(figsize=(16, 18), facecolor=BG)
    fig.suptitle(
        "Kerabot — Trapezoidal Velocity Profile Comparison  (Pilz PTP)\n"
        "Shaded region = cruise phase (flat top) for each profile",
        fontsize=13, color=TCOL, fontweight="bold", y=0.99)

    gs = gridspec.GridSpec(5, 1, figure=fig, hspace=0.55,
                           left=0.09, right=0.98, top=0.94, bottom=0.04)

    for j, jname in enumerate(JOINT_NAMES):
        ax = fig.add_subplot(gs[j])
        style(ax, f"Joint: {jname}", "Velocity (rad/s)")

        for (vel, accel, label, color), res in zip(COMBOS, results):
            if res is None:
                continue
            times = res["times"]
            vels  = res["velocities"][:, j]

            shade_cruise(ax, times, vels, color)
            ax.plot(times, vels, color=color, linewidth=2.2,
                    label=label.replace("\n", "  "))
            ax.axhline(0, color=GRID, linewidth=0.6)

        ax.legend(loc="upper right", fontsize=7, labelcolor=TCOL,
                  framealpha=0.25, frameon=True)

    plt.savefig(OUT_PROFILES, dpi=150, bbox_inches="tight", facecolor=BG)
    print(f"[1/2] Saved velocity comparison → {OUT_PROFILES}")


# ──────────────────────────────────────────────────────────────────────────────
# FIGURE 2 — Stats comparison: duration, peak jerk, peak force (bar charts)
# ──────────────────────────────────────────────────────────────────────────────
def plot_stats_comparison(results):
    """
    3-panel bar chart:
      Left  — total motion duration per combo
      Middle — peak jerk per joint per combo (grouped bars)
      Right  — peak velocity per joint per combo (grouped bars)
    Plus a recommendation panel at the bottom.
    """
    valid = [(combo, res) for combo, res in zip(COMBOS, results) if res is not None]
    labels      = [c[2].replace("\n", " ") for c, _ in valid]
    colors      = [c[3] for c, _ in valid]
    durations   = [r["duration"]  for _, r in valid]
    n_combos    = len(valid)
    bar_w       = 0.15
    joint_x     = np.arange(5)

    fig = plt.figure(figsize=(20, 14), facecolor=BG)
    fig.suptitle("Kerabot — Scaling Factor Stats Comparison  (Pilz PTP + Ruckig Smoothing)",
                 fontsize=14, color=TCOL, fontweight="bold", y=0.99)

    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.55, wspace=0.38,
                           left=0.07, right=0.98, top=0.93, bottom=0.22)

    # ── Duration bar chart ────────────────────────────────────────────────────
    ax_dur = fig.add_subplot(gs[0, 0])
    style(ax_dur, "Total Motion Duration", "Duration (s)", "Combo")
    bars = ax_dur.bar(range(n_combos), durations, color=colors, width=0.6,
                      edgecolor=GRID, linewidth=0.8)
    ax_dur.set_xticks(range(n_combos))
    ax_dur.set_xticklabels([f"C{i+1}" for i in range(n_combos)],
                           color=TCOL2, fontsize=8)
    for bar, dur in zip(bars, durations):
        ax_dur.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.04,
                    f"{dur:.2f}s", ha="center", va="bottom",
                    color=TCOL, fontsize=8, fontweight="bold")
    ax_dur.set_ylim(0, max(durations) * 1.2)

    # ── Peak Jerk per joint ───────────────────────────────────────────────────
    ax_jk = fig.add_subplot(gs[0, 1])
    style(ax_jk, "Peak Jerk per Joint (rad/s³)\nLower = Smoother", "Jerk (rad/s³)", "Joint")
    for i, ((combo, res), color) in enumerate(zip(valid, colors)):
        offsets = (i - n_combos / 2 + 0.5) * bar_w
        ax_jk.bar(joint_x + offsets, res["peak_jerk"],
                  width=bar_w, color=color, edgecolor=GRID,
                  linewidth=0.6, alpha=0.9, label=f"C{i+1}")
    ax_jk.set_xticks(joint_x)
    ax_jk.set_xticklabels(JOINT_NAMES, color=TCOL2, fontsize=7, rotation=15)
    ax_jk.legend(fontsize=7, labelcolor=TCOL, framealpha=0.2)

    # ── Peak Velocity per joint ───────────────────────────────────────────────
    ax_vel = fig.add_subplot(gs[0, 2])
    style(ax_vel, "Peak Velocity per Joint (rad/s)", "Velocity (rad/s)", "Joint")
    for i, ((combo, res), color) in enumerate(zip(valid, colors)):
        offsets = (i - n_combos / 2 + 0.5) * bar_w
        ax_vel.bar(joint_x + offsets, res["peak_vel"],
                   width=bar_w, color=color, edgecolor=GRID,
                   linewidth=0.6, alpha=0.9, label=f"C{i+1}")
    ax_vel.set_xticks(joint_x)
    ax_vel.set_xticklabels(JOINT_NAMES, color=TCOL2, fontsize=7, rotation=15)
    ax_vel.legend(fontsize=7, labelcolor=TCOL, framealpha=0.2)

    # ── RMS Jerk per joint ────────────────────────────────────────────────────
    ax_rms = fig.add_subplot(gs[1, 0])
    style(ax_rms, "RMS Jerk per Joint (rad/s³)\nOverall Smoothness Indicator", "RMS Jerk", "Joint")
    for i, ((combo, res), color) in enumerate(zip(valid, colors)):
        offsets = (i - n_combos / 2 + 0.5) * bar_w
        ax_rms.bar(joint_x + offsets, res["rms_jerk"],
                   width=bar_w, color=color, edgecolor=GRID,
                   linewidth=0.6, alpha=0.9, label=f"C{i+1}")
    ax_rms.set_xticks(joint_x)
    ax_rms.set_xticklabels(JOINT_NAMES, color=TCOL2, fontsize=7, rotation=15)
    ax_rms.legend(fontsize=7, labelcolor=TCOL, framealpha=0.2)

    # ── Speed vs Smoothness scatter ───────────────────────────────────────────
    ax_sc = fig.add_subplot(gs[1, 1])
    style(ax_sc, "Speed vs Smoothness Trade-off\n(ideal = top-left corner)",
          "Avg RMS Jerk (rad/s³)", "Duration (s)")
    for i, ((combo, res), color) in enumerate(zip(valid, colors)):
        avg_rms = np.mean(res["rms_jerk"])
        ax_sc.scatter(res["duration"], avg_rms, s=160, color=color,
                      edgecolors="white", linewidth=1.0, zorder=5)
        ax_sc.annotate(f"C{i+1}", (res["duration"], avg_rms),
                       textcoords="offset points", xytext=(6, 4),
                       color=color, fontsize=8, fontweight="bold")
    # Annotate ideal zone
    ax_sc.annotate("Ideal zone\n(fast + smooth)", xy=(0.08, 0.08),
                   xycoords="axes fraction", color="#aaaaaa", fontsize=7,
                   fontstyle="italic")

    # ── Legend / recommendation panel ─────────────────────────────────────────
    ax_leg = fig.add_subplot(gs[1, 2])
    ax_leg.set_facecolor(PANEL2)
    ax_leg.axis("off")
    ax_leg.set_title("Combo Legend", color=TCOL, fontsize=9,
                     pad=6, fontweight="bold")

    # Score each combo: lower duration + lower avg jerk = better rank
    scores = []
    for combo, res in valid:
        if res:
            norm_dur  = res["duration"] / max(durations)
            norm_jerk = np.mean(res["rms_jerk"]) / max(np.mean(r["rms_jerk"])
                                                        for _, r in valid)
            scores.append(0.4 * norm_dur + 0.6 * norm_jerk)
        else:
            scores.append(999)

    best_idx = int(np.argmin(scores))

    for i, ((combo, res), color) in enumerate(zip(valid, colors)):
        tag  = " <-- RECOMMENDED" if i == best_idx else ""
        text = f"C{i+1}  {combo[2].replace(chr(10), '  ')}{tag}"
        ax_leg.text(0.05, 0.88 - i * 0.17, text,
                    transform=ax_leg.transAxes,
                    color=color if i != best_idx else "#ffffff",
                    fontsize=8, fontweight="bold" if i == best_idx else "normal",
                    bbox=dict(boxstyle="round,pad=0.3",
                              facecolor=color + "33",
                              edgecolor=color,
                              linewidth=1.2) if i == best_idx else None)

    # ── Summary table at the bottom ───────────────────────────────────────────
    col_labels = ["Combo", "vel", "accel", "Duration (s)",
                  "Avg RMS Jerk", "Avg Peak Jerk", "Score"]
    rows = []
    for i, ((combo, res), score) in enumerate(zip(valid, scores)):
        rows.append([
            f"C{i+1} — {combo[2].split(chr(10))[0]}",
            f"{combo[0]:.1f}",
            f"{combo[1]:.1f}",
            f"{res['duration']:.3f}",
            f"{np.mean(res['rms_jerk']):.4f}",
            f"{np.mean(res['peak_jerk']):.4f}",
            f"{score:.3f} {'<<' if i == best_idx else ''}",
        ])

    table_ax = fig.add_axes([0.04, 0.01, 0.92, 0.16])
    table_ax.set_facecolor(BG)
    table_ax.axis("off")
    tbl = table_ax.table(cellText=rows, colLabels=col_labels,
                         cellLoc="center", loc="center")
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8.5)
    tbl.scale(1, 1.7)
    for (r, c), cell in tbl.get_celld().items():
        is_best_row = (r > 0) and (r - 1 == best_idx)
        cell.set_facecolor("#2a3a2a" if is_best_row else
                           (PANEL if r % 2 == 0 else PANEL2))
        cell.set_text_props(color="#00ff88" if is_best_row else TCOL)
        cell.set_edgecolor(GRID)

    plt.savefig(OUT_STATS, dpi=150, bbox_inches="tight", facecolor=BG)
    print(f"[2/2] Saved stats comparison   → {OUT_STATS}")


# ──────────────────────────────────────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="Kerabot velocity/acceleration profile comparison")
    p.add_argument("--cartesian", action="store_true",
                   help="Use Pilz LIN (Cartesian) instead of PTP")
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()

    node = Node("kerabot_profile_comparison")
    cb   = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name=BASE_LINK,
        end_effector_name=END_EFFECTOR,
        group_name=MOVE_GROUP,
        callback_group=cb,
    )
    moveit2.pipeline_id         = "pilz_industrial_motion_planner"
    moveit2.planner_id          = "LIN" if args.cartesian else "PTP"
    moveit2.num_planning_attempts = 10
    moveit2.allowed_planning_time = 10.0

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.0)

    planner = "LIN" if args.cartesian else "PTP"
    print(f"\nPlanner: Pilz {planner} + Ruckig Smoothing")
    print(f"Target : {TARGET_POSITION}")
    print(f"Testing {len(COMBOS)} combinations...\n")
    print(f"{'#':<4} {'vel':<6} {'accel':<7} {'Waypts':<8} {'Duration':<10} {'AvgRMSJerk':<12} Status")
    print("-" * 62)

    results = []
    for i, (vel, accel, label, color) in enumerate(COMBOS):
        moveit2.max_velocity    = vel
        moveit2.max_acceleration = accel

        traj = moveit2.plan(
            position=TARGET_POSITION,
            quat_xyzw=TARGET_QUAT_XYZW,
            cartesian=args.cartesian,
        )

        if traj is None or len(traj.points) < 2:
            print(f"  C{i+1}  {vel:<6} {accel:<7} {'--':<8} {'--':<10} {'--':<12} FAILED / already at target")
            results.append(None)
            continue

        pts   = traj.points
        T     = len(pts)
        n     = len(traj.joint_names)
        times = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in pts])
        vels  = np.array([[p.velocities[j]    if p.velocities    else 0.0 for j in range(n)] for p in pts])
        accs  = np.array([[p.accelerations[j] if p.accelerations else 0.0 for j in range(n)] for p in pts])

        stats = compute_stats(times, vels, accs)
        avg_rms = np.mean(stats["rms_jerk"])
        print(f"  C{i+1}  {vel:<6} {accel:<7} {T:<8} {stats['duration']:<10.3f} {avg_rms:<12.4f} OK")

        results.append({
            "times":       times,
            "velocities":  vels,
            "accelerations": accs,
            **stats,
        })

        # Small pause between plans to avoid overwhelming MoveIt
        time.sleep(0.3)

    print("\nGenerating plots...")
    valid_count = sum(1 for r in results if r is not None)
    if valid_count == 0:
        print("ERROR: All plans failed. Is the robot already at the target?\n"
              "Hint: Run  python3 trapezoidal_dynamics.py --home --execute  first to move it away.")
        rclpy.shutdown()
        spin_thread.join()
        return

    plot_velocity_comparison(results)
    plot_stats_comparison(results)

    print(f"\nDone! Open these files to view results:")
    print(f"  {OUT_PROFILES}")
    print(f"  {OUT_STATS}")
    print(f"\nTip: Copy to Windows Explorer via:")
    print(f"  \\\\wsl$\\Ubuntu-22.04\\home\\saad\\kerabot_ws\\profile_comparison.png")
    print(f"  \\\\wsl$\\Ubuntu-22.04\\home\\saad\\kerabot_ws\\stats_comparison.png")

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
