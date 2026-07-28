#!/usr/bin/env python3
"""
sweep_speeds.py — Plan the same target motion at several velocity/acceleration
scaling factor pairs using Pilz PTP (deterministic, repeatable trapezoidal
profiles). Generates a dark-themed comparison plot of velocity, acceleration,
and jerk for the most-active joint across all combos.

Usage:
    python3 sweep_speeds.py
"""

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
# CONFIG
# ─────────────────────────────────────────────────────────────────────────────
JOINT_NAMES = ["Revolute_1", "Revolute_2", "Revolute_3", "Revolute_4", "Revolute_5"]
BASE_LINK    = "base_link"
END_EFFECTOR = "L70IE_Finger"
MOVE_GROUP   = "arm"

TARGET_POSITION  = [0.0, -0.0595, 0.45]
TARGET_QUAT_XYZW = [-0.4997, -0.5003, -0.4997, 0.5003]

PLOT_PATH = "/home/saad/kerabot_ws/speed_sweep_plot.png"

# (vel_scale, accel_scale, label, colour)
COMBOS = [
    (1.0, 0.8, "Max  (vel=1.0, accel=0.8)",    "#E63946"),
    (0.7, 0.5, "Fast (vel=0.7, accel=0.5)",     "#F4A261"),
    (0.5, 0.3, "Balanced — default",             "#2A9D8F"),
    (0.3, 0.2, "Smooth (vel=0.3, accel=0.2)",   "#457B9D"),
    (0.2, 0.1, "Ultra (vel=0.2, accel=0.1)",    "#A8DADC"),
]

BG, PANEL, GRID, TCOL = "#0f0f1a", "#1a1a2e", "#2a2a4a", "#d0d0f0"


def main():
    rclpy.init()
    node = Node("kerabot_speed_sweep")
    cb   = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name=BASE_LINK,
        end_effector_name=END_EFFECTOR,
        group_name=MOVE_GROUP,
        callback_group=cb,
    )
    # Pilz PTP — deterministic, no randomization, Ruckig-smoothed
    moveit2.planner_id            = "PTP"
    moveit2.num_planning_attempts = 10
    moveit2.allowed_planning_time = 10.0

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.0)

    print(f"\n{'Vel':>5} {'Accel':>6}  {'Duration':>9}  {'WayPts':>6}  "
          f"{'PeakJerk':>9}  {'AvgJerk':>8}  Active joint")
    print("-" * 72)

    fig = plt.figure(figsize=(16, 12), facecolor=BG)
    fig.suptitle("Kerabot Speed Sweep — Pilz PTP + Ruckig  |  Most-Active Joint",
                 fontsize=13, color=TCOL, fontweight="bold", y=0.99)
    gs = gridspec.GridSpec(3, 1, figure=fig, hspace=0.5,
                           left=0.09, right=0.98, top=0.94, bottom=0.06)
    axes = [fig.add_subplot(gs[i]) for i in range(3)]
    labels = ["Velocity (rad/s)", "Acceleration (rad/s²)", "Jerk (rad/s³)"]
    titles = ["Velocity Profile — trapezoidal shape",
              "Acceleration Profile",
              "Jerk Profile — lower = smoother"]

    for ax, label, title in zip(axes, labels, titles):
        ax.set_facecolor(PANEL)
        ax.set_title(title, color=TCOL, fontsize=10, pad=4, fontweight="bold")
        ax.set_ylabel(label, color="#8888aa", fontsize=8)
        ax.tick_params(colors="#8888aa", labelsize=7)
        for sp in ax.spines.values():
            sp.set_color(GRID)
        ax.grid(True, color=GRID, linewidth=0.5, alpha=0.8)
    axes[-1].set_xlabel("Time (s)", color="#8888aa", fontsize=8)

    for vel, accel, label, color in COMBOS:
        moveit2.max_velocity    = vel
        moveit2.max_acceleration = accel

        traj = moveit2.plan(
            position=TARGET_POSITION,
            quat_xyzw=TARGET_QUAT_XYZW,
            cartesian=False,
        )

        if traj is None or len(traj.points) < 2:
            print(f"  {vel:>4}  {accel:>5}  FAILED / already at target — run --home first")
            continue

        pts   = traj.points
        n     = len(traj.joint_names)
        times = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                          for p in pts])
        vel_arr = np.array([[p.velocities[j] if p.velocities else 0.0
                             for j in range(n)] for p in pts])
        acc_arr = np.array([[p.accelerations[j] if p.accelerations else 0.0
                             for j in range(n)] for p in pts])
        jerk_arr = np.gradient(vel_arr, times, axis=0)

        # Most-active joint for this run
        ranges   = np.ptp(vel_arr, axis=0)
        j        = int(np.argmax(ranges))
        jname    = traj.joint_names[j]
        peak_jk  = float(np.max(np.abs(jerk_arr[:, j])))
        avg_jk   = float(np.mean(np.abs(jerk_arr[:, j])))

        print(f"  {vel:>4}  {accel:>5}  {times[-1]:>9.3f}  {len(pts):>6}  "
              f"{peak_jk:>9.3f}  {avg_jk:>8.3f}  {jname}")

        plot_label = f"{label}  [{jname}]"
        axes[0].plot(times, vel_arr[:, j],  color=color, lw=2.0, label=plot_label)
        axes[1].plot(times, acc_arr[:, j],  color=color, lw=2.0, label=plot_label)
        axes[2].plot(times, jerk_arr[:, j], color=color, lw=2.0, label=plot_label)

        time.sleep(0.3)

    for ax in axes:
        ax.legend(loc="upper right", fontsize=7, labelcolor=TCOL, framealpha=0.25)

    plt.savefig(PLOT_PATH, dpi=150, bbox_inches="tight", facecolor=BG)
    print(f"\nSaved → {PLOT_PATH}")
    print(f"View   → \\\\wsl$\\Ubuntu-22.04{PLOT_PATH}")

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
