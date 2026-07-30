#!/usr/bin/env python3
"""
plot_trajectory.py — Plan a motion via MoveIt2 (dry-run, no execution),
then plot ALL 5 joints': position, velocity, acceleration, and jerk.

Uses Pilz PTP for a deterministic, trapezoidal velocity profile.

Usage:
    python3 plot_trajectory.py
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
CARTESIAN        = False

VEL_SCALE   = 0.5
ACCEL_SCALE = 0.3

PLOT_PATH = "/home/saad/kerabot_ws/trajectory_plot.png"

# Dark theme
BG, PANEL, GRID, TCOL = "#0f0f1a", "#1a1a2e", "#2a2a4a", "#d0d0f0"
COLORS = ["#E63946", "#F4A261", "#2A9D8F", "#457B9D", "#A8DADC"]


def main():
    rclpy.init()

    node = Node("kerabot_trajectory_plotter")
    cb   = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name=BASE_LINK,
        end_effector_name=END_EFFECTOR,
        group_name=MOVE_GROUP,
        callback_group=cb,
    )
    moveit2.pipeline_id            = "pilz_industrial_motion_planner"
    moveit2.planner_id            = "PTP"
    moveit2.num_planning_attempts = 10
    moveit2.allowed_planning_time = 10.0
    moveit2.max_velocity          = VEL_SCALE
    moveit2.max_acceleration      = ACCEL_SCALE

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.0)

    node.get_logger().info(
        f"Planning Pilz PTP to {TARGET_POSITION} | vel={VEL_SCALE} accel={ACCEL_SCALE}"
    )
    trajectory = moveit2.plan(
        position=TARGET_POSITION,
        quat_xyzw=TARGET_QUAT_XYZW,
        cartesian=CARTESIAN,
    )

    if trajectory is None:
        node.get_logger().error("Planning failed — nothing to plot.")
        rclpy.shutdown()
        spin_thread.join()
        return

    pts      = trajectory.points
    jnames   = trajectory.joint_names
    n_joints = len(jnames)
    T        = len(pts)

    if T < 2:
        node.get_logger().warn("Only 1 waypoint — robot already at target. Run --home first.")
        rclpy.shutdown()
        spin_thread.join()
        return

    times = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in pts])
    pos   = np.array([[p.positions[j]     for j in range(n_joints)] for p in pts])
    vel   = np.array([[p.velocities[j]    if p.velocities    else 0.0 for j in range(n_joints)] for p in pts])
    acc   = np.array([[p.accelerations[j] if p.accelerations else 0.0 for j in range(n_joints)] for p in pts])
    jerk  = np.gradient(vel, times, axis=0)

    node.get_logger().info(
        f"Got {T} waypoints over {times[-1]:.3f}s. Generating plot..."
    )

    # ── 4-panel figure ────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 14), facecolor=BG)
    fig.suptitle(
        f"Kerabot Trajectory — Pilz PTP + Ruckig  |  {T} pts, {times[-1]:.2f}s  "
        f"|  vel={VEL_SCALE} accel={ACCEL_SCALE}",
        fontsize=13, color=TCOL, fontweight="bold", y=0.99
    )

    gs = gridspec.GridSpec(4, 1, figure=fig, hspace=0.5,
                           left=0.1, right=0.97, top=0.94, bottom=0.05)

    panels = [
        (pos,  "Position (rad)",    "Joint Positions"),
        (vel,  "Velocity (rad/s)",  "Joint Velocities — trapezoidal profile"),
        (acc,  "Accel (rad/s²)",    "Joint Accelerations"),
        (jerk, "Jerk (rad/s³)",     "Joint Jerk — lower = smoother"),
    ]

    for row, (data, ylabel, title) in enumerate(panels):
        ax = fig.add_subplot(gs[row])
        ax.set_facecolor(PANEL)
        ax.set_title(title, color=TCOL, fontsize=10, pad=4, fontweight="bold")
        ax.set_ylabel(ylabel, color="#8888aa", fontsize=8)
        ax.tick_params(colors="#8888aa", labelsize=7)
        for sp in ax.spines.values():
            sp.set_color(GRID)
        ax.grid(True, color=GRID, linewidth=0.5, alpha=0.8)

        for j, (jname, color) in enumerate(zip(jnames, COLORS)):
            ax.plot(times, data[:, j], color=color, linewidth=1.8, label=jname)

        if row == len(panels) - 1:
            ax.set_xlabel("Time (s)", color="#8888aa", fontsize=8)
        ax.legend(loc="upper right", fontsize=7, labelcolor=TCOL,
                  framealpha=0.25, ncol=3)

    plt.savefig(PLOT_PATH, dpi=150, bbox_inches="tight", facecolor=BG)
    node.get_logger().info(f"Saved plot to {PLOT_PATH}")
    print(f"\nView at: \\\\wsl$\\Ubuntu-22.04{PLOT_PATH}")

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
