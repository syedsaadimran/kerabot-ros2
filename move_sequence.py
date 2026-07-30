#!/usr/bin/env python3
"""
move_sequence.py — Run a chained sequence of poses via MoveIt2,
using Pilz PTP for smooth, repeatable trapezoidal motion.

Edit WAYPOINTS below. Each entry: (position, quat_xyzw, pause_seconds_after).
The robot automatically returns to HOME_JOINT_POSITIONS at the end.

Usage:
    python3 move_sequence.py
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2


# ─────────────────────────────────────────────────────────────────────────────
# ROBOT CONFIG
# ─────────────────────────────────────────────────────────────────────────────
JOINT_NAMES = [
    "Revolute_1",
    "Revolute_2",
    "Revolute_3",
    "Revolute_4",
    "Revolute_5",
]
BASE_LINK           = "base_link"
END_EFFECTOR        = "L70IE_Finger"
MOVE_GROUP          = "arm"
HOME_JOINT_POSITIONS = [0.0, 0.0, 0.0, 0.0, 0.0]

VEL_SCALE   = 0.5    # velocity scaling  0.0–1.0
ACCEL_SCALE = 0.3    # acceleration scaling 0.0–1.0
CARTESIAN   = False  # True = straight-line Pilz LIN between each waypoint

# ─────────────────────────────────────────────────────────────────────────────
# WAYPOINT SEQUENCE — edit to define your motion
# Format: (position [x, y, z], quat_xyzw [x, y, z, w], pause_after_seconds)
# ─────────────────────────────────────────────────────────────────────────────
WAYPOINTS = [
    ([0.0, -0.0595, 1.00], [-0.4997, -0.5003, -0.4997, 0.5003], 2.0),
    ([0.0, -0.0595, 0.65], [-0.4997, -0.5003, -0.4997, 0.5003], 2.0),
    ([0.0, -0.0595, 0.45], [-0.4997, -0.5003, -0.4997, 0.5003], 2.0),
]


def main():
    rclpy.init()

    node = Node("kerabot_sequence_node")
    cb   = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name=BASE_LINK,
        end_effector_name=END_EFFECTOR,
        group_name=MOVE_GROUP,
        callback_group=cb,
    )

    # ── Pilz PTP: deterministic trapezoidal + Ruckig smoothed corners ─────────
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

    time.sleep(1.0)   # wait for joint states to arrive

    node.get_logger().info(
        f"Starting sequence: {len(WAYPOINTS)} waypoints | "
        f"Pilz PTP | vel={VEL_SCALE} accel={ACCEL_SCALE}"
    )

    for i, (position, quat_xyzw, pause_after) in enumerate(WAYPOINTS, start=1):
        node.get_logger().info(
            f"[Step {i}/{len(WAYPOINTS)}] Moving to position={position}"
        )
        moveit2.move_to_pose(
            position=position,
            quat_xyzw=quat_xyzw,
            cartesian=CARTESIAN,
        )
        moveit2.wait_until_executed()
        node.get_logger().info(f"[Step {i}] Reached. Pausing {pause_after}s...")
        time.sleep(pause_after)

    node.get_logger().info("Sequence complete. Returning to home...")
    moveit2.move_to_configuration(HOME_JOINT_POSITIONS)
    moveit2.wait_until_executed()
    node.get_logger().info("Home reached. All done.")

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
