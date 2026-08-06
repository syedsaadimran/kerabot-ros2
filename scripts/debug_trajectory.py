#!/usr/bin/env python3
"""
debug_trajectory.py — Plan the same joint-space move manual_move.py uses,
and print the RAW time_from_start for every waypoint, to see exactly
what Pilz is generating.
"""

import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from pymoveit2 import MoveIt2

JOINT_NAMES = ["Revolute_1", "Revolute_2", "Revolute_3", "Revolute_4", "Revolute_5"]
BASE_LINK = "base_link"
END_EFFECTOR = "L70IE_Finger"
MOVE_GROUP = "arm"

TARGET_JOINTS = [0.3, -0.5, 0.4, 0.0, 0.2]


def main():
    rclpy.init()
    node = Node("debug_trajectory")
    cb = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name=BASE_LINK,
        end_effector_name=END_EFFECTOR,
        group_name=MOVE_GROUP,
        callback_group=cb,
    )
    moveit2.planner_id = "PTP"
    moveit2.num_planning_attempts = 10
    moveit2.allowed_planning_time = 10.0
    moveit2.max_velocity = 0.5
    moveit2.max_acceleration = 0.3

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.5)

    traj = moveit2.plan(joint_positions=TARGET_JOINTS, joint_names=JOINT_NAMES)

    if traj is None:
        print("Planning failed entirely.")
    else:
        print(f"joint_names in trajectory: {traj.joint_names}")
        print(f"Number of points: {len(traj.points)}\n")
        print(f"{'idx':<5} {'time_from_start (s)':<22} {'positions'}")
        for i, p in enumerate(traj.points):
            t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
            print(f"{i:<5} {t:<22.6f} {list(p.positions)}")

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
