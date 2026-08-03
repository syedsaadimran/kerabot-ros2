#!/usr/bin/env python3
"""
orientation_sweep.py — Test increasing orientation deviations from the
confirmed-working home quaternion, at a confirmed-reachable position,
all within a single persistent ROS session (avoids DDS crashes from
rapid init/shutdown cycles).
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

# Confirmed-reachable position (worked in earlier Pilz sweep tests)
TEST_POSITION = [0.0, -0.0595, 0.45]

# Home quaternion + increasing pitch deviations layered on top of it
QUATS_TO_TEST = [
    (-0.4997, -0.5003, -0.4997, 0.5003, "pitch_delta=0 (home)"),
    (-0.4542, -0.4548, -0.5414, 0.5420, "pitch_delta=10"),
    (-0.4053, -0.4058, -0.5789, 0.5796, "pitch_delta=20"),
    (-0.3533, -0.3538, -0.6120, 0.6127, "pitch_delta=30"),
    (-0.2704, -0.2708, -0.6529, 0.6537, "pitch_delta=45"),
    (-0.1829, -0.1831, -0.6826, 0.6834, "pitch_delta=60"),
]


def main():
    rclpy.init()
    node = Node("orientation_sweep")
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

    print(f"\n{'Label':<25} {'Result':<10} {'Waypoints':<10}")
    print("-" * 50)

    for x, y, z, w, label in QUATS_TO_TEST:
        traj = moveit2.plan(
            position=TEST_POSITION,
            quat_xyzw=[x, y, z, w],
            cartesian=False,
        )
        if traj is not None and len(traj.points) >= 1:
            result = "SUCCESS"
            n_pts = len(traj.points)
        else:
            result = "FAILED"
            n_pts = 0
        print(f"{label:<25} {result:<10} {n_pts:<10}")
        time.sleep(1.0)

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
