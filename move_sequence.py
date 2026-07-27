#!/usr/bin/env python3
"""
move_sequence.py — Run a chained sequence of poses via MoveIt 2,
using the pymoveit2 wrapper.

Edit the WAYPOINTS list below to define your own sequence:
each entry is (position, quat_xyzw, pause_seconds_after).
"""

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2


# ---------------------------------------------------------------------
# ROBOT CONFIG (same as move_to_pose.py)
# ---------------------------------------------------------------------
JOINT_NAMES = [
    "Revolute_1",
    "Revolute_2",
    "Revolute_3",
    "Revolute_4",
    "Revolute_5",
]
BASE_LINK = "base_link"
END_EFFECTOR_LINK = "L70IE_Finger"
MOVE_GROUP_NAME = "arm"
HOME_JOINT_POSITIONS = [0.0, 0.0, 0.0, 0.0, 0.0]
CARTESIAN = False

# ---------------------------------------------------------------------
# YOUR SEQUENCE — edit this list to define the chain of events
# Each entry: (position [x,y,z], quat_xyzw [x,y,z,w], pause_seconds_after)
# ---------------------------------------------------------------------
WAYPOINTS = [
    # Move to position 1, then pause 3 seconds
    ([0.0, -0.0595, 1.00], [-0.4997, -0.5003, -0.4997, 0.5003], 3.0),

    # Move to position 2, then pause 3 seconds
    ([0.0, -0.0595, 0.65], [-0.4997, -0.5003, -0.4997, 0.5003], 3.0),
    
    ([0.0, -0.0595, 0.45], [-0.4997, -0.5003, -0.4997, 0.5003], 3.0),

    # Then return home (handled automatically at the end, see below)
]


def main():
    rclpy.init()

    node = Node("kerabot_sequence_node")
    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name=BASE_LINK,
        end_effector_name=END_EFFECTOR_LINK,
        group_name=MOVE_GROUP_NAME,
        callback_group=callback_group,
    )

    # Increase planning effort well beyond the library's low defaults
    moveit2.num_planning_attempts = 50
    moveit2.allowed_planning_time = 10.0

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Give joint states a moment to arrive before the first command
    time.sleep(1.0)

    for i, (position, quat_xyzw, pause_after) in enumerate(WAYPOINTS, start=1):
        node.get_logger().info(
            f"[Step {i}] Moving to position={position}, quat_xyzw={quat_xyzw}"
        )
        moveit2.move_to_pose(
            position=position,
            quat_xyzw=quat_xyzw,
            cartesian=CARTESIAN,
        )
        moveit2.wait_until_executed()
        node.get_logger().info(f"[Step {i}] Reached target. Pausing {pause_after}s...")
        time.sleep(pause_after)

    node.get_logger().info("Sequence complete. Returning to home...")
    moveit2.move_to_configuration(HOME_JOINT_POSITIONS)
    moveit2.wait_until_executed()
    node.get_logger().info("Reached home. All done.")

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
