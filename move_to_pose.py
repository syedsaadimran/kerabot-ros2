#!/usr/bin/env python3
"""
move_to_pose.py — Send a target pose (position + orientation) to kerabot
via MoveIt 2, using the pymoveit2 wrapper.

Position is in metres, in the base_link frame.
Orientation is a quaternion [x, y, z, w]. Use [0,0,0,1] for "no rotation".
"""

import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2


# ---------------------------------------------------------------------
# ROBOT CONFIG (already filled in for kerabot)
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

# ---------------------------------------------------------------------
# TARGET POSE — edit these two lines to move the arm somewhere else
# ---------------------------------------------------------------------
TARGET_POSITION = [0.0, -0.0595, 0.65]      # [x, y, z] in metres, base_link frame
TARGET_QUAT_XYZW = [-0.4997, -0.5003, -0.4997, 0.5003]   # [x, y, z, w] quaternion, identity = no rotation
CARTESIAN = False                          # True = straight-line path, False = joint-space path


def euler_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float):
    """Convert Roll/Pitch/Yaw in degrees to a [x, y, z, w] quaternion."""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return [qx, qy, qz, qw]


def main():
    rclpy.init()

    node = Node("kerabot_pose_goal_node")
    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name=BASE_LINK,
        end_effector_name=END_EFFECTOR_LINK,
        group_name=MOVE_GROUP_NAME,
        callback_group=callback_group,
    )
    moveit2.num_planning_attempts = 50
    moveit2.allowed_planning_time = 10.0
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.get_logger().info(
        f"Moving to position={TARGET_POSITION}, quat_xyzw={TARGET_QUAT_XYZW}, "
        f"cartesian={CARTESIAN}"
    )

    moveit2.move_to_pose(
        position=TARGET_POSITION,
        quat_xyzw=TARGET_QUAT_XYZW,
        cartesian=CARTESIAN,
    )
    moveit2.wait_until_executed()

    node.get_logger().info("Motion complete.")

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()

# ---------------------------------------------------------------------
# Example: using Roll/Pitch/Yaw instead of a raw quaternion
# ---------------------------------------------------------------------
# Replace the TARGET_QUAT_XYZW line above with, e.g.:
#   TARGET_QUAT_XYZW = euler_to_quat(roll_deg=0, pitch_deg=90, yaw_deg=0)
