#!/usr/bin/env python3
"""
move_to_pose.py — Send a target pose (position + orientation) to Kerabot
via MoveIt2, using the Pilz PTP planner for smooth trapezoidal motion.

Position is in metres, in the base_link frame.
Orientation is a quaternion [x, y, z, w].

Usage examples
--------------
# Move to hardcoded TARGET_POSITION / TARGET_QUAT_XYZW below:
python3 move_to_pose.py

# Move using Roll/Pitch/Yaw (degrees) instead of a raw quaternion:
# Edit the TARGET_QUAT_XYZW line below:
#   TARGET_QUAT_XYZW = euler_to_quat(roll_deg=0, pitch_deg=90, yaw_deg=0)

# For an interactive/validated version with position checking, use:
#   python3 manual_move.py --pose <x> <y> <z> <roll> <pitch> <yaw>
"""

import math
import threading
import time

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
BASE_LINK        = "base_link"
END_EFFECTOR     = "L70IE_Finger"
MOVE_GROUP       = "arm"

# ─────────────────────────────────────────────────────────────────────────────
# TARGET POSE — edit these two lines to move the arm somewhere else
# ─────────────────────────────────────────────────────────────────────────────
TARGET_POSITION  = [0.0, -0.0595, 0.65]                   # [x, y, z] metres
TARGET_QUAT_XYZW = [-0.4997, -0.5003, -0.4997, 0.5003]   # [x, y, z, w] quaternion

CARTESIAN    = False   # True = straight Cartesian line (Pilz LIN), False = PTP
VEL_SCALE    = 0.5     # 0.0–1.0  velocity scaling
ACCEL_SCALE  = 0.3     # 0.0–1.0  acceleration scaling


def euler_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float):
    """Convert Roll/Pitch/Yaw (degrees) → [x, y, z, w] quaternion."""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)

    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def main():
    rclpy.init()

    node = Node("kerabot_pose_goal_node")
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
    moveit2.planner_id              = "PTP"
    moveit2.num_planning_attempts   = 10
    moveit2.allowed_planning_time   = 10.0
    moveit2.max_velocity            = VEL_SCALE
    moveit2.max_acceleration        = ACCEL_SCALE

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.0)

    planner = "LIN" if CARTESIAN else "PTP"
    node.get_logger().info(
        f"Moving to position={TARGET_POSITION}, quat={TARGET_QUAT_XYZW} "
        f"| Pilz {planner} | vel={VEL_SCALE} accel={ACCEL_SCALE}"
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
