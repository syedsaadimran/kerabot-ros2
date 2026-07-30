#!/usr/bin/env python3
"""
explore_reachable_poses.py — Generate random VALID joint configurations
(guaranteed reachable, since we're not solving IK at all) and use forward
kinematics to see what (position, orientation) pairs they actually produce.

This sidesteps IK/goal-sampling entirely, which is more reliable for a
5-DOF arm with a narrow orientation-reachability envelope.
"""

import math
import random
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

# Joint limits (radians) — adjust if your real limits differ
JOINT_LIMITS = [(-3.14, 3.14)] * 5

N_SAMPLES = 15


def quat_to_euler_deg(x, y, z, w):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def main():
    rclpy.init()
    node = Node("explore_reachable_poses")
    cb = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name=BASE_LINK,
        end_effector_name=END_EFFECTOR,
        group_name=MOVE_GROUP,
        callback_group=cb,
    )

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.5)

    print(f"\n{'Joint config (rad)':<45} {'Position (x,y,z)':<30} {'RPY (deg)'}")
    print("-" * 120)

    random.seed(42)  # reproducible results

    for i in range(N_SAMPLES):
        joints = [round(random.uniform(lo, hi), 3) for lo, hi in JOINT_LIMITS]

        fk_result = moveit2.compute_fk(
            joint_state=joints,
            fk_link_names=[END_EFFECTOR],
        )

        if fk_result is None:
            print(f"{str(joints):<45} FK FAILED")
            continue

        pose = fk_result.pose if hasattr(fk_result, "pose") else fk_result[0].pose
        p = pose.position
        q = pose.orientation
        roll, pitch, yaw = quat_to_euler_deg(q.x, q.y, q.z, q.w)

        pos_str = f"({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
        rpy_str = f"roll={roll:.1f} pitch={pitch:.1f} yaw={yaw:.1f}"
        print(f"{str(joints):<45} {pos_str:<30} {rpy_str}")

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
