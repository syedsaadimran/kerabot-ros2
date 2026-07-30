#!/usr/bin/env python3
"""
verify_ground_collision.py — Verify Ground Plane Collision Detection
======================================================================
Tests whether MoveIt rejects motion plans that drive any link or end-effector
below the ground plane (Z < 0.0m).

Test cases:
  1. Valid above-ground pose (Z = 0.40m): SHOULD PASS
  2. Below-ground pose (Z = -0.15m): SHOULD BE REJECTED
  3. Severe downward joint fold (Revolute_2 = -2.2 rad into ground): SHOULD BE REJECTED

Usage:
    python3 verify_ground_collision.py
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2


JOINT_NAMES  = ["Revolute_1", "Revolute_2", "Revolute_3", "Revolute_4", "Revolute_5"]
BASE_LINK    = "base_link"
END_EFFECTOR = "L70IE_Finger"
MOVE_GROUP   = "arm"


def check_plan(moveit2, label, target_joints=None, target_pose=None):
    t0 = time.time()
    if target_joints:
        traj = moveit2.plan(joint_positions=target_joints, joint_names=JOINT_NAMES)
    else:
        x, y, z, q = target_pose
        traj = moveit2.plan(position=[x, y, z], quat_xyzw=q, cartesian=False)

    p_ms = (time.time() - t0) * 1000
    ok = traj is not None and len(traj.points) >= 2
    return ok, p_ms


def main():
    rclpy.init()
    node = Node("verify_ground_collision_node")
    cb = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name=BASE_LINK,
        end_effector_name=END_EFFECTOR,
        group_name=MOVE_GROUP,
        callback_group=cb,
    )
    moveit2.pipeline_id           = "pilz_industrial_motion_planner"
    moveit2.planner_id            = "PTP"
    moveit2.max_velocity          = 0.5
    moveit2.max_acceleration      = 0.3
    moveit2.allowed_planning_time = 5.0

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.0)

    print("\n" + "=" * 72)
    print("  VERIFYING GROUND PLANE COLLISION DETECTION")
    print("=" * 72)

    # Test 1: Valid pose above ground (Z = +0.40m)
    # Pose: x=0.0, y=-0.2, z=0.4, quat pointing forward
    t1_ok, t1_ms = check_plan(moveit2, "Above ground", target_joints=[0.0, -0.5, 0.5, 0.0, 0.0])
    print(f"  Test 1: Above-ground target (Z = +0.40m)  -> {'PASSED (Plan OK)' if t1_ok else 'FAILED'} ({t1_ms:.1f}ms)")

    # Test 2: Downward fold driving link into Z = -0.15m
    # Revolute_2 = -1.8, Revolute_3 = 0.0 drives link3 and wrist down through Z = 0.0
    t2_ok, t2_ms = check_plan(moveit2, "Below ground fold", target_joints=[0.0, -1.8, 0.0, 0.0, 0.0])
    rejected_2 = not t2_ok
    print(f"  Test 2: Below-ground fold (Z < 0.0m)       -> {'REJECTED BY MOVEIT (Correct)' if rejected_2 else 'FAILED (Allowed illegal move!)'} ({t2_ms:.1f}ms)")

    # Test 3: Downward extreme fold (Revolute_2 = -2.2, Revolute_3 = -0.5)
    t3_ok, t3_ms = check_plan(moveit2, "Extreme downward fold", target_joints=[0.0, -2.2, -0.5, 0.0, 0.0])
    rejected_3 = not t3_ok
    print(f"  Test 3: Extreme downward fold (Z < -0.2m)  -> {'REJECTED BY MOVEIT (Correct)' if rejected_3 else 'FAILED (Allowed illegal move!)'} ({t3_ms:.1f}ms)")

    print("-" * 72)
    if t1_ok and rejected_2 and rejected_3:
        print("  ✅ GROUND PLANE COLLISION DETECTION VERIFIED: MoveIt correctly rejects below-ground moves.")
    else:
        print("  ❌ GROUND PLANE VERIFICATION FAILED: Did you run python3 add_ground_plane.py first?")
    print("=" * 72 + "\n")

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
