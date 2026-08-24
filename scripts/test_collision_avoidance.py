#!/usr/bin/env python3
"""
test_collision_avoidance.py — Verification of Collision Detection & Auto-Reroute Pipeline
========================================================================================
Validates:
  Test 1: Self-Collision Detection on Colliding Poses (ACM check against Wrist_Motor / J2J3_Shoulder)
  Test 2: Pre-Execution Target Abort Guard on Invalid Target Pose
  Test 3: Collision-Free Safe Execution to Gazebo via CollisionAwarePlanner
"""

import sys
import os
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2
from collision_aware_planner import CollisionAwarePlanner, JOINT_NAMES, BASE_LINK, END_EFFECTOR, MOVE_GROUP


def main():
    rclpy.init()
    node = Node(
        "test_collision_avoidance",
        parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        ],
    )
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
    time.sleep(2.0)

    planner = CollisionAwarePlanner(node, moveit2, max_reroute_attempts=5, planning_timeout=5.0)

    print("\n" + "="*75)
    print(" 🛡️  KERABOT COLLISION-AWARE PIPELINE VERIFICATION SUITE")
    print("="*75)

    # ─────────────────────────────────────────────────────────────────────────────
    # TEST 1: Direct ACM & PlanningScene Collision Query on Folded Colliding Pose
    # (Revolute_5 tilted into Wrist_Motor / J2J3_Shoulder as in user screenshot)
    # ─────────────────────────────────────────────────────────────────────────────
    print("\n[TEST 1] Querying PlanningScene for Self-Colliding Robot Poses...")
    
    # Pose A: Box folded back into forearm (as in screenshot: Revolute_5 = +2.4 rad, tilt into Wrist_Motor)
    colliding_pose_1 = [0.0, -0.8, 1.2, 0.0, 2.4, 0.0]
    is_valid_1, contacts_1 = planner.check_joint_state_validity(colliding_pose_1)
    print(f"  Pose 1 {colliding_pose_1}:")
    print(f"    - PlanningScene Valid: {is_valid_1} (Expected: False)")
    print(f"    - Conflicting Links:   {contacts_1}")
    assert not is_valid_1, "FAIL: Self-collision was NOT detected!"
    assert any("end_effector_box_link" in c for pair in contacts_1 for c in pair), "FAIL: Box link was not in collision pair!"
    print("  ✅ TEST 1 PASSED: Self-collision between end_effector_box_link and arm links is actively detected by ACM.")

    # ─────────────────────────────────────────────────────────────────────────────
    # TEST 2: Pre-Execution Target Abort Guard on Invalid Colliding Pose
    # ─────────────────────────────────────────────────────────────────────────────
    print("\n[TEST 2] Testing Pre-Execution Target Abort Mechanism...")
    res_abort = planner.plan_and_execute_with_rerouting(
        target_joint_positions=colliding_pose_1,
        preferred_pipeline="ompl",
        preferred_planner="RRTConnect",
        stage_name="Invalid Self-Collision Target Test"
    )
    print(f"  Result Success: {res_abort['success']} (Expected: False)")
    print(f"  Error Message:  {res_abort['error']}")
    assert not res_abort["success"], "FAIL: Invalid pose was executed without aborting!"
    assert "[COLLISION DETECTED]" in res_abort["error"], "FAIL: Expected [COLLISION DETECTED] prefix!"
    print("  ✅ TEST 2 PASSED: Pipeline strictly aborted execution before commanding actuators.")

    # ─────────────────────────────────────────────────────────────────────────────
    # TEST 3: Safe Collision-Free Execution to Gazebo via CollisionAwarePlanner
    # ─────────────────────────────────────────────────────────────────────────────
    print("\n[TEST 3] Testing Verified Safe Collision-Free Motion to Gazebo...")
    safe_pose = [0.2, -0.4, 0.6, 0.1, -0.3, 0.2]
    is_valid_safe, contacts_safe = planner.check_joint_state_validity(safe_pose)
    print(f"  Pose {safe_pose}: Valid = {is_valid_safe}")
    assert is_valid_safe, f"FAIL: Safe pose was flagged in collision! {contacts_safe}"

    res_safe = planner.plan_and_execute_with_rerouting(
        target_joint_positions=safe_pose,
        preferred_pipeline="ompl",
        preferred_planner="RRTConnect",
        vel_scale=0.5,
        accel_scale=0.3,
        stage_name="Safe Verified Motion"
    )
    print(f"  Result Success: {res_safe['success']} (Expected: True)")
    print(f"  Planning Time:  {res_safe['plan_time_ms']:.1f} ms")
    print(f"  Execution Time: {res_safe['exec_time_s']:.2f} s")
    assert res_safe["success"], "FAIL: Safe motion failed execution!"
    print("  ✅ TEST 3 PASSED: Verified collision-free trajectory safely planned and executed in Gazebo.")

    print("\n" + "="*75)
    print(" 🎉 ALL COLLISION-AWARE PIPELINE TESTS PASSED SUCCESSFULLY!")
    print("="*75 + "\n")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
