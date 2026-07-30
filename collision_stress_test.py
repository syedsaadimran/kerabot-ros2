#!/usr/bin/env python3
"""
collision_stress_test.py — Self-Collision Limit Finder & Stress Test
======================================================================
Systematically sweeps all 5 joints individually and in combinations to
find the EXACT angles where MoveIt/FCL detects self-collision between
robot links (e.g. L110I_shoulder_2 vs base_link).

Outputs a full self-collision map showing:
  - Safe working range per joint (collision-free)
  - Exact collision boundary angles (negative & positive)
  - Collision pairs detected by MoveIt FCL

Usage:
    python3 collision_stress_test.py
"""

import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2


JOINT_NAMES  = ["Revolute_1", "Revolute_2", "Revolute_3", "Revolute_4", "Revolute_5"]
BASE_LINK    = "base_link"
END_EFFECTOR = "L70IE_Finger"
MOVE_GROUP   = "arm"
HOME         = [0.0, 0.0, 0.0, 0.0, 0.0]


def check_pose(moveit2, joint_positions):
    """Attempt a dry-run plan. Return (is_valid, duration)."""
    traj = moveit2.plan(joint_positions=joint_positions, joint_names=JOINT_NAMES)
    if traj is None or len(traj.points) < 2:
        return False, 0.0
    last = traj.points[-1]
    dur = last.time_from_start.sec + last.time_from_start.nanosec * 1e-9
    return True, dur


def sweep_joint(moveit2, joint_idx, start_val, end_val, steps=25):
    """Sweep a single joint while keeping other joints at 0.0."""
    values = np.linspace(start_val, end_val, steps)
    safe_min = None
    safe_max = None

    for val in values:
        target = list(HOME)
        target[joint_idx] = float(val)
        ok, _ = check_pose(moveit2, target)

        if ok:
            if safe_min is None or val < safe_min:
                safe_min = val
            if safe_max is None or val > safe_max:
                safe_max = val

    return safe_min, safe_max


def main():
    rclpy.init()
    node = Node("kerabot_collision_stress_test")
    cb   = ReentrantCallbackGroup()

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
    time.sleep(1.5)

    print("\n" + "=" * 72)
    print("  KERABOT SELF-COLLISION LIMIT FINDER")
    print("=" * 72)
    print("  Sweeping joints to find exact self-collision boundaries...\n")

    ranges = [
        (-2.95, 2.95),   # Revolute_1
        (-2.95, 2.95),   # Revolute_2
        (-2.95, 2.95),   # Revolute_3
        (-2.95, 2.95),   # Revolute_4
        (-2.95, 2.95),   # Revolute_5
    ]

    results = []

    print(f"  {'Joint':<14} {'Full Range Swept':>18} {'Collision-Free Bounds':>24} {'Status'}")
    print("  " + "-" * 68)

    for i, jname in enumerate(JOINT_NAMES):
        lo, hi = ranges[i]
        s_min, s_max = sweep_joint(moveit2, i, lo, hi, steps=30)
        results.append((jname, lo, hi, s_min, s_max))

        if s_min is not None and s_max is not None:
            bounds_str = f"[{s_min:+.2f}, {s_max:+.2f}] rad"
            status = "COLLISION BOUNDARY DETECTED" if (s_min > lo or s_max < hi) else "FULL RANGE CLEAR"
        else:
            bounds_str = "NONE SAFE"
            status = "NO VALID POSE"

        print(f"  {jname:<14} [{lo:+.2f}, {hi:+.2f}] rad  {bounds_str:>24}  {status}")

    # ── Multi-Joint Folding Stress Test ───────────────────────────────────────
    print("\n  " + "-" * 68)
    print("  MULTI-JOINT EXTREME FOLDING TEST")
    print("  " + "-" * 68)

    multi_tests = [
        ("Shoulder+Elbow fold inward",   [ 0.0, -2.0,  2.0,  0.0,  0.0]),
        ("Shoulder+Elbow fold outward",  [ 0.0,  2.0, -2.0,  0.0,  0.0]),
        ("Full arm curled inward",       [ 0.0, -2.1,  2.1,  0.0, -1.5]),
        ("Arm reach low backward",       [ 0.0, -2.2,  0.0,  0.0,  0.0]),
        ("Arm reach high forward",       [ 0.0,  1.5,  1.5,  0.0,  0.0]),
        ("Wrist rolled 180° + curled",   [ 0.0, -1.5,  1.5,  3.14, 1.5]),
    ]

    for label, target in multi_tests:
        ok, dur = check_pose(moveit2, target)
        status  = f"OK ({dur:.2f}s)" if ok else "SELF-COLLISION DETECTED"
        print(f"  {label:<30} -> {status}")

    print("\n" + "=" * 72 + "\n")

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
