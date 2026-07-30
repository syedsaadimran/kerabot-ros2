#!/usr/bin/env python3
"""
stress_test.py — Comprehensive stress test suite for Kerabot 5-DOF arm.
Plans and executes motion across 3 categories using Pilz PTP + Ruckig:

  Category 1: Joint-space sweep — targets across the workspace
  Category 2: Rapid-fire repeats — back-to-back execution without home reset
  Category 3: Edge-of-limit targets — near valid joint limits (±2.9 rad)

Usage:
    python3 stress_test.py
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
HOME         = [0.0, 0.0, 0.0, 0.0, 0.0]

# Category 1: Joint-space sweep targets (radians)
SWEEP_TARGETS = [
    [ 0.3, -0.3,  0.3, -0.3,  0.3],
    [-0.3,  0.3, -0.3,  0.3, -0.3],
    [ 0.6, -0.6,  0.6, -0.6,  0.6],
    [-0.6,  0.6, -0.6,  0.6, -0.6],
    [ 0.9, -0.9,  0.9, -0.9,  0.9],
    [ 0.0, -0.5,  0.5,  0.0, -0.5],
    [ 0.5,  0.0, -0.5,  0.5,  0.0],
    [ 1.2, -1.2,  0.6, -0.3,  0.8],
]

# Category 2: Rapid-fire repeated motion
RAPID_FIRE_TARGET  = [0.2, -0.2, 0.2, -0.2, 0.2]
RAPID_FIRE_REPEATS = 8

# Category 3: Edge-of-limit targets (URDF limit is ±2.967 rad / ~170°)
EDGE_TARGETS = [
    [ 2.8,  0.0,  0.0,  0.0,  0.0],
    [-2.8,  0.0,  0.0,  0.0,  0.0],
    [ 0.0, -2.5,  0.0,  0.0,  0.0],
    [ 0.0,  0.0,  2.5,  0.0,  0.0],
    [ 0.0,  0.0,  0.0,  2.8,  0.0],
    [ 0.0,  0.0,  0.0,  0.0,  2.8],
    [-1.5, -1.0,  1.0,  1.0, -1.0],
]


class StressResult:
    def __init__(self, label, target):
        self.label       = label
        self.target      = target
        self.plan_ok     = False
        self.exec_ok     = False
        self.duration    = 0.0
        self.plan_time_s = 0.0
        self.max_error   = None
        self.note        = ""


def get_current_joints(moveit2):
    """Safely extract current joint positions mapped by joint name."""
    if moveit2.joint_state is None:
        return None
    js = moveit2.joint_state
    mapping = dict(zip(js.name, js.position))
    return [mapping.get(jn, float("nan")) for jn in JOINT_NAMES]


def run_move(moveit2, node, label, target_joints, results):
    r  = StressResult(label, target_joints)
    t0 = time.time()

    traj = moveit2.plan(joint_positions=target_joints, joint_names=JOINT_NAMES)
    r.plan_time_s = time.time() - t0

    if traj is None or len(traj.points) < 2:
        r.note = "PLAN FAILED"
        results.append(r)
        print(f"  [{label:<20}] PLAN FAILED ({r.plan_time_s*1000:.1f}ms)")
        return r

    r.plan_ok = True
    last = traj.points[-1]
    r.duration = last.time_from_start.sec + last.time_from_start.nanosec * 1e-9

    moveit2.execute(traj)
    moveit2.wait_until_executed()

    time.sleep(0.3)  # settle
    actual = get_current_joints(moveit2)
    if actual is not None:
        errors = [abs(a - t) for a, t in zip(actual, target_joints)]
        r.max_error = max(errors)
        r.exec_ok   = r.max_error < 0.05
    else:
        r.note = "NO JOINT STATE"

    status = "OK" if r.exec_ok else "EXEC/VERIFY FAILED"
    err_str = f"{r.max_error:.4f}rad" if r.max_error is not None else "?"
    print(
        f"  [{label:<20}] plan={r.plan_time_s*1000:>5.1f}ms  "
        f"duration={r.duration:>5.2f}s  max_err={err_str:>8}  {status}"
    )
    results.append(r)
    return r


def go_home(moveit2, node):
    traj = moveit2.plan(joint_positions=HOME, joint_names=JOINT_NAMES)
    if traj is not None:
        moveit2.execute(traj)
        moveit2.wait_until_executed()
    time.sleep(0.3)


def main():
    rclpy.init()
    node = Node("kerabot_stress_test")
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
    moveit2.num_planning_attempts = 10
    moveit2.allowed_planning_time = 10.0
    moveit2.max_velocity          = 0.5
    moveit2.max_acceleration      = 0.3

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.5)

    results = []

    print("\n=== CATEGORY 1: Joint-space sweep ===")
    for i, target in enumerate(SWEEP_TARGETS):
        go_home(moveit2, node)
        run_move(moveit2, node, f"sweep_{i}", target, results)

    print("\n=== CATEGORY 2: Rapid-fire repeats (no home reset) ===")
    go_home(moveit2, node)
    for i in range(RAPID_FIRE_REPEATS):
        target = RAPID_FIRE_TARGET if i % 2 == 0 else HOME
        run_move(moveit2, node, f"rapid_{i}", target, results)

    print("\n=== CATEGORY 3: Edge-of-limit targets (±2.8 to ±2.9 rad) ===")
    for i, target in enumerate(EDGE_TARGETS):
        go_home(moveit2, node)
        run_move(moveit2, node, f"edge_{i}", target, results)

    go_home(moveit2, node)

    # ── Summary Report ────────────────────────────────────────────────────────
    print("\n" + "=" * 70)
    print("KERABOT STRESS TEST SUMMARY")
    print("=" * 70)
    total     = len(results)
    plan_fail = sum(1 for r in results if not r.plan_ok)
    exec_fail = sum(1 for r in results if r.plan_ok and not r.exec_ok)
    passed    = sum(1 for r in results if r.exec_ok)

    print(f"Total moves attempted  : {total}")
    print(f"Passed                 : {passed} / {total} ({passed/total*100:.1f}%)")
    print(f"Planning failures      : {plan_fail}")
    print(f"Execution/verify fails : {exec_fail}")

    if results:
        avg_plan_ms  = (sum(r.plan_time_s for r in results) / total) * 1000
        avg_duration = sum(r.duration for r in results if r.plan_ok) / max(1, total - plan_fail)
        print(f"Avg planning time      : {avg_plan_ms:.1f} ms")
        print(f"Avg motion duration    : {avg_duration:.2f} s")

    if plan_fail or exec_fail:
        print("\nFAILED MOVES:")
        for r in results:
            if not r.exec_ok:
                print(f"  {r.label}: target={r.target}  note={r.note or 'exec/verify mismatch'}")
    else:
        print("\n[ALL TESTS PASSED PERFECTLY - 100% RELIABILITY]")

    print("=" * 70 + "\n")

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
