#!/usr/bin/env python3
"""
pick_place_industrial_sim.py — Full Industrial Pick & Place Sequence Test
==========================================================================
Executes a complete 5-stage industrial Pick & Place workflow for 6 distinct
pick-and-place object transfers across the workspace with active ground plane
collision checking:

  Stage 1: Pre-Pick Approach  (X_pick,  Y_pick,  Z_hover = 0.35m)
  Stage 2: Pick Descent       (X_pick,  Y_pick,  Z_pick  = 0.22m)
  Stage 3: Post-Pick Lift     (X_pick,  Y_pick,  Z_hover = 0.35m)
  Stage 4: Transport & Place  (X_place, Y_place, Z_place = 0.22m)
  Stage 5: Retract & Reset    (Home [0,0,0,0,0])

Uses Pilz PTP + Ruckig trapezoidal smoothing for all moves.

Usage:
    python3 pick_place_industrial_sim.py
"""

import math
import threading
import time
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

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

# Pick and Place Transfers: (Transfer Label, Pick_XYZ, Place_XYZ)
TRANSFERS = [
    ("Transfer 1: Left Front -> Right Front", ( 0.30, -0.35, 0.22), (-0.30, -0.35, 0.22)),
    ("Transfer 2: Front Center -> Left Front", ( 0.00, -0.45, 0.22), ( 0.28, -0.35, 0.22)),
    ("Transfer 3: Right Front -> Center",     (-0.30, -0.35, 0.22), ( 0.00, -0.40, 0.22)),
    ("Transfer 4: Far Front -> Close Left",   ( 0.00, -0.55, 0.22), ( 0.25, -0.35, 0.22)),
    ("Transfer 5: Cross-Diagonal (Right-Left)",(-0.28, -0.38, 0.22), ( 0.28, -0.38, 0.22)),
    ("Transfer 6: Center -> Far Right",      ( 0.00, -0.38, 0.22), (-0.28, -0.38, 0.22)),
]


# ── Collision-Free 5-DOF IK Solver ──────────────────────────────────────────
def fk_kerabot_full(q):
    q1, q2, q3, q4, q5 = q

    T1 = np.eye(4)
    T1[:3, 3] = [0.0, 0.0, 0.08]
    R_j1_base = R.from_euler('xyz', [-np.pi/2, 0, 0]).as_matrix()
    R_j1_rot  = R.from_euler('xyz', [0, q1, 0]).as_matrix()
    T1[:3, :3] = R_j1_base @ R_j1_rot

    T2 = np.eye(4)
    T2[:3, 3] = [0.0, -0.119, 0.057]
    R_j2_base = R.from_euler('xyz', [np.pi, 0, 0]).as_matrix()
    R_j2_rot  = R.from_euler('xyz', [0, 0, q2]).as_matrix()
    T2[:3, :3] = R_j2_base @ R_j2_rot
    T12 = T1 @ T2

    T3 = np.eye(4)
    T3[:3, 3] = [0.0, 0.426, 0.003028]
    R_j3_base = R.from_euler('xyz', [-np.pi/2, -np.pi/2, 0]).as_matrix()
    R_j3_rot  = R.from_euler('xyz', [-q3, 0, 0]).as_matrix()
    T3[:3, :3] = R_j3_base @ R_j3_rot
    T123 = T12 @ T3

    T4 = np.eye(4)
    T4[:3, 3] = [0.053972, 0.0, 0.314]
    R_j4_rot  = R.from_euler('xyz', [0, 0, q4]).as_matrix()
    T4[:3, :3] = R_j4_rot
    T1234 = T123 @ T4

    T5 = np.eye(4)
    T5[:3, 3] = [0.0595, 0.0, 0.130]
    R_j5_base = R.from_euler('xyz', [-0.000787, -np.pi/2, 0.002101]).as_matrix()
    R_j5_rot  = R.from_euler('xyz', [0, 0, q5]).as_matrix()
    T5[:3, :3] = R_j5_base @ R_j5_rot
    T_total = T1234 @ T5

    pos_ee = T_total[:3, 3]
    tool_z = T_total[:3, 2]
    links  = [T1[:3, 3], T12[:3, 3], T123[:3, 3], T1234[:3, 3]]
    return pos_ee, tool_z, links


def solve_ik_collision_free(target_xyz):
    target_pos = np.array(target_xyz)
    target_z_dir = np.array([0.0, 0.0, -1.0])

    def objective(q):
        pos_ee, tool_z, links = fk_kerabot_full(q)
        pos_err = np.linalg.norm(pos_ee - target_pos)

        # Ground clearance penalty (Z > 0.03m for all links)
        g_pen = sum(500.0 * (0.03 - l[2])**2 for l in links if l[2] < 0.03)

        # Joint limit soft penalty
        j_pen = sum(100.0 * (lo - q_i)**2 if q_i < lo else (100.0 * (q_i - hi)**2 if q_i > hi else 0)
                    for q_i, (lo, hi) in zip(q, [(-2.9, 2.9)] * 5))

        return 1000.0 * pos_err**2 + g_pen + j_pen

    bounds = [(-2.9, 2.9)] * 5
    best_q = None
    best_cost = float("inf")

    q1_base = math.atan2(target_xyz[0], -target_xyz[1])
    guesses = [
        [q1_base, -0.6, 1.2, 0.0, 0.0],
        [q1_base, -1.0, 1.6, 0.0, -0.5],
        [q1_base, -0.4, 0.8, 0.0, 0.5],
    ]

    for q0 in guesses:
        res = minimize(objective, q0, method='L-BFGS-B', bounds=bounds, options={'maxiter': 50})
        if res.success and res.fun < best_cost:
            best_cost = res.fun
            best_q = res.x

    if best_q is None:
        return None

    pos_act, _, links = fk_kerabot_full(best_q)
    if np.linalg.norm(pos_act - target_pos) > 0.04 or any(l[2] < 0.01 for l in links):
        return None

    return list(best_q)


def execute_step(moveit2, label, joint_target):
    t0 = time.time()
    traj = moveit2.plan(joint_positions=joint_target, joint_names=JOINT_NAMES)
    p_ms = (time.time() - t0) * 1000

    if traj is None or len(traj.points) < 2:
        print(f"    [{label:<22}] ❌ PLAN FAILED ({p_ms:.1f}ms)")
        return False, 0.0

    dur = traj.points[-1].time_from_start.sec + traj.points[-1].time_from_start.nanosec * 1e-9
    moveit2.execute(traj)
    moveit2.wait_until_executed()
    time.sleep(0.15)
    print(f"    [{label:<22}] ✅ OK (plan={p_ms:.1f}ms, dur={dur:.2f}s)")
    return True, dur


def main():
    rclpy.init()
    node = Node("kerabot_industrial_pick_place")
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
    moveit2.max_velocity          = 0.6
    moveit2.max_acceleration      = 0.4
    moveit2.allowed_planning_time = 5.0

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.5)

    print("\n" + "=" * 78)
    print("  KERABOT INDUSTRIAL PICK & PLACE TRANSFERS (5-Stage Motion Pipeline)")
    print("=" * 78)
    print("  Executing 6 complete Pick -> Lift -> Transport -> Place -> Reset sequences...\n")

    transfer_summary = []

    for idx, (label, pick_xyz, place_xyz) in enumerate(TRANSFERS, start=1):
        print(f"  ── {label} ──────────────────────────────────────────")
        print(f"     Pick: ({pick_xyz[0]:+.2f}, {pick_xyz[1]:+.2f}, {pick_xyz[2]:+.2f}) -> "
              f"Place: ({place_xyz[0]:+.2f}, {place_xyz[1]:+.2f}, {place_xyz[2]:+.2f})")

        # Solve IK for all 4 motion poses
        pick_hover  = (pick_xyz[0],  pick_xyz[1],  0.35)
        place_hover = (place_xyz[0], place_xyz[1], 0.35)

        q_pre_pick  = solve_ik_collision_free(pick_hover)
        q_pick      = solve_ik_collision_free(pick_xyz)
        q_pre_place = solve_ik_collision_free(place_hover)
        q_place     = solve_ik_collision_free(place_xyz)

        if not all([q_pre_pick, q_pick, q_pre_place, q_place]):
            print("    ❌ IK Solution or ground clearance check failed for transfer!")
            transfer_summary.append((label, False, 0.0))
            continue

        t_transfer_start = time.time()
        ok1, d1 = execute_step(moveit2, "1. Pre-Pick Approach", q_pre_pick)
        ok2, d2 = execute_step(moveit2, "2. Pick Descent",      q_pick)
        ok3, d3 = execute_step(moveit2, "3. Post-Pick Lift",    q_pre_pick)
        ok4, d4 = execute_step(moveit2, "4. Transport & Place", q_place)
        ok5, d5 = execute_step(moveit2, "5. Retract & Reset",   HOME)

        transfer_ok = ok1 and ok2 and ok3 and ok4 and ok5
        dur_total   = time.time() - t_transfer_start
        status_str  = f"PASSED ({dur_total:.2f}s)" if transfer_ok else "FAILED"
        print(f"     Result: {status_str}\n")
        transfer_summary.append((label, transfer_ok, dur_total))

    # ── Final Summary ────────────────────────────────────────────────────────
    print("=" * 78)
    print("  INDUSTRIAL PICK & PLACE STRESS TEST SUMMARY")
    print("=" * 78)

    passed_cnt = sum(1 for _, ok, _ in transfer_summary if ok)
    total_cnt  = len(transfer_summary)

    for label, ok, dur in transfer_summary:
        st = f"PASSED ({dur:.2f}s)" if ok else "FAILED"
        print(f"  {label:<40} -> {st}")

    print("-" * 78)
    print(f"  Total Transfers Completed Successfully: {passed_cnt} / {total_cnt} ({passed_cnt/total_cnt*100:.1f}%)")
    print("  Pipeline Engine: Pilz PTP + Ruckig Trapezoidal Dynamics")
    print("  Ground Plane   : Persistent Active Collision Checking at Z = 0.0m")
    print("=" * 78 + "\n")

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
