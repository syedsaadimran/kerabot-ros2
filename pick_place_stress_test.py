#!/usr/bin/env python3
"""
pick_place_stress_test.py — Pick & Place Orientation Stress Test
===================================================================
Simulates a top-down pick & place operation at 16 scattered grid locations
across the reachable workspace at Z = 0.10m (10cm above the ground plane).

For each pick target (X, Y, Z=0.10m):
  1. Finds the optimal 5-DOF joint configuration that jointly minimizes:
       - Position error relative to target (X, Y, Z)
       - Orientation error relative to straight-down ([0, 0, -1])
  2. Measures exact position error (metres) and orientation deviation (degrees).
  3. Plans and executes the move via Pilz PTP + Ruckig with a --home reset.
  4. Categorizes the pick viability:
       - Ideal (< 5° error)
       - Acceptable (5° - 15° error)
       - Compromised (15° - 45° error)
       - Severe / Unreachable (> 45° error)

Usage:
    python3 pick_place_stress_test.py
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

# Pick height above table (10cm above Z=0.0m)
PICK_Z = 0.10

# Generate 16 pick/place targets across workspace radii (0.30m to 0.60m) and angles (-120° to +120°)
PICK_TARGETS = [
    # (label, x, y, z)
    ("Front-Center (0.35m)",   0.00, -0.35, PICK_Z),
    ("Front-Center (0.45m)",   0.00, -0.45, PICK_Z),
    ("Front-Center (0.55m)",   0.00, -0.55, PICK_Z),
    ("Front-Left 30° (0.40m)",  0.20, -0.35, PICK_Z),
    ("Front-Left 45° (0.45m)",  0.32, -0.32, PICK_Z),
    ("Front-Left 60° (0.50m)",  0.43, -0.25, PICK_Z),
    ("Front-Right 30° (0.40m)",-0.20, -0.35, PICK_Z),
    ("Front-Right 45° (0.45m)",-0.32, -0.32, PICK_Z),
    ("Front-Right 60° (0.50m)",-0.43, -0.25, PICK_Z),
    ("Side-Left 90° (0.40m)",   0.40,  0.00, PICK_Z),
    ("Side-Right 90° (0.40m)", -0.40,  0.00, PICK_Z),
    ("Far-Left 120° (0.45m)",   0.39,  0.22, PICK_Z),
    ("Far-Right 120° (0.45m)", -0.39,  0.22, PICK_Z),
    ("Close-Front (0.28m)",     0.00, -0.28, PICK_Z),
    ("Far-Front (0.62m)",      0.00, -0.62, PICK_Z),
    ("Far-Left 45° (0.58m)",   0.41, -0.41, PICK_Z),
]


# ── Forward Kinematics (FK) for Kerabot 5-DOF ────────────────────────────────
def fk_kerabot(q):
    """
    Computes end-effector position (3,) and tool Z-axis direction vector (3,)
    for joint angles q = [q1, q2, q3, q4, q5] in radians.
    Matches exact URDF offsets & joint axes.
    """
    q1, q2, q3, q4, q5 = q

    # Joint 1: Base yaw (axis [0, 1, 0] in URDF rpy="-1.570796 0 0")
    # Base link -> L110I_Shoulder
    T1 = np.eye(4)
    T1[:3, 3] = [0.0, 0.0, 0.08]
    # rpy="-1.570796 0 0"
    R_j1_base = R.from_euler('xyz', [-np.pi/2, 0, 0]).as_matrix()
    R_j1_rot  = R.from_euler('xyz', [0, q1, 0]).as_matrix()
    T1[:3, :3] = R_j1_base @ R_j1_rot

    # Joint 2: Shoulder pitch (L110I_Shoulder -> L110I_shoulder_2)
    T2 = np.eye(4)
    T2[:3, 3] = [0.0, -0.119, 0.057]
    R_j2_base = R.from_euler('xyz', [np.pi, 0, 0]).as_matrix()
    R_j2_rot  = R.from_euler('xyz', [0, 0, q2]).as_matrix()
    T2[:3, :3] = R_j2_base @ R_j2_rot

    # Joint 3: Elbow pitch (L110I_shoulder_2 -> J2J3_Shoulder)
    T3 = np.eye(4)
    T3[:3, 3] = [0.0, 0.426, 0.003028]
    R_j3_base = R.from_euler('xyz', [-np.pi/2, -np.pi/2, 0]).as_matrix()
    R_j3_rot  = R.from_euler('xyz', [-q3, 0, 0]).as_matrix()
    T3[:3, :3] = R_j3_base @ R_j3_rot

    # Joint 4: Wrist roll (J2J3_Shoulder -> Wrist_Motor)
    T4 = np.eye(4)
    T4[:3, 3] = [0.053972, 0.0, 0.314]
    R_j4_rot  = R.from_euler('xyz', [0, 0, q4]).as_matrix()
    T4[:3, :3] = R_j4_rot

    # Joint 5: Wrist pitch (Wrist_Motor -> L70IE_Finger)
    T5 = np.eye(4)
    T5[:3, 3] = [0.0595, 0.0, 0.130]
    R_j5_base = R.from_euler('xyz', [-0.000787, -np.pi/2, 0.002101]).as_matrix()
    R_j5_rot  = R.from_euler('xyz', [0, 0, q5]).as_matrix()
    T5[:3, :3] = R_j5_base @ R_j5_rot

    # Full transform: T = T1 * T2 * T3 * T4 * T5
    T_total = T1 @ T2 @ T3 @ T4 @ T5
    pos = T_total[:3, 3]
    tool_z = T_total[:3, 2]  # Z-axis of tool in base_link frame
    return pos, tool_z, T_total


def optimize_top_down_ik(target_xyz):
    """
    Finds q = [q1..q5] that minimizes position error to target_xyz
    AND orientation error relative to straight-down [0, 0, -1].
    """
    target_pos = np.array(target_xyz)
    target_z_dir = np.array([0.0, 0.0, -1.0])

    def objective(q):
        pos, tool_z, _ = fk_kerabot(q)
        pos_err = np.linalg.norm(pos - target_pos)
        # Orientation cost: 1 - dot(tool_z, [0,0,-1])
        dot_val = np.clip(np.dot(tool_z, target_z_dir), -1.0, 1.0)
        ori_err = 1.0 - dot_val
        # Heavily weight position so position accuracy is primary, orientation secondary
        return 100.0 * pos_err**2 + 5.0 * ori_err

    # Joint limits [-2.9, 2.9]
    bounds = [(-2.9, 2.9)] * 5
    best_q = None
    best_cost = float("inf")

    # Multi-start initial guesses
    for q1_guess in [math.atan2(target_xyz[0], -target_xyz[1]), 0.0]:
        for q2_guess in [-0.5, -1.0, -1.5, 0.5]:
            for q3_guess in [0.5, 1.0, 1.5, -0.5]:
                q0 = [q1_guess, q2_guess, q3_guess, 0.0, 0.0]
                res = minimize(objective, q0, method='L-BFGS-B', bounds=bounds)
                if res.success and res.fun < best_cost:
                    best_cost = res.fun
                    best_q = res.x

    if best_q is None:
        return None, 999.0, 999.0

    pos_act, tool_z_act, _ = fk_kerabot(best_q)
    pos_err = float(np.linalg.norm(pos_act - target_pos))
    dot_val = float(np.clip(np.dot(tool_z_act, target_z_dir), -1.0, 1.0))
    ori_err_deg = float(math.acos(dot_val) * (180.0 / math.pi))

    return best_q, pos_err, ori_err_deg


def get_current_joints(moveit2):
    if moveit2.joint_state is None:
        return None
    mapping = dict(zip(moveit2.joint_state.name, moveit2.joint_state.position))
    return [mapping.get(jn, float("nan")) for jn in JOINT_NAMES]


def main():
    rclpy.init()
    node = Node("kerabot_pick_place_test")
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
    moveit2.allowed_planning_time = 10.0

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.5)

    print("\n" + "=" * 78)
    print("  KERABOT TOP-DOWN PICK & PLACE STRESS TEST (Z = 0.10m)")
    print("=" * 78)
    print("  Testing 16 pick locations for position accuracy & top-down orientation...\n")

    results = []

    print(f"  {'Location':<24} {'Pos Target (x,y,z)':<22} {'Pos Err':>8} {'Ori Error':>11} {'Viability'}")
    print("  " + "-" * 74)

    for label, x, y, z in PICK_TARGETS:
        # Home reset before each pick
        t_home = moveit2.plan(joint_positions=HOME, joint_names=JOINT_NAMES)
        if t_home:
            moveit2.execute(t_home)
            moveit2.wait_until_executed()
            time.sleep(0.2)

        t0 = time.time()
        best_q, pos_err, ori_err_deg = optimize_top_down_ik((x, y, z))
        plan_ms = (time.time() - t0) * 1000

        if best_q is None or pos_err > 0.05:
            viability = "UNREACHABLE"
            print(f"  {label:<24} ({x:+.2f},{y:+.2f},{z:+.2f})       {pos_err:>7.3f}m  {'N/A':>11}  {viability}")
            results.append((label, x, y, z, pos_err, ori_err_deg, False, viability))
            continue

        # Plan & execute via Pilz PTP
        traj = moveit2.plan(joint_positions=list(best_q), joint_names=JOINT_NAMES)

        if traj is None or len(traj.points) < 2:
            viability = "PLAN FAILED"
            print(f"  {label:<24} ({x:+.2f},{y:+.2f},{z:+.2f})       {pos_err:>7.3f}m  {ori_err_deg:>10.1f}°  {viability}")
            results.append((label, x, y, z, pos_err, ori_err_deg, False, viability))
            continue

        moveit2.execute(traj)
        moveit2.wait_until_executed()
        time.sleep(0.3)

        actual = get_current_joints(moveit2)
        exec_err = max(abs(a - t) for a, t in zip(actual, best_q)) if actual else 999.0
        exec_ok  = exec_err < 0.05

        if ori_err_deg < 5.0:
            viability = "IDEAL (<5°)"
        elif ori_err_deg < 15.0:
            viability = "ACCEPTABLE (5-15°)"
        elif ori_err_deg < 45.0:
            viability = "COMPROMISED (15-45°)"
        else:
            viability = "SEVERE (>45°)"

        status_str = viability if exec_ok else "EXEC FAILED"
        print(f"  {label:<24} ({x:+.2f},{y:+.2f},{z:+.2f})       {pos_err*1000:>6.1f}mm  {ori_err_deg:>10.1f}°  {status_str}")
        results.append((label, x, y, z, pos_err, ori_err_deg, exec_ok, viability))

    # Return home
    t_home = moveit2.plan(joint_positions=HOME, joint_names=JOINT_NAMES)
    if t_home:
        moveit2.execute(t_home)
        moveit2.wait_until_executed()

    # ── Categorized Viability Summary ─────────────────────────────────────────
    print("\n" + "=" * 78)
    print("  PICK & PLACE VIABILITY SUMMARY REPORT")
    print("=" * 78)

    ideal_cnt       = sum(1 for r in results if r[6] and r[7] == "IDEAL (<5°)")
    acceptable_cnt  = sum(1 for r in results if r[6] and r[7] == "ACCEPTABLE (5-15°)")
    compromised_cnt = sum(1 for r in results if r[6] and r[7] == "COMPROMISED (15-45°)")
    severe_cnt      = sum(1 for r in results if not r[6] or "SEVERE" in r[7] or "UNREACHABLE" in r[7] or "PLAN" in r[7])
    total           = len(results)

    print(f"  Total Pick Targets Tested : {total}")
    print(f"  1. Ideal Top-Down Pick (< 5° error)         : {ideal_cnt:>2} / {total} ({ideal_cnt/total*100:.1f}%)")
    print(f"  2. Acceptable Top-Down Pick (5° - 15° error) : {acceptable_cnt:>2} / {total} ({acceptable_cnt/total*100:.1f}%)")
    print(f"  3. Compromised Pick (15° - 45° error)        : {compromised_cnt:>2} / {total} ({compromised_cnt/total*100:.1f}%)")
    print(f"  4. Severe / Unreachable (> 45° or fail)      : {severe_cnt:>2} / {total} ({severe_cnt/total*100:.1f}%)")

    valid_results = [r for r in results if r[6]]
    if valid_results:
        avg_pos_mm = (sum(r[4] for r in valid_results) / len(valid_results)) * 1000
        avg_ori_deg = sum(r[5] for r in valid_results) / len(valid_results)
        print(f"\n  Average Position Error   : {avg_pos_mm:.2f} mm")
        print(f"  Average Orientation Error: {avg_ori_deg:.1f}°")

    print("=" * 78 + "\n")

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
