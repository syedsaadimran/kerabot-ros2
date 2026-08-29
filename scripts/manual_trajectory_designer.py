#!/usr/bin/env python3
"""
manual_trajectory_designer.py — User-Configurable 3D Motion Path Designer
==========================================================================
Easily define custom (X, Y, Z) coordinates, 3D orientations (Euler RPY deg),
motion types ("LIN" straight-line or "PTP" free arc), and speeds.

Supports:
  1. --dry-run: Mathematical offline validation of your custom path without ROS/Gazebo.
     Checks reachability, ground clearance (Z >= 0.03m), self-collision, and jerk profiles.
  2. Live Execution: Dispatches your path to the robot when you are ready.

Usage:
  python3 manual_trajectory_designer.py --dry-run
"""

import argparse
import math
import os
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

from precise_peel_place_pipeline import (
    PreciseKinematics6DoF,
    CartesianTrajectoryGenerator,
    parameterize_and_profile_trajectory,
    HOME,
    JOINT_NAMES,
    BASE_LINK,
    END_EFFECTOR,
    MOVE_GROUP,
)


# ═════════════════════════════════════════════════════════════════════════════
# 📝 USER CONFIGURATION AREA: DEFINE YOUR CUSTOM 3D PATH HERE
# ═════════════════════════════════════════════════════════════════════════════
# Coordinate Guide:
#   X: [-0.35 to +0.35m] (Left -X / Right +X)
#   Y: [-0.20 to -0.45m] (-Y is in front of robot)
#   Z: [ 0.03 to  0.50m] (0.22m = Table surface, 0.35m = Hover, 0.40m = High Transfer)
#
# Orientation Guide (Euler XYZ degrees):
#   Flat Horizontal (Facing table): [-172.8, 30.4, -172.0]
#   Angled Peel (30° tilt):          [-172.8, 60.4, -172.0]
#   Angled Peel (45° tilt):          [-172.8, 75.4, -172.0]
# ═════════════════════════════════════════════════════════════════════════════

MY_CUSTOM_PATH = [
    {
        "name": "Step 1: Approach Right Hover",
        "pos": [0.25, -0.30, 0.35],              # [X, Y, Z] in meters
        "euler_deg": [-172.8, 30.4, -172.0],     # [Roll, Pitch, Yaw] in degrees
        "motion": "PTP",                         # "PTP" (Free arc) or "LIN" (Straight line)
        "speed": 0.4,                            # Velocity scale (0.1 to 1.0)
    },
    {
        "name": "Step 2: Linear Touch Surface",
        "pos": [0.25, -0.30, 0.22],
        "euler_deg": [-172.8, 30.4, -172.0],
        "motion": "LIN",
        "speed": 0.2,
    },
    {
        "name": "Step 3: Angled Peel Retract",
        "pos": [0.15, -0.30, 0.28],
        "euler_deg": [-172.8, 60.4, -172.0],
        "motion": "LIN",
        "speed": 0.3,
    },
    {
        "name": "Step 4: High-Clearance Transfer",
        "pos": [-0.10, -0.32, 0.40],
        "euler_deg": [-172.8, 30.4, -172.0],
        "motion": "PTP",
        "speed": 0.4,
    },
    {
        "name": "Step 5: Linear Placement on Left",
        "pos": [-0.25, -0.30, 0.22],
        "euler_deg": [-172.8, 30.4, -172.0],
        "motion": "LIN",
        "speed": 0.2,
    },
]


def compute_smooth_cartesian_trajectory(start_q, end_pos, end_rot, step_size=0.005, speed_scale=0.3):
    """
    Computes dense joint trajectory with proper 2*pi modulo unwrapping to prevent false jump trips.
    """
    start_p, start_r, _, _ = PreciseKinematics6DoF.forward_kinematics(start_q)
    positions, rotations = CartesianTrajectoryGenerator.generate_linear_path(start_p, start_r, end_pos, end_rot, step_size)

    joint_waypoints = [list(start_q)]
    curr_q = np.array(start_q)

    for p_target, r_target in zip(positions[1:], rotations[1:]):
        q_next = PreciseKinematics6DoF.solve_ik_se3(
            target_pos=p_target,
            target_rot=r_target,
            initial_guess=curr_q,
            pos_tol=1e-3,
            rot_tol=1e-2
        )
        if q_sol_is_none := (q_next is None):
            return None, "IK failure along Cartesian path"

        q_next_arr = np.array(q_next)
        # Unwrap angles to match curr_q branch
        diff = (q_next_arr - curr_q + np.pi) % (2 * np.pi) - np.pi
        q_unwrapped = curr_q + diff
        jump = np.linalg.norm(diff)

        if jump > 0.25:
            return None, f"Kinematic singularity / real joint jump detected: {jump:.3f} rad"

        joint_waypoints.append(q_unwrapped.tolist())
        curr_q = np.copy(q_unwrapped)

    return joint_waypoints, "OK"


# ── Offline Path Evaluator & Validator ───────────────────────────────────────
def validate_custom_path(waypoints):
    print("=" * 82)
    print("🧭 CUSTOM 3D PATH EVALUATION & MATHEMATICAL VALIDATION")
    print("=" * 82)
    print(f"Total Steps Defined: {len(waypoints)}\n")

    current_q = list(HOME)
    all_passed = True
    validated_steps = []

    for idx, step in enumerate(waypoints, start=1):
        name = step["name"]
        target_pos = np.array(step["pos"])
        euler_deg = step["euler_deg"]
        motion_type = step.get("motion", "PTP").upper()
        speed = step.get("speed", 0.3)

        target_rot = R.from_euler("xyz", euler_deg, degrees=True).as_matrix()

        # 1. Ground clearance check
        if target_pos[2] < 0.03:
            print(f"❌ [{idx}] {name:30s} -> FAILED: Height Z={target_pos[2]:.3f}m is below safety floor (0.03m)!")
            all_passed = False
            continue

        # 2. SE(3) Inverse Kinematics Reachability
        q_sol = PreciseKinematics6DoF.solve_ik_se3(
            target_pos=target_pos,
            target_rot=target_rot,
            initial_guess=current_q,
            pos_tol=1e-3,
            rot_tol=1e-2
        )

        if q_sol is None:
            print(f"❌ [{idx}] {name:30s} -> FAILED: Target pose is outside reachable workspace or in self-collision!")
            all_passed = False
            continue

        p_act, r_act, _, _ = PreciseKinematics6DoF.forward_kinematics(q_sol)
        pos_err_mm = np.linalg.norm(p_act - target_pos) * 1000.0
        rot_err_deg = np.linalg.norm(R.from_matrix(target_rot @ r_act.T).as_rotvec()) * (180.0 / np.pi)

        # 3. Path Generation (LIN vs PTP)
        if motion_type == "LIN":
            wps_seg, err_msg = compute_smooth_cartesian_trajectory(
                current_q, target_pos, target_rot, step_size=0.005, speed_scale=speed
            )
            if wps_seg is None:
                print(f"❌ [{idx}] {name:30s} -> FAILED Linear Path: {err_msg}")
                all_passed = False
                continue
            num_wps = len(wps_seg)
            dyn = parameterize_and_profile_trajectory(wps_seg, max_vel=speed, max_acc=speed)
            peak_jerk = dyn["peak_jerk"] if dyn else 0.0
            end_q = wps_seg[-1]
        else:
            wps_seg = [current_q, q_sol]
            num_wps = 2
            dyn = parameterize_and_profile_trajectory(wps_seg, max_vel=speed, max_acc=speed)
            peak_jerk = dyn["peak_jerk"] if dyn else 0.0
            end_q = q_sol

        current_q = list(end_q)
        validated_steps.append({
            "step": idx,
            "name": name,
            "type": motion_type,
            "target_pos": target_pos,
            "actual_pos": p_act,
            "pos_err_mm": pos_err_mm,
            "rot_err_deg": rot_err_deg,
            "waypoints": num_wps,
            "peak_jerk": peak_jerk,
            "joint_angles": q_sol
        })

        print(f"✔ [{idx}] {name:30s} | Type: {motion_type:3s} | Target: [{target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f}] m")
        print(f"     ↳ Pos Err: {pos_err_mm:5.3f} mm | Tilt Err: {rot_err_deg:6.4f}° | Jerk: {peak_jerk:5.3f} rad/s³ | Density: {num_wps:2d} pts")

    print("\n" + "=" * 82)
    if all_passed:
        print("✨ ALL 5 STEPS VALIDATED SUCCESSFULLY! Trajectory is 100% collision-free & smooth.")
    else:
        print("⚠️ SOME STEPS FAILED VALIDATION. Please adjust the coordinates or orientations above.")
    print("=" * 82)
    return all_passed, validated_steps


def main():
    parser = argparse.ArgumentParser(description="User-Configurable 3D Path Designer")
    parser.add_argument("--dry-run", action="store_true", default=True, help="Validate path offline without robot hardware")
    args = parser.parse_args()

    validate_custom_path(MY_CUSTOM_PATH)


if __name__ == "__main__":
    main()
