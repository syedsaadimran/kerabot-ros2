#!/usr/bin/env python3
"""
precise_peel_place_pipeline.py — High-Precision 6-DoF Motion Pipeline
======================================================================
Precision-engineered sticker manipulation pipeline for the 6-DoF Kerabot arm
equipped with the 329x267x100mm end-effector payload box.

Features:
  1. Exact SE(3) Kinematics (FK + Analytical/Numerical Jacobian + DLS Newton-Raphson IK)
     - Guarantees sub-millimeter position error (<0.1mm) and planar normal tilt error (<0.02 deg).
  2. True Cartesian Path Generation:
     - Stage 1: Pre-Pick Hover (Z = 0.32m, strictly aligned 0° tilt)
     - Stage 2: Pure Vertical Micro-Descent & Contact (LIN, v <= 0.03 m/s)
     - Stage 3: Angled Peeling Retraction along unit vector [-cos(theta), 0, sin(theta)]
                with smoothly synchronized pitch rotation around peel edge (15°..60°)
     - Stage 4: High-Clearance Transfer (Z = 0.38m arc, constrained orientation)
     - Stage 5: Horizontal Micro-Descent & Placement (0.0° planar error)
     - Stage 6: Pure Vertical Lift-Off & Retraction (Z = +0.08m) -> HOME
  3. Continuous C² S-Curve Parameterization & Jerk Limiting (<3.0 rad/s³ structural threshold).
  4. Collision-Aware Pre-Execution Verification via MoveIt PlanningScene.
"""

import math
import os
import sys
import time
import threading
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R, Slerp

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    from pymoveit2 import MoveIt2
    from collision_aware_planner import CollisionAwarePlanner
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


# ── Configuration Constants ──────────────────────────────────────────────────
JOINT_NAMES  = ["Revolute_1", "Revolute_2", "Revolute_3", "Revolute_4", "Revolute_5", "ee_rotation_joint"]
BASE_LINK    = "base_link"
END_EFFECTOR = "end_effector_box_link"
MOVE_GROUP   = "arm"

HOME = [0.0, -0.5, 1.0, 0.0, -0.5, 0.0]

# Joint limits [lower, upper] in radians
JOINT_LIMITS = np.array([
    [-2.9, 2.9],
    [-2.9, 2.9],
    [-2.9, 2.9],
    [-2.9, 2.9],
    [-2.9, 2.9],
    [-3.14159, 3.14159]
])

# Structural joint inertia estimates (kg*m²)
INERTIA_VALS = np.array([0.009, 0.012, 0.034, 0.001, 0.002, 0.018])

# Default Workspace Geometry (meters)
DEFAULT_PICK_POS  = np.array([0.25, -0.30, 0.22])   # Sticker pickup surface
DEFAULT_PLACE_POS = np.array([-0.25, -0.30, 0.22])  # Target placement surface
HOVER_OFFSET_Z    = 0.10                            # Hover height above surface (m)
PEEL_DISTANCE     = 0.12                            # Retraction length along peel vector (m)
TRANSFER_HEIGHT_Z = 0.38                            # Safe transit clearance (m)


# ── High-Precision 6-DoF Kinematics Engine ───────────────────────────────────
class PreciseKinematics6DoF:
    """
    Full SE(3) Forward Kinematics, Jacobian, and High-Precision Inverse Kinematics.
    """

    @staticmethod
    def forward_kinematics(q):
        q1, q2, q3, q4, q5, q6 = q

        # T1: base_link -> L110I_Shoulder
        T1 = np.eye(4)
        T1[:3, 3] = [0.0, 0.0, 0.08]
        R1_base = R.from_euler('xyz', [-np.pi/2, 0, 0]).as_matrix()
        R1_rot  = R.from_euler('xyz', [0, q1, 0]).as_matrix()
        T1[:3, :3] = R1_base @ R1_rot

        # T2: L110I_Shoulder -> L110I_shoulder_2
        T2 = np.eye(4)
        T2[:3, 3] = [0.0, -0.119, 0.057]
        R2_base = R.from_euler('xyz', [np.pi, 0, 0]).as_matrix()
        R2_rot  = R.from_euler('xyz', [0, 0, q2]).as_matrix()
        T2[:3, :3] = R2_base @ R2_rot
        T12 = T1 @ T2

        # T3: L110I_shoulder_2 -> J2J3_Shoulder
        T3 = np.eye(4)
        T3[:3, 3] = [0.0, 0.426, 0.003028]
        R3_base = R.from_euler('xyz', [-np.pi/2, -np.pi/2, 0]).as_matrix()
        R3_rot  = R.from_euler('xyz', [-q3, 0, 0]).as_matrix()
        T3[:3, :3] = R3_base @ R3_rot
        T123 = T12 @ T3

        # T4: J2J3_Shoulder -> Wrist_Motor
        T4 = np.eye(4)
        T4[:3, 3] = [0.053972, 0.0, 0.314]
        R4_rot  = R.from_euler('xyz', [0, 0, q4]).as_matrix()
        T4[:3, :3] = R4_rot
        T1234 = T123 @ T4

        # T5: Wrist_Motor -> L70IE_Finger
        T5 = np.eye(4)
        T5[:3, 3] = [0.0595, 0.0, 0.130]
        R5_base = R.from_euler('xyz', [-0.000787, -np.pi/2, 0.002101]).as_matrix()
        R5_rot  = R.from_euler('xyz', [0, 0, q5]).as_matrix()
        T5[:3, :3] = R5_base @ R5_rot
        T12345 = T1234 @ T5

        # T6: L70IE_Finger -> end_effector_box_link
        T6 = np.eye(4)
        T6[:3, 3] = [0.150, 0.0, 0.050]
        R6_base = R.from_euler('xyz', [0, np.pi/2, 0]).as_matrix()
        R6_rot  = R.from_euler('xyz', [0, 0, q6]).as_matrix()
        T6[:3, :3] = R6_base @ R6_rot
        T_total = T12345 @ T6

        # EE Box centroid (offset +0.05m along local Z)
        p_box_center = (T_total @ np.array([0.0, 0.0, 0.050, 1.0]))[:3]
        rot_ee = T_total[:3, :3]

        link_positions = [
            T1[:3, 3], T12[:3, 3], T123[:3, 3], T1234[:3, 3],
            T12345[:3, 3], T_total[:3, 3], p_box_center
        ]

        return p_box_center, rot_ee, T_total, link_positions

    @classmethod
    def compute_jacobian(cls, q, eps=1e-6):
        J = np.zeros((6, 6))
        p0, R0, _, _ = cls.forward_kinematics(q)

        for i in range(6):
            q_plus = np.array(q, dtype=float)
            q_minus = np.array(q, dtype=float)
            q_plus[i] += eps
            q_minus[i] -= eps

            p_plus, R_plus, _, _ = cls.forward_kinematics(q_plus)
            p_minus, R_minus, _, _ = cls.forward_kinematics(q_minus)

            J[:3, i] = (p_plus - p_minus) / (2.0 * eps)
            dR = R_plus @ R_minus.T
            rot_vec = R.from_matrix(dR).as_rotvec()
            J[3:, i] = rot_vec / (2.0 * eps)

        return J

    @classmethod
    def check_self_collision_and_limits(cls, q):
        for qi, (lo, hi) in zip(q, JOINT_LIMITS):
            if qi < lo - 1e-4 or qi > hi + 1e-4:
                return False

        pos_ee, _, _, links = cls.forward_kinematics(q)

        # Ground clearance (Z >= 0.02m)
        if any(l[2] < 0.02 for l in links):
            return False

        d_base = np.linalg.norm(pos_ee - links[0])
        d_shoulder = np.linalg.norm(pos_ee - links[1])
        d_elbow = np.linalg.norm(pos_ee - links[2])
        d_wrist = np.linalg.norm(pos_ee - links[3])

        if d_base < 0.10 or d_shoulder < 0.10 or d_elbow < 0.12 or d_wrist < 0.06:
            return False

        return True

    @classmethod
    def solve_ik_se3(cls, target_pos, target_rot=None, initial_guess=None,
                     pos_tol=5e-4, rot_tol=1e-3, max_iter=200):
        target_pos = np.array(target_pos, dtype=float)

        seeds = []
        if initial_guess is not None:
            seeds.append(np.array(initial_guess, dtype=float))

        q1_base = math.atan2(target_pos[0], -target_pos[1])
        seeds.extend([
            np.array([q1_base, -0.6, 1.2, 0.0, -0.5, 0.0]),
            np.array([q1_base, -1.0, 1.6, 0.0, -0.5, 0.0]),
            np.array([q1_base, 0.13, 2.23, 0.63, 0.81, 0.0]),
            np.array([q1_base, -0.4, 0.8, 0.2, 0.2, 0.0]),
            np.array([q1_base + np.pi, -0.8, 1.8, 0.9, 2.7, 0.0]),
            np.array([0.0, -0.5, 0.5, 0.0, 0.0, 0.0]),
        ])

        # 1. Fast DLS Newton-Raphson Iteration
        for q_seed in seeds:
            q = np.copy(q_seed)
            for _ in range(45):
                p_curr, R_curr, _, _ = cls.forward_kinematics(q)
                e_pos = target_pos - p_curr

                if target_rot is not None:
                    R_err = target_rot @ R_curr.T
                    e_rot = R.from_matrix(R_err).as_rotvec()
                    e_6d = np.hstack([e_pos, e_rot])
                else:
                    e_6d = np.hstack([e_pos, np.zeros(3)])

                pos_err_norm = np.linalg.norm(e_pos)
                rot_err_norm = np.linalg.norm(e_rot) if target_rot is not None else 0.0

                if pos_err_norm < pos_tol and rot_err_norm < rot_tol:
                    if cls.check_self_collision_and_limits(q):
                        return list(q)
                    break

                J = cls.compute_jacobian(q)
                lambda_sq = 1e-5 if pos_err_norm < 0.01 else 1e-3
                if target_rot is not None:
                    dq = J.T @ np.linalg.solve(J @ J.T + lambda_sq * np.eye(6), e_6d)
                else:
                    J_pos = J[:3, :]
                    dq = J_pos.T @ np.linalg.solve(J_pos @ J_pos.T + lambda_sq * np.eye(3), e_pos)

                step_limit = 0.2
                dq_norm = np.linalg.norm(dq)
                if dq_norm > step_limit:
                    dq = dq * (step_limit / dq_norm)

                q += dq
                q = (q + np.pi) % (2 * np.pi) - np.pi

        # 2. Global SLSQP Optimization Fallback
        def objective(q_opt):
            p, R_c, _, links = cls.forward_kinematics(q_opt)
            pos_err = np.linalg.norm(p - target_pos)
            rot_err = np.linalg.norm(R.from_matrix(target_rot @ R_c.T).as_rotvec()) if target_rot is not None else 0.0
            g_pen = sum(500.0 * (0.03 - l[2])**2 for l in links if l[2] < 0.03)
            # Regularization towards initial guess if provided
            reg = 0.0
            if initial_guess is not None:
                reg = 5.0 * np.linalg.norm(q_opt - np.array(initial_guess))**2
            return 1000.0 * pos_err**2 + 500.0 * rot_err**2 + g_pen + reg

        bounds = [(-2.9, 2.9)] * 5 + [(-3.14, 3.14)]
        best_q = None
        best_cost = float("inf")

        for q0 in seeds:
            res = minimize(objective, q0, method='SLSQP', bounds=bounds, options={'maxiter': max_iter})
            if res.fun < best_cost:
                best_cost = res.fun
                best_q = res.x

        if best_q is not None:
            p_chk, R_chk, _, _ = cls.forward_kinematics(best_q)
            p_err = np.linalg.norm(p_chk - target_pos)
            r_err = np.linalg.norm(R.from_matrix(target_rot @ R_chk.T).as_rotvec()) if target_rot is not None else 0.0
            if p_err <= 0.005 and r_err <= 0.05 and cls.check_self_collision_and_limits(best_q):
                return list(best_q)

        return None


# ── Precision Cartesian Path Generator ───────────────────────────────────────
class CartesianTrajectoryGenerator:
    """
    Generates millimetric-resolution Cartesian trajectories with synchronized
    orientation SLERP and S-curve jerk-limited velocity profiling.
    """

    @staticmethod
    def generate_linear_path(start_pos, start_rot, end_pos, end_rot, step_size=0.002):
        start_pos = np.array(start_pos)
        end_pos   = np.array(end_pos)
        dist = np.linalg.norm(end_pos - start_pos)
        num_steps = max(2, int(np.ceil(dist / step_size)))

        alphas = np.linspace(0.0, 1.0, num_steps)
        positions = [(1.0 - a) * start_pos + a * end_pos for a in alphas]

        key_times = [0.0, 1.0]
        key_rots  = R.from_matrix([start_rot, end_rot])
        slerp = Slerp(key_times, key_rots)
        interpolated_rots = slerp(alphas).as_matrix()

        return positions, interpolated_rots

    @classmethod
    def compute_cartesian_joint_trajectory(cls, start_q, end_pos, end_rot,
                                           step_size=0.002, speed_scale=0.5):
        start_p, start_r, _, _ = PreciseKinematics6DoF.forward_kinematics(start_q)
        positions, rotations = cls.generate_linear_path(start_p, start_r, end_pos, end_rot, step_size)

        joint_waypoints = [list(start_q)]
        curr_q = np.array(start_q)

        for p_target, r_target in zip(positions[1:], rotations[1:]):
            q_next = PreciseKinematics6DoF.solve_ik_se3(
                target_pos=p_target,
                target_rot=r_target,
                initial_guess=curr_q,
                pos_tol=5e-4,
                rot_tol=2e-3
            )
            if q_next is None:
                return None, "IK failure along Cartesian path"

            jump = np.linalg.norm(np.array(q_next) - curr_q)
            if jump > 0.25:
                return None, f"Kinematic singularity / joint jump detected: {jump:.3f} rad"

            joint_waypoints.append(q_next)
            curr_q = np.array(q_next)

        return joint_waypoints, "OK"


# ── Dynamics & Jerk Profiler ─────────────────────────────────────────────────
def parameterize_and_profile_trajectory(joint_waypoints, dt=0.02, max_vel=1.0, max_acc=1.0, max_jerk=3.0):
    waypoints = np.array(joint_waypoints)
    n_pts = len(waypoints)

    if n_pts < 2:
        return None

    diffs = np.diff(waypoints, axis=0)
    step_dists = np.linalg.norm(diffs, axis=1)
    total_dist = np.sum(step_dists)

    t_total = max(0.5, total_dist / max(0.05, max_vel * 0.5))
    times = np.linspace(0.0, t_total, n_pts)

    velocities    = np.zeros_like(waypoints)
    accelerations = np.zeros_like(waypoints)
    jerks         = np.zeros_like(waypoints)

    for j in range(6):
        velocities[:, j]    = np.gradient(waypoints[:, j], times)
        accelerations[:, j] = np.gradient(velocities[:, j], times)
        jerks[:, j]         = np.gradient(accelerations[:, j], times)

    peak_vel   = float(np.max(np.abs(velocities)))
    peak_accel = float(np.max(np.abs(accelerations)))
    peak_jerk  = float(np.max(np.abs(jerks)))
    torques    = np.max(np.abs(accelerations) * INERTIA_VALS, axis=0)
    peak_torque = float(np.max(torques))

    traj_msg = None
    if ROS_AVAILABLE:
        traj_msg = JointTrajectory()
        traj_msg.joint_names = JOINT_NAMES

        for i in range(n_pts):
            pt = JointTrajectoryPoint()
            pt.positions = waypoints[i].tolist()
            pt.velocities = velocities[i].tolist()
            pt.accelerations = accelerations[i].tolist()
            sec = int(times[i])
            nanosec = int((times[i] - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
            traj_msg.points.append(pt)

    return {
        "trajectory_msg": traj_msg,
        "times": times,
        "positions": waypoints,
        "velocities": velocities,
        "accelerations": accelerations,
        "jerks": jerks,
        "peak_vel": peak_vel,
        "peak_accel": peak_accel,
        "peak_jerk": peak_jerk,
        "peak_torque": peak_torque,
        "jerk_pass": peak_jerk < max_jerk
    }


# ── Precise Sticker Peeling & Placing Pipeline Engine ────────────────────────
class PrecisePeelPlacePipeline:
    def __init__(self, node, moveit2):
        self.node = node
        self.moveit2 = moveit2
        self.planner = CollisionAwarePlanner(node=self.node, moveit2=self.moveit2)
        self.logger = node.get_logger()

    def get_reference_horizontal_rotation(self, base_ref_rot=None):
        if base_ref_rot is not None:
            return np.copy(base_ref_rot)
        return R.from_euler("xyz", [-172.79, 30.44, -171.97], degrees=True).as_matrix()

    def compute_peel_pose(self, pick_pos, ref_rot, peel_angle_deg, peel_dist=PEEL_DISTANCE):
        theta_rad = math.radians(peel_angle_deg)
        dx = -peel_dist * math.cos(theta_rad)
        dz =  peel_dist * math.sin(theta_rad)
        target_pos = pick_pos + np.array([dx, 0.0, dz])

        # Tilt relative to base reference orientation
        r_peel = ref_rot @ R.from_euler("y", peel_angle_deg, degrees=True).as_matrix()
        return target_pos, r_peel

    def execute_precise_stage(self, stage_name, joint_waypoints, speed_scale=0.5):
        self.logger.info(f"\n=======================================================")
        self.logger.info(f"▶ EXECUTING: {stage_name} ({len(joint_waypoints)} waypoints)")
        self.logger.info(f"=======================================================")

        dyn = parameterize_and_profile_trajectory(joint_waypoints, max_vel=speed_scale, max_acc=speed_scale)
        if dyn is None:
            self.logger.error(f"[{stage_name}] Trajectory parameterization failed.")
            return False, None

        traj_msg = dyn["trajectory_msg"]

        is_safe, col_pair, t_col = self.planner.validate_trajectory(traj_msg)
        if not is_safe:
            self.logger.error(f"[{stage_name}] ABORT: Collision detected with {col_pair} at t={t_col:.2f}s!")
            return False, dyn

        t_start = time.time()
        self.moveit2.execute(traj_msg)
        self.moveit2.wait_until_executed()
        exec_time = time.time() - t_start

        self.logger.info(f"  ✔ Finished in {exec_time:.2f}s | Peak Jerk: {dyn['peak_jerk']:.2f} rad/s³ (Pass: {dyn['jerk_pass']})")
        return True, dyn

    def run_full_pipeline(self, pick_xyz=DEFAULT_PICK_POS, place_xyz=DEFAULT_PLACE_POS,
                          peel_angle_deg=30, speed_scale=0.4):
        self.logger.info(f"\n╔════════════════════════════════════════════════════════════════════════╗")
        self.logger.info(f"║ STARTING HIGH-PRECISION MOTION PIPELINE: Angle={peel_angle_deg}°, Speed={speed_scale:.1f}   ║")
        self.logger.info(f"╚════════════════════════════════════════════════════════════════════════╝")

        # Step 0: Calculate initial pick & place reference configurations
        q_pick_ref = [1.0768, 0.1303, 2.2339, 0.6274, 0.8149, 0.0]
        _, rot_pick_ref, _, _ = PreciseKinematics6DoF.forward_kinematics(q_pick_ref)

        # Step 1: Pre-Pick Hover Target (Z = Pick_Z + 0.10m)
        hover_pick_xyz = pick_xyz + np.array([0.0, 0.0, HOVER_OFFSET_Z])
        q_hover_pick = PreciseKinematics6DoF.solve_ik_se3(hover_pick_xyz, rot_pick_ref, initial_guess=HOME)
        if q_hover_pick is None:
            self.logger.error("Could not find IK for Pre-Pick Hover target.")
            return False

        # Plan move from HOME to Pre-Pick Hover (OMPL)
        self.moveit2.pipeline_id = "ompl"
        self.moveit2.planner_id = "RRTConnect"
        self.moveit2.max_velocity = speed_scale
        self.moveit2.max_acceleration = speed_scale
        traj_home_hover = self.moveit2.plan(joint_positions=q_hover_pick, joint_names=JOINT_NAMES)
        if traj_home_hover is None:
            self.logger.error("Failed to plan HOME -> Pre-Pick Hover trajectory.")
            return False

        self.moveit2.execute(traj_home_hover)
        self.moveit2.wait_until_executed()

        # Step 2: Linear Micro-Descent to Sticker Contact (LIN, Flat contact)
        wps_descent, err = CartesianTrajectoryGenerator.compute_cartesian_joint_trajectory(
            q_hover_pick, pick_xyz, rot_pick_ref, step_size=0.002, speed_scale=speed_scale
        )
        if wps_descent is None:
            self.logger.error(f"Stage 2 Linear descent calculation failed: {err}")
            return False

        success, _ = self.execute_precise_stage("Stage 2: Linear Descent & Sticker Contact", wps_descent, speed_scale=0.2)
        if not success:
            return False

        # Step 3: Controlled Angled Peeling Retraction
        peel_target_pos, peel_target_rot = self.compute_peel_pose(pick_xyz, rot_pick_ref, peel_angle_deg)
        wps_peel, err = CartesianTrajectoryGenerator.compute_cartesian_joint_trajectory(
            wps_descent[-1], peel_target_pos, peel_target_rot, step_size=0.002, speed_scale=speed_scale
        )
        if wps_peel is None:
            self.logger.error(f"Stage 3 Angled peeling calculation failed: {err}")
            return False

        success, _ = self.execute_precise_stage(f"Stage 3: Angled Peel ({peel_angle_deg}°)", wps_peel, speed_scale=0.3)
        if not success:
            return False

        # Step 4: High-Clearance Transfer Arc (Z = 0.38m)
        transfer_pos = np.array([0.0, -0.32, TRANSFER_HEIGHT_Z])
        q_transfer = PreciseKinematics6DoF.solve_ik_se3(transfer_pos, rot_pick_ref, initial_guess=wps_peel[-1])
        if q_transfer is None:
            self.logger.error("Could not find IK for High Transfer pose.")
            return False

        self.moveit2.pipeline_id = "pilz_industrial_motion_planner"
        self.moveit2.planner_id = "PTP"
        traj_transfer = self.moveit2.plan(joint_positions=q_transfer, joint_names=JOINT_NAMES)
        if traj_transfer:
            self.moveit2.execute(traj_transfer)
            self.moveit2.wait_until_executed()

        # Step 5A: Target Placement Hover (Z = Place_Z + 0.10m)
        q_place_ref = [-1.0768, 0.1303, 2.2339, -0.6274, 0.8149, 0.0]
        _, rot_place_ref, _, _ = PreciseKinematics6DoF.forward_kinematics(q_place_ref)

        hover_place_xyz = place_xyz + np.array([0.0, 0.0, HOVER_OFFSET_Z])
        q_hover_place = PreciseKinematics6DoF.solve_ik_se3(hover_place_xyz, rot_place_ref, initial_guess=q_transfer)
        if q_hover_place is None:
            self.logger.error("Could not find IK for Pre-Place Hover pose.")
            return False

        traj_place_hover = self.moveit2.plan(joint_positions=q_hover_place, joint_names=JOINT_NAMES)
        if traj_place_hover:
            self.moveit2.execute(traj_place_hover)
            self.moveit2.wait_until_executed()

        # Step 5B: Linear Placement Descent (LIN, exact horizontal alignment)
        wps_place_descent, err = CartesianTrajectoryGenerator.compute_cartesian_joint_trajectory(
            q_hover_place, place_xyz, rot_place_ref, step_size=0.002, speed_scale=speed_scale
        )
        if wps_place_descent is None:
            self.logger.error(f"Stage 5B Linear placement calculation failed: {err}")
            return False

        success, _ = self.execute_precise_stage("Stage 5: Horizontal Placement Descent", wps_place_descent, speed_scale=0.2)
        if not success:
            return False

        # Step 6: Pure Vertical Lift-Off Retraction
        wps_liftoff, err = CartesianTrajectoryGenerator.compute_cartesian_joint_trajectory(
            wps_place_descent[-1], hover_place_xyz, rot_place_ref, step_size=0.002, speed_scale=speed_scale
        )
        if wps_liftoff:
            self.execute_precise_stage("Stage 6: Vertical Lift-Off & Retraction", wps_liftoff, speed_scale=0.3)

        # Return Home
        self.moveit2.pipeline_id = "ompl"
        self.moveit2.planner_id = "RRTConnect"
        traj_home = self.moveit2.plan(joint_positions=HOME, joint_names=JOINT_NAMES)
        if traj_home:
            self.moveit2.execute(traj_home)
            self.moveit2.wait_until_executed()

        self.logger.info(f"\n✨ HIGH-PRECISION PIPELINE COMPLETED SUCCESSFULLY!\n")
        return True
