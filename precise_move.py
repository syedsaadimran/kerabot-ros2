#!/usr/bin/env python3
"""
precise_move.py — Reach an EXACT target position precisely, using a real
numerical inverse kinematics solve (not random sampling). Orientation is
left free (this arm's 5-DOF geometry generally can't hold both position
AND an arbitrary orientation), but the POSITION is solved to high
precision, deterministically, every time.

Usage: edit TARGET_POSITION below, then run.
"""

import threading
import time
import numpy as np
from scipy.optimize import least_squares

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from pymoveit2 import MoveIt2

JOINT_NAMES = ["Revolute_1", "Revolute_2", "Revolute_3", "Revolute_4", "Revolute_5"]
BASE_LINK = "base_link"
END_EFFECTOR = "L70IE_Finger"
MOVE_GROUP = "arm"
JOINT_LIMITS = [(-3.14, 3.14)] * 5

# ---------------------------------------------------------------------
# TARGET — the exact position you want the end effector to reach
# ---------------------------------------------------------------------
TARGET_POSITION = [0.2, 0.1, 0.5]
POSITION_TOLERANCE_M = 0.001   # 1mm — how precise the solve must be
EXECUTE = True                 # True = actually move the robot, False = solve + report only


class FKSolver:
    """Wraps MoveIt's compute_fk service for use inside a numerical optimizer."""
    def __init__(self, moveit2):
        self.moveit2 = moveit2
        self.call_count = 0

    def position_error(self, joint_angles):
        self.call_count += 1
        fk = self.moveit2.compute_fk(
            joint_state=list(joint_angles),
            fk_link_names=[END_EFFECTOR],
        )
        if fk is None:
            return np.array([1e3, 1e3, 1e3])  # heavy penalty if FK fails
        pose = fk.pose if hasattr(fk, "pose") else fk[0].pose
        p = pose.position
        return np.array([
            p.x - TARGET_POSITION[0],
            p.y - TARGET_POSITION[1],
            p.z - TARGET_POSITION[2],
        ])


def main():
    rclpy.init()
    node = Node("precise_move")
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

    solver = FKSolver(moveit2)
    lower = [lo for lo, hi in JOINT_LIMITS]
    upper = [hi for lo, hi in JOINT_LIMITS]

    node.get_logger().info(f"Solving precise IK for position={TARGET_POSITION} ...")

    best_result = None
    best_residual = float("inf")

    # Try multiple starting seeds — improves odds of finding a solution
    # since this is a nonlinear, potentially multi-solution problem
    seeds = [
        [0.0, 0.0, 0.0, 0.0, 0.0],
        [0.5, -0.5, 0.5, -0.5, 0.5],
        [-0.5, 0.5, -0.5, 0.5, -0.5],
        [1.0, 1.0, -1.0, -1.0, 1.0],
        [-1.0, -1.0, 1.0, 1.0, -1.0],
    ]

    for seed in seeds:
        result = least_squares(
            solver.position_error,
            x0=seed,
            bounds=(lower, upper),
            method="trf",
            xtol=1e-10,
            ftol=1e-10,
            max_nfev=150,
        )
        residual_norm = float(np.linalg.norm(result.fun))
        if residual_norm < best_residual:
            best_residual = residual_norm
            best_result = result
        if residual_norm < POSITION_TOLERANCE_M:
            break  # good enough, stop early

    node.get_logger().info(
        f"Best solution found after {solver.call_count} FK evaluations. "
        f"Position error: {best_residual*1000:.3f} mm"
    )

    if best_residual > POSITION_TOLERANCE_M:
        node.get_logger().error(
            f"Could NOT reach target within {POSITION_TOLERANCE_M*1000:.1f}mm "
            f"tolerance — target likely outside reachable workspace."
        )
        rclpy.shutdown()
        spin_thread.join()
        return

    solved_joints = best_result.x.tolist()
    node.get_logger().info(f"Solved joint angles (rad): {[round(j,4) for j in solved_joints]}")

    # Confirm with one final FK check and report resulting orientation
    fk = moveit2.compute_fk(joint_state=solved_joints, fk_link_names=[END_EFFECTOR])
    pose = fk.pose if hasattr(fk, "pose") else fk[0].pose
    p, q = pose.position, pose.orientation
    node.get_logger().info(
        f"Verified position: ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})  "
        f"Resulting orientation quat: ({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})"
    )

    if EXECUTE:
        node.get_logger().info("Executing move to solved joint configuration...")
        moveit2.move_to_configuration(solved_joints)
        moveit2.wait_until_executed()
        node.get_logger().info("Motion complete.")
    else:
        node.get_logger().info("EXECUTE=False — dry run only, robot did not move.")

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
