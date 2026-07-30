#!/usr/bin/env python3
"""
find_natural_orientation.py — For a given target POSITION (no orientation
constraint), let MoveIt find whatever configuration reaches it, then use
forward kinematics to report the orientation the arm naturally settles on.

This works WITH the arm's 5-DOF limitation instead of fighting it: since
this arm generally can't hit an arbitrary orientation at an arbitrary
position, we let the solver pick orientation freely and just tell you
what it found.
"""

import math
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

# List of positions to test — edit this to whatever points you want mapped
POSITIONS_TO_TEST = [
    [0.0, -0.0595, 1.069],
    [0.0, -0.0595, 0.65],
    [0.0, -0.0595, 0.45],
    [0.2, 0.0, 0.5],
    [0.15, 0.15, 0.35],
]


def quat_to_euler_deg(x, y, z, w):
    """Convert quaternion to Roll/Pitch/Yaw in degrees."""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def main():
    rclpy.init()
    node = Node("find_natural_orientation")
    cb = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name=BASE_LINK,
        end_effector_name=END_EFFECTOR,
        group_name=MOVE_GROUP,
        callback_group=cb,
    )
    moveit2.planner_id = "ptpx"
    moveit2.num_planning_attempts = 10
    moveit2.allowed_planning_time = 10.0
    moveit2.max_velocity = 0.5
    moveit2.max_acceleration = 0.3

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.5)

    print(f"\n{'Position':<25} {'Result':<10} {'Quat xyzw':<40} {'RPY (deg)'}")
    print("-" * 110)

    for pos in POSITIONS_TO_TEST:
        # Position-only goal — quat_xyzw intentionally omitted, orientation left free
        traj = moveit2.plan(position=pos, cartesian=False)

        if traj is None or len(traj.points) < 1:
            print(f"{str(pos):<25} {'FAILED':<10}")
            time.sleep(1.0)
            continue

        # Take the final waypoint's joint configuration
        final_point = traj.points[-1]
        final_joint_positions = list(final_point.positions)

        # Compute forward kinematics for that configuration to see the
        # actual pose (position + orientation) the arm lands on
        fk_result = moveit2.compute_fk(
            joint_state=final_joint_positions,
            fk_link_names=[END_EFFECTOR],
        )

        if fk_result is None:
            print(f"{str(pos):<25} {'FK FAILED':<10}")
            time.sleep(1.0)
            continue

        pose = fk_result.pose if hasattr(fk_result, "pose") else fk_result[0].pose
        q = pose.orientation
        quat_str = f"({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})"
        roll, pitch, yaw = quat_to_euler_deg(q.x, q.y, q.z, q.w)
        rpy_str = f"roll={roll:.1f} pitch={pitch:.1f} yaw={yaw:.1f}"

        print(f"{str(pos):<25} {'SUCCESS':<10} {quat_str:<40} {rpy_str}")
        time.sleep(1.0)

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
