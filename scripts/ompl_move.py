#!/usr/bin/env python3
"""
ompl_move.py — Position-Only Goal via OMPL RRTConnect
======================================================
Sends a POSITION-ONLY Cartesian goal (no orientation constraint) so that
the IK solver is free to choose any valid joint configuration that reaches
the XYZ target. Uses OMPL/RRTConnect, which accepts partial Cartesian goals.

Pilz PTP/LIN CANNOT be used for this because:
  - Pilz requires a FULL pose (position + orientation) OR a joint-space goal.
  - Sending only position → "Only cartesian XOR joint goal allowed" error.

This script uses MoveIt2.move_to_pose() with quat_xyzw=None, which sends
only a PositionConstraint (no OrientationConstraint) in the planning request.

Usage:
    # Move to a position, IK chooses orientation:
    python3 ompl_move.py --pos 0.0 -0.06 0.65

    # Execute on the real robot:
    python3 ompl_move.py --pos 0.0 -0.06 0.65 --execute

    # Slower speed:
    python3 ompl_move.py --pos 0.0 -0.06 0.65 --vel 0.3 --accel 0.2

    # Set a tolerance on the position goal (default 0.01 m):
    python3 ompl_move.py --pos 0.0 -0.06 0.65 --tolerance 0.05
"""

import argparse
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2


# ─────────────────────────────────────────────────────────────────────────────
# ROBOT CONFIG
# ─────────────────────────────────────────────────────────────────────────────
JOINT_NAMES  = ["Revolute_1", "Revolute_2", "Revolute_3", "Revolute_4", "Revolute_5"]
BASE_LINK    = "base_link"
END_EFFECTOR = "L70IE_Finger"
MOVE_GROUP   = "arm"


def parse_args():
    p = argparse.ArgumentParser(description="Position-only goal via OMPL")
    p.add_argument("--pos",       nargs=3, type=float, required=True,
                   metavar=("X", "Y", "Z"),
                   help="Target position in base_link frame (metres)")
    p.add_argument("--vel",       type=float, default=0.5,
                   help="Velocity scaling 0-1 (default 0.5)")
    p.add_argument("--accel",     type=float, default=0.3,
                   help="Acceleration scaling 0-1 (default 0.3)")
    p.add_argument("--tolerance", type=float, default=0.01,
                   help="Position tolerance in metres (default 0.01)")
    p.add_argument("--execute",   action="store_true",
                   help="Execute on the real robot (default: dry-run, plan only)")
    p.add_argument("--planner",   default="RRTConnect",
                   choices=["RRTConnect", "RRT", "RRTstar", "PRM", "PRMstar",
                            "BKPIECE", "EST", "LBKPIECE", "KPIECE", "SBL",
                            "TRRT", "SPARS"],
                   help="OMPL planner to use (default: RRTConnect)")
    p.add_argument("--timeout",   type=float, default=15.0,
                   help="Planning timeout in seconds (default 15.0)")
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()

    node = Node("kerabot_ompl_position_move")
    cb   = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name=BASE_LINK,
        end_effector_name=END_EFFECTOR,
        group_name=MOVE_GROUP,
        callback_group=cb,
    )

    # ── Route to OMPL pipeline ────────────────────────────────────────────────
    # In MoveIt2 Humble, the MoveGroup action has two separate fields:
    #   - pipeline_id: selects which pipeline ("ompl", "pilz_industrial_motion_planner")
    #   - planner_id:  selects the algorithm within that pipeline ("RRTConnect", "PTP", etc.)
    # pymoveit2 ≥ 0.10 exposes both.
    moveit2.pipeline_id             = "ompl"
    moveit2.planner_id              = args.planner
    moveit2.num_planning_attempts   = 10
    moveit2.allowed_planning_time   = args.timeout
    moveit2.max_velocity            = args.vel
    moveit2.max_acceleration        = args.accel

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.0)

    x, y, z = args.pos
    dry = "DRY-RUN" if not args.execute else "EXECUTE"
    node.get_logger().info(
        f"[ompl_move] Pipeline=ompl  Planner={args.planner}  "
        f"Target=({x:.3f},{y:.3f},{z:.3f})  tol={args.tolerance}  {dry}"
    )

    if args.execute:
        # Move and wait
        moveit2.move_to_pose(
            position=[x, y, z],
            quat_xyzw=None,         # position-only goal — IK chooses orientation
            cartesian=False,
        )
        moveit2.wait_until_executed()
        node.get_logger().info("[ompl_move] Done.")
    else:
        # Plan only — inspect result
        traj = moveit2.plan(
            position=[x, y, z],
            quat_xyzw=None,         # position-only goal
            cartesian=False,
        )
        if traj is None:
            node.get_logger().error(
                "[ompl_move] Planning FAILED.\n"
                "  Possible causes:\n"
                "  1. OMPL pipeline not loaded — did you restart MoveIt after\n"
                "     applying the config fix?  Run: colcon build --symlink-install\n"
                "     then restart demo.launch.py\n"
                "  2. Position is outside robot workspace\n"
                "  3. IK timeout — try --timeout 30.0\n"
                "  4. Check: ros2 param get /move_group planning_pipelines"
            )
        else:
            pts = traj.points
            dur = pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec*1e-9 \
                  if pts else 0.0
            node.get_logger().info(
                f"[ompl_move] Plan OK: {len(pts)} waypoints, {dur:.3f}s\n"
                f"  Add --execute to run on the robot."
            )

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
