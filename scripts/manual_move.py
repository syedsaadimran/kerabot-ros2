#!/usr/bin/env python3
"""
manual_move.py — Interactive Validated Motion Planner for Kerabot
==================================================================
Move the robot to any position with full validation:

  STEP 1 — Input validation:
            • Workspace bounding box check (derived from URDF reach)
            • Quaternion normalization check
            • Joint angle range check (--joint mode)

  STEP 2 — IK / planning dry-run:
            • Calls MoveIt plan() to verify the pose is reachable
            • Reports trajectory duration and waypoint count
            • Aborts if no valid IK solution exists

  STEP 3 — User confirmation:
            • Prints a summary and prompts "Execute? [y/N]"
            • Robot does NOT move until you confirm

  STEP 4 — Execution + post-move verification:
            • Executes the trajectory via Pilz PTP + Ruckig
            • Reads back joint states after completion
            • Reports final position vs target (PASS / WARN)

Modes
-----
  --pose x y z roll pitch yaw   Move to Cartesian pose (RPY in degrees)
  --joint q1 q2 q3 q4 q5        Move to joint configuration (radians)
  --home                         Return to all-zeros home position

Options
-------
  --vel   FLOAT    Velocity scaling 0–1     (default 0.5)
  --accel FLOAT    Acceleration scaling 0–1 (default 0.3)
  --cartesian      Use Pilz LIN (straight Cartesian line) instead of PTP
  --yes            Skip confirmation prompt (auto-confirm — use with care)
  --dry-run        Plan only, never execute regardless of confirmation

Examples
--------
  python3 manual_move.py --pose 0.0 -0.06 0.65 0 90 0
  python3 manual_move.py --pose 0.1 -0.1 0.5 0 0 0 --vel 0.3 --accel 0.2
  python3 manual_move.py --joint 0.3 -0.5 0.4 0.0 0.2
  python3 manual_move.py --home
  python3 manual_move.py --pose 0.0 -0.06 0.65 0 90 0 --dry-run
"""

import argparse
import math
import sys
import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2
from sensor_msgs.msg import JointState


# ─────────────────────────────────────────────────────────────────────────────
# ROBOT CONFIG
# ─────────────────────────────────────────────────────────────────────────────
JOINT_NAMES  = ["Revolute_1", "Revolute_2", "Revolute_3", "Revolute_4", "Revolute_5"]
BASE_LINK    = "base_link"
END_EFFECTOR = "L70IE_Finger"
MOVE_GROUP   = "arm"

# Joint limits (rad) — from URDF, symmetric
JOINT_LIMITS = [
    (-2.967, 2.967),   # Revolute_1
    (-2.967, 2.967),   # Revolute_2
    (-2.967, 2.967),   # Revolute_3
    (-2.967, 2.967),   # Revolute_4
    (-2.967, 2.967),   # Revolute_5
]

# Workspace bounding box (metres, base_link frame)
# Conservative estimates derived from URDF link lengths + robot geometry
WS_X = (-0.70, 0.70)
WS_Y = (-0.70, 0.70)
WS_Z = ( 0.05, 1.10)

# Minimum reachable radius from base (avoid singularity near base)
WS_R_MIN = 0.05
WS_R_MAX = 0.70

HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0]


# ─────────────────────────────────────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────────────────────────────────────
def euler_to_quat(roll_deg, pitch_deg, yaw_deg):
    r, p, y = math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def quat_norm(q):
    return math.sqrt(sum(v * v for v in q))


def validate_pose(x, y, z, roll, pitch, yaw):
    """
    Check workspace bounds and quaternion validity.
    Returns (ok: bool, errors: list[str]).
    """
    errors = []

    # Bounding box
    if not (WS_X[0] <= x <= WS_X[1]):
        errors.append(f"x={x:.3f} outside workspace [{WS_X[0]}, {WS_X[1]}] m")
    if not (WS_Y[0] <= y <= WS_Y[1]):
        errors.append(f"y={y:.3f} outside workspace [{WS_Y[0]}, {WS_Y[1]}] m")
    if not (WS_Z[0] <= z <= WS_Z[1]):
        errors.append(f"z={z:.3f} outside workspace [{WS_Z[0]}, {WS_Z[1]}] m")

    # Reachability radius in XY plane
    r = math.sqrt(x * x + y * y)
    if r < WS_R_MIN:
        errors.append(f"XY radius={r:.3f} m too close to base (min {WS_R_MIN} m). "
                      "Risk of singularity.")
    if r > WS_R_MAX:
        errors.append(f"XY radius={r:.3f} m likely out of reach (max ~{WS_R_MAX} m).")

    return len(errors) == 0, errors


def validate_joints(angles):
    """Check all joint angles are within URDF limits."""
    errors = []
    for i, (angle, (lo, hi)) in enumerate(zip(angles, JOINT_LIMITS)):
        if not (lo <= angle <= hi):
            errors.append(
                f"Revolute_{i+1}: {angle:.3f} rad outside [{lo:.3f}, {hi:.3f}] rad "
                f"(= [{math.degrees(lo):.1f}°, {math.degrees(hi):.1f}°])"
            )
    return len(errors) == 0, errors


def print_banner(title):
    w = 68
    print("\n" + "=" * w)
    print(f"  {title}")
    print("=" * w)


def print_ok(msg):  print(f"  [OK]  {msg}")
def print_warn(msg): print(f"  [!!]  {msg}")
def print_info(msg): print(f"  [>>]  {msg}")
def print_fail(msg): print(f"  [XX]  {msg}")


# ─────────────────────────────────────────────────────────────────────────────
# JOINT STATE READER
# ─────────────────────────────────────────────────────────────────────────────
class JointStateReader(Node):
    def __init__(self):
        super().__init__("kerabot_joint_reader")
        self._positions = {}
        self.create_subscription(
            JointState, "/joint_states", self._cb, 10
        )

    def _cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self._positions[name] = pos

    def get(self, joint_names):
        return [self._positions.get(n, float("nan")) for n in joint_names]


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(
        description="Kerabot validated interactive motion planner",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("Examples")[1] if "Examples" in __doc__ else ""
    )
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--pose",  nargs=6, type=float,
                      metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
                      help="Cartesian target: x y z roll pitch yaw (RPY in degrees)")
    mode.add_argument("--joint", nargs=5, type=float,
                      metavar=("Q1", "Q2", "Q3", "Q4", "Q5"),
                      help="Joint target: 5 angles in radians")
    mode.add_argument("--home",  action="store_true",
                      help="Return to home (all zeros)")

    p.add_argument("--vel",       type=float, default=0.5)
    p.add_argument("--accel",     type=float, default=0.3)
    p.add_argument("--cartesian", action="store_true",
                   help="Use Pilz LIN (straight-line Cartesian path)")
    p.add_argument("--yes",       action="store_true",
                   help="Auto-confirm execution (skip [y/N] prompt)")
    p.add_argument("--dry-run",   action="store_true",
                   help="Plan only, never execute")
    return p.parse_args()


def main():
    args = parse_args()

    # ── STEP 1: Input Validation ──────────────────────────────────────────────
    print_banner("KERABOT Manual Move — Input Validation")

    if args.home:
        target_joints = HOME_JOINTS
        mode          = "joint"
        print_info("Mode: HOME (all zeros)")

    elif args.joint:
        target_joints = args.joint
        mode          = "joint"
        print_info(f"Mode: Joint-space  |  angles = {[f'{a:.3f}' for a in target_joints]} rad")
        ok, errs = validate_joints(target_joints)
        for e in errs:
            print_warn(e)
        if not ok:
            print_fail("Joint angle validation failed. Aborting.")
            sys.exit(1)
        print_ok("Joint limits check passed.")

    else:  # --pose
        x, y, z, roll, pitch, yaw = args.pose
        quat = euler_to_quat(roll, pitch, yaw)
        mode = "pose"
        print_info(f"Mode: Cartesian pose")
        print_info(f"  Position : x={x:.3f}  y={y:.3f}  z={z:.3f}  (metres)")
        print_info(f"  Euler RPY: roll={roll:.1f}°  pitch={pitch:.1f}°  yaw={yaw:.1f}°")
        print_info(f"  Quat xyzw: {[f'{v:.4f}' for v in quat]}")

        # Workspace check
        ok, errs = validate_pose(x, y, z, roll, pitch, yaw)
        for e in errs:
            print_warn(e)
        if not ok:
            print_fail("Workspace validation failed. Aborting.")
            sys.exit(1)
        print_ok("Workspace bounds check passed.")

        # Quaternion norm
        n = quat_norm(quat)
        if abs(n - 1.0) > 0.01:
            print_warn(f"Quaternion norm = {n:.5f} (expected ~1.0) — will be normalised by MoveIt.")
        else:
            print_ok(f"Quaternion norm = {n:.6f} (valid).")

    # ── ROS Init ─────────────────────────────────────────────────────────────
    rclpy.init()

    node     = Node("kerabot_manual_move")
    js_node  = JointStateReader()
    cb       = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name=BASE_LINK,
        end_effector_name=END_EFFECTOR,
        group_name=MOVE_GROUP,
        callback_group=cb,
    )
    moveit2.pipeline_id           = "pilz_industrial_motion_planner"
    moveit2.planner_id            = "LIN" if args.cartesian else "PTP"
    moveit2.num_planning_attempts = 10
    moveit2.allowed_planning_time = 15.0
    moveit2.max_velocity          = args.vel
    moveit2.max_acceleration      = args.accel

    executor = MultiThreadedExecutor(3)
    executor.add_node(node)
    executor.add_node(js_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.2)

    # ── STEP 2: IK / Planning Dry-Run ────────────────────────────────────────
    print_banner("STEP 2 — IK & Planning Verification (dry-run)")
    planner = "LIN" if args.cartesian else "PTP"
    print_info(f"Planner: Pilz {planner} + Ruckig  |  vel={args.vel}  accel={args.accel}")

    t0 = time.monotonic()

    if mode == "joint" or args.home:
        traj = moveit2.plan(joint_positions=target_joints)
    else:
        traj = moveit2.plan(
            position=[x, y, z],
            quat_xyzw=quat,
            cartesian=args.cartesian,
        )

    plan_time = time.monotonic() - t0

    if traj is None:
        print_fail(f"Planning failed after {plan_time:.1f}s — pose may be unreachable.")
        print_fail("Possible causes:")
        print_fail("  • Position is in collision or outside robot reach")
        print_fail("  • IK solver could not find a valid configuration")
        print_fail("  • MoveIt is not running — start it first with demo.launch.py")
        rclpy.shutdown()
        spin_thread.join()
        sys.exit(1)

    pts      = traj.points
    n_pts    = len(pts)
    duration = pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec * 1e-9 \
               if n_pts > 0 else 0.0

    if n_pts < 2:
        print_warn("Trajectory has 1 waypoint — robot is already at the target position.")
        print_warn("No motion needed.")
        rclpy.shutdown()
        spin_thread.join()
        sys.exit(0)

    print_ok(f"Planning succeeded in {plan_time:.2f}s")
    print_ok(f"Trajectory: {n_pts} waypoints  |  duration: {duration:.3f}s")

    # Per-joint travel summary
    jnames = traj.joint_names
    n_j    = len(jnames)
    times  = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                       for p in pts])
    vel_arr  = np.array([[p.velocities[j]    if p.velocities    else 0.0
                          for j in range(n_j)] for p in pts])
    jerk_arr = np.gradient(vel_arr, times, axis=0)

    print(f"\n  {'Joint':<14} {'Travel (rad)':>13} {'Peak Vel':>10} {'Peak Jerk':>10}")
    print(f"  {'-'*50}")
    for j, jn in enumerate(jnames):
        positions = [p.positions[j] for p in pts]
        travel    = max(positions) - min(positions)
        pk_vel    = float(np.max(np.abs(vel_arr[:, j])))
        pk_jerk   = float(np.max(np.abs(jerk_arr[:, j])))
        print(f"  {jn:<14} {travel:>13.4f} {pk_vel:>10.4f} {pk_jerk:>10.4f}")

    # ── STEP 3: User Confirmation ─────────────────────────────────────────────
    print_banner("STEP 3 — Confirm Execution")
    if args.dry_run:
        print_info("DRY-RUN mode — skipping execution.")
        rclpy.shutdown()
        spin_thread.join()
        sys.exit(0)

    if args.yes:
        print_info("Auto-confirmed (--yes flag).")
        confirmed = True
    else:
        print_info("Review the plan above. The robot WILL MOVE if you confirm.")
        try:
            answer = input("\n  Execute? [y/N] > ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            answer = "n"
        confirmed = answer in ("y", "yes")

    if not confirmed:
        print_info("Cancelled — no motion performed.")
        rclpy.shutdown()
        spin_thread.join()
        sys.exit(0)

    # ── STEP 4: Execute + Post-move Verification ──────────────────────────────
    print_banner("STEP 4 — Execution")
    print_info(f"Executing Pilz {planner} trajectory...")

    # Read start joint state
    start_joints = js_node.get(JOINT_NAMES)

    moveit2.execute(traj)
    moveit2.wait_until_executed()
    time.sleep(0.5)   # settle

    print_ok("Motion command complete.")

    # Read final joint state
    final_joints = js_node.get(JOINT_NAMES)

    print_banner("STEP 4 — Post-move Verification")
    if mode in ("joint",) or args.home:
        target = target_joints
        print(f"\n  {'Joint':<14} {'Target (rad)':>13} {'Actual (rad)':>13} {'Error (rad)':>12} Status")
        print(f"  {'-'*60}")
        all_pass = True
        for i, jn in enumerate(JOINT_NAMES):
            tgt  = float(target[i])
            act  = final_joints[i]
            err  = abs(act - tgt) if not math.isnan(act) else float("nan")
            ok   = err < 0.05 if not math.isnan(err) else False
            if not ok:
                all_pass = False
            status = "[OK]" if ok else "[WARN]"
            print(f"  {jn:<14} {tgt:>13.4f} {act:>13.4f} {err:>12.4f}  {status}")
        print()
        if all_pass:
            print_ok("All joints within tolerance (< 0.05 rad). PASS.")
        else:
            print_warn("Some joints outside tolerance. Check for slippage or controller issues.")
    else:
        # Pose mode — just report final joints for now
        print_info("Final joint positions after move:")
        for jn, val in zip(JOINT_NAMES, final_joints):
            flag = "" if not math.isnan(val) else "  [no data]"
            print(f"    {jn}: {val:.4f} rad{flag}")
        print_ok("Motion complete. Verify end-effector position visually in RViz.")

    print("\n" + "=" * 68 + "\n")

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
