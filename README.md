# Kerabot — 6DoF Robotic Arm Software Stack

> A complete ROS2 Humble + MoveIt2 software stack for the **Kerabot** 5-actuated DoF robotic arm,
> featuring jerk-limited trapezoidal motion via **Pilz PTP + Ruckig smoothing**,
> persistent **ground plane & self-collision detection**, dynamic pipeline switching to OMPL,
> and a complete **5-stage industrial Pick & Place simulation suite**.

---

## Table of Contents

1. [Hardware & Kinematics Overview](#hardware--kinematics-overview)
2. [Software Architecture & Pipelines](#software-architecture--pipelines)
3. [Quick Setup & Verification Checklist](#quick-setup--verification-checklist)
4. [Ground Plane & Collision Management](#ground-plane--collision-management)
5. [Stress Test & Verification Suite](#stress-test--verification-suite)
   - [add_ground_plane.py — Environment Setup](#add_ground_planepy--environment-setup)
   - [verify_ground_collision.py — Ground Collision Test](#verify_ground_collisionpy--ground-collision-test)
   - [collision_stress_test.py — Self-Collision Limit Finder](#collision_stress_testpy--self-collision-limit-finder)
   - [pipeline_stress_test.py — Pipelining & Switching Test](#pipeline_stress_testpy--pipelining--switching-test)
   - [pick_place_industrial_sim.py — Full 5-Stage Pick & Place](#pick_place_industrial_simpy--full-5-stage-pick--place)
   - [pick_place_stress_test.py — Pick Viability Evaluator](#pick_place_stress_testpy--pick-viability-evaluator)
   - [stress_test.py — 23-Profile Motion Stress Test](#stress_testpy--23-profile-motion-stress-test)
   - [manual_move.py — Interactive Validated Planner](#manual_movepy--interactive-validated-planner)
6. [Pilz PTP + Ruckig Smoothing Architecture](#pilz-ptp--ruckig-smoothing-architecture)
7. [Troubleshooting & Verification Commands](#troubleshooting--verification-commands)

---

## Hardware & Kinematics Overview

| Property | Value |
|---|---|
| DOF | 5 actuated revolute joints + end effector |
| MoveIt planning group | `arm` (Revolute_1 – Revolute_5) |
| End effector link | `L70IE_Finger` |
| Base mounting link | `base_link` (Z = 0.00m table level) |
| Peak joint torque limit | 5.0 N·m |
| Max joint velocity | 2.79 rad/s (~160 °/s) |

### Joint Axes & Angular Boundaries
- **Revolute_1** (base yaw) — rotation about **Z**: `[-2.95, +2.95] rad` (Full Range Clear)
- **Revolute_2** (shoulder pitch) — rotation about **X**: `[-1.53, +1.53] rad` (Restricted by table & base self-collision)
- **Revolute_3** (elbow pitch) — rotation about **X**: `[-2.75, +2.75] rad` (Self-collision limited)
- **Revolute_4** (forearm roll) — rotation about **Z**: `[-2.95, +2.95] rad` (Full Range Clear)
- **Revolute_5** (wrist pitch) — rotation about **X**: `[-2.95, +2.95] rad` (Full Range Clear)

---

## Software Architecture & Pipelines

```text
kerabot_ws/
├── src/
│   ├── kerabot_description/       # URDF geometry, meshes, launch
│   ├── kerabot_moveit_config/     # MoveIt2 configuration & adapters
│   │   ├── config/ompl_planning.yaml                  (TOTG adapter for OMPL)
│   │   ├── config/pilz_industrial_motion_planner.yaml (request_adapters: "")
│   │   └── launch/demo.launch.py                      (default: pilz, secondary: ompl)
│   └── pymoveit2/                 # Python MoveIt2 API wrapper
├── add_ground_plane.py            ← Adds 2m x 2m x 0.1m collision box (Z_top = 0.0m)
├── verify_ground_collision.py    ← Verifies MoveIt rejects Z < 0.0m moves
├── collision_stress_test.py      ← Sweeps self-collision joint limits
├── pipeline_stress_test.py       ← Chained waypoints & Pilz <-> OMPL switching
├── pick_place_industrial_sim.py  ← Full 5-stage Pick & Place transfer sequence
├── pick_place_stress_test.py     ← 16-point pick orientation & clearance evaluator
├── stress_test.py                ← 23-profile joint-space & boundary test
└── manual_move.py                ← Interactive 4-step validated mover
```

---

## Quick Setup & Verification Checklist

Always execute setup & verification steps in the following order:

```bash
# 1. Source ROS 2 Humble & workspace setup
source /opt/ros/humble/setup.bash
source ~/kerabot_ws/install/setup.bash

# 2. Launch MoveIt 2 + RViz
ros2 launch kerabot_moveit_config demo.launch.py

# 3. Add ground plane collision object (in a 2nd terminal)
python3 add_ground_plane.py

# 4. Verify ground collision rejection
python3 verify_ground_collision.py

# 5. Run motion pipelining & pipeline switching test
python3 pipeline_stress_test.py

# 6. Run full 5-stage industrial pick-and-place simulation
python3 pick_place_industrial_sim.py
```

---

## Ground Plane & Collision Management

MoveIt does not persist planning scene collision objects inside URDF/SRDF files. `add_ground_plane.py` adds a **persistent table collision box** to MoveIt's planning scene via ROS 2 `/apply_planning_scene`:

* **Dimensions**: 2.0m (X) x 2.0m (Y) x 0.10m (Z thickness)
* **Mounting Plane**: Center at `Z = -0.05m`, setting the top table surface **at Z = 0.00m** (exact base_link mounting plane).
* **Collision Behavior**: Any trajectory attempting to drive the end-effector or intermediate links into `Z < 0.00m` is rejected by MoveIt FCL with `INVALID_MOTION_PLAN`.

```bash
# Add table collision plane:
python3 add_ground_plane.py

# Remove table collision plane:
python3 add_ground_plane.py --remove
```

---

## Stress Test & Verification Suite

### `verify_ground_collision.py`
Verifies ground plane collision detection by executing:
1. Valid above-ground move (`Z = +0.40m`) -> **PASSED**
2. Downward fold (`Z < 0.00m`) -> **REJECTED**
3. Extreme downward fold (`Z < -0.20m`) -> **REJECTED**

### `collision_stress_test.py`
Sweeps each of the 5 joints in fine steps to map out the exact angular boundaries where link-to-link or link-to-base collisions occur.

### `pipeline_stress_test.py`
Tests two core MoveIt operational features:
1. **Chained Continuous Pipelining**: Executes 8 back-to-back waypoints in sequence without returning home. (Pass rate: 100%)
2. **Dynamic Pipeline Switching**: Alternates dynamically between `pilz_industrial_motion_planner` (PTP) and `ompl` (RRTConnect) while changing velocity scaling factors (0.3 -> 0.5 -> 0.7 -> 0.4). (Pass rate: 100%)

### `pick_place_industrial_sim.py`
Executes 6 complete 5-stage Pick & Place transfer workflows:
```text
Stage 1: Pre-Pick Approach  (X_pick,  Y_pick,  Z_hover = 0.35m)
Stage 2: Pick Descent       (X_pick,  Y_pick,  Z_pick  = 0.22m)
Stage 3: Post-Pick Lift     (X_pick,  Y_pick,  Z_hover = 0.35m)
Stage 4: Transport & Place  (X_place, Y_place, Z_place = 0.22m)
Stage 5: Retract & Reset    (Home [0,0,0,0,0])
```
* **Pass Rate**: **6/6 Transfers Passed (30/30 sub-steps completed)** using Pilz PTP + Ruckig trapezoidal dynamics with active ground plane checking.

---

## Pilz PTP + Ruckig Smoothing Architecture

```text
 Pilz PTP / LIN (Trapezoidal profile)
      │
      ▼
 AddRuckigTrajectorySmoothing  ← Rounds velocity corners into jerk-limited S-curves
      │                          Enforces max_jerk limits from joint_limits.yaml
      ▼
 joint_limits.yaml             (Velocity, acceleration, and jerk caps)
      │
      ▼
 arm_controller (FollowJointTrajectory)
```

---

## Troubleshooting & Verification Commands

### MoveIt reports `INVALID_MOTION_PLAN`
1. Check `move_group` logs: `tail -n 25 ~/.ros/log/move_group_*.log`
2. Check if the target configuration penetrates the ground plane (`Z < 0.00m`) or violates joint bounds (`Revolute_2 < -1.53 rad`).

### Verify Planners are loaded in ROS 2
```bash
ros2 param get /move_group default_planning_pipeline
# Should return: pilz_industrial_motion_planner

ros2 param get /move_group planning_pipelines
# Should return: ['ompl', 'pilz_industrial_motion_planner']
```
