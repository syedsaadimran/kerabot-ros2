# Kerabot — 6-DoF Robotic Arm Software Stack

> A complete ROS 2 Humble + MoveIt 2 software stack for the **Kerabot 6-DoF** robotic arm,
> featuring an active rotating 6th axis (`ee_rotation_joint`), a $329 \times 267 \times 100\text{ mm}$ end-effector box payload (`end_effector_box_link`),
> jerk-limited trapezoidal motion via **Pilz PTP / LIN + Ruckig smoothing**,
> persistent **ground plane & strict self-collision detection**, dynamic pipeline switching to OMPL,
> and a comprehensive **Sticker Pick, Peel & Place Trajectory Benchmark Suite**.

---

## Table of Contents

1. [Hardware & Kinematics Overview](#hardware--kinematics-overview)
2. [Software Architecture & Layout](#software-architecture--layout)
3. [Quick Setup & Verification Checklist](#quick-setup--verification-checklist)
4. [Sticker Pick, Peel & Place Benchmark Suite](#sticker-pick-peel--place-benchmark-suite)
5. [Motion Scripts & Test Suite](#motion-scripts--test-suite)
6. [Ground Plane & Strict Collision Safety](#ground-plane--strict-collision-safety)
7. [Pilz PTP / LIN + Ruckig Dynamics Architecture](#pilz-ptp--lin--ruckig-dynamics-architecture)
8. [Troubleshooting & Verification Commands](#troubleshooting--verification-commands)

---

## Hardware & Kinematics Overview

| Property | Value |
|---|---|
| **DOF** | 6 active revolute joints (`Revolute_1` – `Revolute_5` + `ee_rotation_joint`) |
| **MoveIt Planning Group** | `arm` (`base_link` $\rightarrow$ `end_effector_box_link`) |
| **End Effector Payload** | `end_effector_box_link` ($0.329 \text{ m} \times 0.267 \text{ m} \times 0.100 \text{ m}$) |
| **Base Mounting Link** | `base_link` (Z = 0.00m table level) |
| **Peak Joint Torque Limit** | 5.0 N·m (Joints 1–5), 10.0 N·m (Joint 6) |
| **Max Velocity Limits** | 2.79 rad/s (Joints 1–5), 3.14 rad/s (Joint 6) |

### Joint Axes & Kinematic Boundaries
- **Revolute_1** (base yaw) — rotation about **Z**: `[-2.90, +2.90] rad`
- **Revolute_2** (shoulder pitch) — rotation about **X**: `[-2.90, +2.90] rad`
- **Revolute_3** (elbow pitch) — rotation about **X**: `[-2.90, +2.90] rad`
- **Revolute_4** (forearm roll) — rotation about **Z**: `[-2.90, +2.90] rad`
- **Revolute_5** (wrist pitch) — rotation about **X**: `[-2.90, +2.90] rad`
- **ee_rotation_joint** (active 6th axis EE rotation) — rotation about **Z**: `[-3.14159, +3.14159] rad`

### Kinematic Tree (`check_urdf`)
```text
base_link
  └── L110I_Shoulder
      └── L110I_shoulder_2
          └── J2J3_Shoulder
              └── Wrist_Motor
                  └── L70IE_Finger
                      └── end_effector_box_link (Kinematic Tip)
```

---

## Software Architecture & Layout

```text
kerabot_ws/
├── src/
│   ├── Robot_to_URDF_New_Pakka_description/  # URDF xacro geometry, meshes, CMakeLists
│   ├── kerabot_moveit_config/                # MoveIt 2 configuration, SRDF, joint limits
│   └── pymoveit2/                            # Python MoveIt 2 API wrapper
├── scripts/                                   # All executable motion & benchmark scripts
│   ├── peel_place_benchmark_suite.py         # 6-DoF Sticker Pick, Peel & Place Trajectory Benchmark Suite
│   ├── pick_place_industrial_sim.py          # 6-DoF 5-Stage Pick & Place Simulation
│   ├── pipeline_stress_test.py               # Chained Waypoint & Dynamic Pipeline Switching Test
│   ├── stress_test.py                        # 3-Category Joint Sweep & Limit Stress Test
│   ├── collision_stress_test.py              # Self-Collision Boundary Finder
│   ├── compare_profiles.py                   # Trajectory Dynamics & Profile Comparisons
│   ├── manual_move.py                        # Interactive Validated Planner
│   └── add_ground_plane.py                   # Ground Plane Collision Publisher
└── results/                                   # Exported PNG visual analytics & plots
    ├── peel_benchmark_summary.png            # Sticker peel execution & jerk metrics
    ├── peel_benchmark_velocity.png           # Speed scaling factor curves (0.2 -> 1.0)
    ├── trapezoidal_dynamics_plot v1.png
    ├── ruckig_comparison.png
    └── ruckig_stats.png
```

---

## Quick Setup & Verification Checklist

Always execute setup & verification steps in the following order:

```bash
# 1. Source ROS 2 Humble & workspace setup
source /opt/ros/humble/setup.bash
source ~/kerabot_ws/install/setup.bash

# 2. Rebuild packages & compile URDF from xacro (if modifying URDF/SRDF)
cd ~/kerabot_ws
colcon build --packages-select Robot_to_URDF_New_Pakka_description kerabot_moveit_config
source install/setup.bash

# 3. Launch MoveIt 2 + RViz
ros2 launch kerabot_moveit_config demo.launch.py

# 4. Run the 6-DoF Sticker Pick, Peel & Place Benchmark Suite (in a 2nd terminal)
python3 ~/kerabot_ws/scripts/peel_place_benchmark_suite.py

# 5. Run the 6-DoF Industrial Pick & Place Simulation
python3 ~/kerabot_ws/scripts/pick_place_industrial_sim.py

# 6. Run the Motion Pipelining & Switching Stress Test
python3 ~/kerabot_ws/scripts/pipeline_stress_test.py
```

---

## Sticker Pick, Peel & Place Benchmark Suite

The flagship benchmark script [`scripts/peel_place_benchmark_suite.py`](file:///wsl.localhost/Ubuntu-22.04/home/saad/kerabot_ws/scripts/peel_place_benchmark_suite.py) tests realistic peeling kinematics across MoveIt 2 planners and dynamics scaling factors:

### Multi-Stage Peeling State Machine
- **Stage 1: Home $\rightarrow$ Pre-Pick Approach** (OMPL `RRTConnect` / Free-space hover at $Z = 0.35\text{ m}$)
- **Stage 2: Pre-Pick $\rightarrow$ Sticker Contact** (Pilz `LIN` / Straight vertical descent to vacuum contact at $Z = 0.22\text{ m}$)
- **Stage 3: Dynamic Sticker Peel Phase** (Pilz `LIN` / Angled Cartesian Retraction):
  - **Peel Angles Tested**: **15°**, **30°**, **45°**, and **60°** wrist/pitch tilt angles relative to substrate.
  - Retracts along vector $\vec{v} = (-\cos\alpha, 0, \sin\alpha)$ over $0.12\text{ m}$ with steady linear velocity to simulate clean sticker detachment without tearing.
- **Stage 4: Peel Exit $\rightarrow$ High Clearance Transfer** (Pilz `PTP` / Elevates to $Z = 0.40\text{ m}$)
- **Stage 5: Transfer $\rightarrow$ Placement Contact** (Pilz `LIN` / Straight vertical approach to placement surface at $Z = 0.22\text{ m}$ with 0° flat alignment)
- **Stage 6: Release & Return Home** (OMPL `RRTConnect` / Return sweep back to `Home`)

### Pipeline & Speed Scaling Matrix
- **Pipelines Benchmark**:
  - `Pipeline A: OMPL (RRTConnect) + Ruckig`
  - `Pipeline B: Pilz LIN + Ruckig`
  - `Pipeline C: Pilz PTP + Ruckig`
- **Speed Scaling Sweeps**: `max_velocity_scaling_factor` and `max_acceleration_scaling_factor` from $0.2 \rightarrow 1.0$ in $0.2$ increments.

### Analytics & Visual Exports (`~/kerabot_ws/results/`)
- Outputs an itemized terminal summary table listing **Plan Time (ms)**, **Exec Time (s)**, **Peak Vel**, **Peak Accel**, **Peak Jerk** ($< 5.0\text{ rad/s}^3$ limit), **Torques**, and **Collision Pass/Fail**.
- Saves comparative PNG plots:
  - `results/peel_benchmark_summary.png`: Bar charts comparing execution time, planning time, and jerk thresholds across peel angles.
  - `results/peel_benchmark_velocity.png`: Speed scaling factor curves ($0.2 \rightarrow 1.0$) overlaying Peak Velocity, Acceleration, and Jerk profiles.

---

## Motion Scripts & Test Suite

All executable scripts are housed in [`~/kerabot_ws/scripts/`](file:///wsl.localhost/Ubuntu-22.04/home/saad/kerabot_ws/scripts/):

* **`peel_place_benchmark_suite.py`**: Full 6-DoF Sticker Pick, Peel & Place Trajectory Benchmark Suite.
* **`pick_place_industrial_sim.py`**: 6-DoF 5-stage Pick & Place simulation with active EE box collision checking.
* **`pipeline_stress_test.py`**: Chained continuous waypoint execution & dynamic pipeline switching test.
* **`stress_test.py`**: 3-category joint-space sweep, rapid-fire, and limit boundary stress test.
* **`collision_stress_test.py`**: Joint sweep tool mapping self-collision boundaries.
* **`add_ground_plane.py`**: Adds/removes persistent $2\text{ m} \times 2\text{ m} \times 0.1\text{ m}$ table collision box ($Z_{\text{top}} = 0.0\text{ m}$).

---

## Ground Plane & Strict Collision Safety

### SRDF Strict Self-Collision Matrix
In `src/kerabot_moveit_config/config/Robot_to_URDF_New_Pakka.srdf`:
- **Allowed Exception**: ONLY `<disable_collisions link1="end_effector_box_link" link2="L70IE_Finger" reason="Adjacent"/>` (its immediate mounting flange).
- **Strict Active Checks**: All other structural links (`Wrist_Motor`, `J2J3_Shoulder`, `L110I_shoulder_2`, `L110I_Shoulder`, `base_link`) have **active self-collision checking** against `end_effector_box_link`.

### Robot Collision Padding
In `src/kerabot_moveit_config/config/kinematics.yaml`:
- `default_robot_padding: 0.01` ($10\text{ mm}$ safety margin around the end-effector box geometry).

### Ground Clearance Enforcement
Any trajectory attempting to drive the end-effector box or intermediate links below $Z < 0.02\text{ m}$ is rejected by MoveIt FCL and guarded in Python IK solvers.

---

## Pilz PTP / LIN + Ruckig Dynamics Architecture

```text
 Pilz PTP / LIN (Trapezoidal Cartesian & Joint Profiles)
      │
      ▼
 AddRuckigTrajectorySmoothing  ← Rounds velocity corners into jerk-limited S-curves
      │                          Enforces max_jerk limits from joint_limits.yaml
      ▼
 joint_limits.yaml             (Velocity, acceleration, and jerk caps)
      │
      ▼
 arm_controller (FollowJointTrajectory for 6 active joints)
```

---

## Troubleshooting & Verification Commands

### Check Planners Loaded in ROS 2
```bash
ros2 param get /move_group default_planning_pipeline
# Expected: pilz_industrial_motion_planner

ros2 param get /move_group planning_pipelines
# Expected: ['ompl', 'pilz_industrial_motion_planner']
```

### Validate URDF Structure
```bash
check_urdf install/Robot_to_URDF_New_Pakka_description/share/Robot_to_URDF_New_Pakka_description/urdf/Robot_to_URDF_New_Pakka.urdf
```
