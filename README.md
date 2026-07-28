# Kerabot — 6DoF Robotic Arm Software Stack

> A complete ROS2 + MoveIt2 software stack for the **Kerabot** 6-degree-of-freedom robotic arm,
> featuring jerk-limited trapezoidal motion via **Pilz PTP + Ruckig smoothing**,
> full 3D joint dynamics analysis (F=ma, τ=Iα), and an interactive validated motion planner.

---

## Table of Contents

1. [Hardware Overview](#hardware-overview)
2. [Software Architecture](#software-architecture)
3. [Fresh Linux Setup — Complete Guide](#fresh-linux-setup--complete-guide)
   - [Step 1 — Install Ubuntu 22.04](#step-1--install-ubuntu-2204)
   - [Step 2 — Install ROS2 Humble](#step-2--install-ros2-humble)
   - [Step 3 — Install MoveIt2](#step-3--install-moveit2)
   - [Step 4 — Install Pilz Industrial Motion Planner](#step-4--install-pilz-industrial-motion-planner)
   - [Step 5 — Clone This Repository](#step-5--clone-this-repository)
   - [Step 6 — Install pymoveit2](#step-6--install-pymoveit2)
   - [Step 7 — Build the Workspace](#step-7--build-the-workspace)
   - [Step 8 — Source Your Workspace](#step-8--source-your-workspace)
4. [Running the Robot](#running-the-robot)
   - [Launch MoveIt + RViz](#launch-moveit--rviz)
   - [View Robot Only (no MoveIt)](#view-robot-only-no-moveit)
   - [Move to a Pose](#move-to-a-pose)
   - [Run a Waypoint Sequence](#run-a-waypoint-sequence)
5. [Motion Planning Scripts](#motion-planning-scripts)
   - [manual\_move.py — Interactive Validated Mover](#manual_movepy--interactive-validated-mover)
   - [trapezoidal\_dynamics.py — Smooth Motion + 3D Dynamics](#trapezoidal_dynamicspy--smooth-motion--3d-dynamics)
   - [smooth\_motion.py — Pilz+Ruckig Comparison Plotter](#smooth_motionpy--pilzruckig-comparison-plotter)
   - [compare\_profiles.py — Speed/Accel Sweep](#compare_profilespy--speedaccel-sweep)
   - [sweep\_speeds.py — Quick Speed Sweep](#sweep_speedspy--quick-speed-sweep)
   - [plot\_trajectory.py — 4-Panel Profile Plot](#plot_trajectorypy--4-panel-profile-plot)
6. [Pilz PTP + Ruckig Smoothing](#pilz-ptp--ruckig-smoothing)
7. [Using RViz MotionPlanning Panel with Pilz](#using-rviz-motionplanning-panel-with-pilz)
8. [Workspace Structure](#workspace-structure)
9. [Configuration Reference](#configuration-reference)
10. [Troubleshooting](#troubleshooting)
11. [Changelog](#changelog)

---

## Hardware Overview

| Property | Value |
|---|---|
| DOF | 6 revolute joints |
| MoveIt planning group | `arm` (Revolute_1 – Revolute_5, 5 actuated) |
| End effector | `L70IE_Finger` |
| Base link | `base_link` |
| Total arm mass | ~18 kg (all links) |
| Peak torque per joint | 5.0 N·m |
| Max joint velocity | 2.79 rad/s (~160 °/s) |

Joint axes:
- **Revolute_1** (joint1) — rotation about **Z** (base yaw)
- **Revolute_2** (joint2) — rotation about **X** (shoulder pitch)
- **Revolute_3** (joint3) — rotation about **X** (elbow pitch)
- **Revolute_4** (joint4) — rotation about **Z** (forearm roll)
- **Revolute_5** (joint5) — rotation about **X** (wrist pitch)

---

## Software Architecture

```
kerabot_ws/
├── src/
│   ├── kerabot_description/       # URDF, meshes, launch for visualisation
│   ├── kerabot_moveit_config/     # MoveIt2 config (planners, limits, controllers)
│   ├── Robot_to_URDF_New_Pakka_description/
│   └── pymoveit2/                 # Python MoveIt2 wrapper (submodule)
├── trapezoidal_dynamics.py        ← Main dynamics + smooth motion script
├── manual_move.py                 ← Interactive validated mover (NEW)
├── smooth_motion.py               ← Pilz+Ruckig comparison plotter (NEW)
├── compare_profiles.py            ← Speed/accel sweep comparison
├── move_to_pose.py                ← Single pose command
├── move_sequence.py               ← Chained waypoint sequence
├── plot_trajectory.py             ← Plot all joint profiles from MoveIt plan
└── sweep_speeds.py                ← Compare profiles at different speed scales
```

**Planning pipeline (updated):**
```
 Pilz PTP / LIN
      │
      ▼
 AddRuckigTrajectorySmoothing  ←── NEW: rounds sharp ramp corners into S-curves
      │                                enforces jerk limits from joint_limits.yaml
      ▼
 joint_limits.yaml  (vel + accel + jerk caps)
      │
      ▼
 arm_controller (FollowJointTrajectory)

 OMPL RRTConnect (fallback)
      │
      ▼
 AddRuckigTrajectorySmoothing
      │
      ▼
 arm_controller
```

---

## Fresh Linux Setup — Complete Guide

> **Tested on:** Ubuntu 22.04 LTS (native or WSL2 on Windows 11)

### Step 1 — Install Ubuntu 22.04

**Native:** Download from [ubuntu.com/download](https://ubuntu.com/download/desktop)

**WSL2 (Windows users):**
```powershell
# In PowerShell (as Administrator):
wsl --install -d Ubuntu-22.04
# Restart when prompted, then open Ubuntu from Start menu
```

---

### Step 2 — Install ROS2 Humble

```bash
# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 apt repo
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble Desktop (includes RViz, rqt, demos)
sudo apt update
sudo apt install -y ros-humble-desktop

# Auto-source ROS2 on every terminal open
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify
ros2 --version   # should print: ros2 humble
```

---

### Step 3 — Install MoveIt2

```bash
sudo apt install -y ros-humble-moveit

# Verify
ros2 pkg list | grep moveit   # should list many moveit packages
```

---

### Step 4 — Install Pilz Industrial Motion Planner

```bash
sudo apt install -y ros-humble-pilz-industrial-motion-planner

# Verify
ros2 pkg list | grep pilz   # should print: pilz_industrial_motion_planner
```

---

### Step 5 — Clone This Repository

```bash
mkdir -p ~/kerabot_ws/src
cd ~/kerabot_ws

# Clone the repo (replace with your actual GitHub URL)
git clone https://github.com/YOUR_USERNAME/kerabot.git src/kerabot_description
# OR clone the whole workspace:
git clone --recurse-submodules https://github.com/YOUR_USERNAME/kerabot-ros2.git .
```

---

### Step 6 — Install pymoveit2

```bash
cd ~/kerabot_ws/src

# Clone pymoveit2 (if not already present as a submodule)
git clone https://github.com/AndrejOrsula/pymoveit2.git

# Install its Python dependencies
pip3 install scipy transforms3d
```

---

### Step 7 — Build the Workspace

```bash
cd ~/kerabot_ws

# Install all ROS dependencies declared in package.xml files
sudo apt install -y python3-rosdep
sudo rosdep init   # only needed once on a fresh system
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Check for errors — all packages should show [1.00s] or similar
```

---

### Step 8 — Source Your Workspace

```bash
# Add to ~/.bashrc so it's automatic on every new terminal
echo "source ~/kerabot_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify the packages are visible
ros2 pkg list | grep kerabot   # should show: kerabot_description, kerabot_moveit_config
```

---

## Running the Robot

> **Always open a new terminal after sourcing** to make sure all paths are loaded.

### Launch MoveIt + RViz

This is the main command. Launch this **first** before running any Python scripts.

```bash
ros2 launch kerabot_moveit_config demo.launch.py
```

RViz will open. You should see the Kerabot arm in its home position.
Use the **MotionPlanning** panel to drag the end-effector to a goal and click **Plan & Execute**.

---

### View Robot Only (no MoveIt)

For quick visualisation without the full MoveIt stack:

```bash
ros2 launch kerabot_description view_kerabot.launch.py
```

A joint_state_publisher_gui slider window will let you manually move each joint.

---

### Move to a Pose

In a **second terminal** (MoveIt must be running in the first):

```bash
cd ~/kerabot_ws
python3 move_to_pose.py
```

Edit `TARGET_POSITION` and `TARGET_QUAT_XYZW` at the top of the file to change the goal.

---

### Run a Waypoint Sequence

```bash
cd ~/kerabot_ws
python3 move_sequence.py
```

Edit the `WAYPOINTS` list to define your custom motion sequence.

---

## Smooth Trapezoidal Motion + Dynamics

### What Is Trapezoidal Planning?

Standard OMPL planners (RRTConnect) find geometrically valid paths but produce irregular velocity profiles — the robot accelerates and decelerates erratically, causing **jerk and vibration**.

The **Pilz PTP planner** solves this by guaranteeing a **trapezoidal velocity profile** for every joint:

```
Velocity
  │         ┌───────────────┐
  │        /                 \
  │       /   cruise phase    \
  │      /                     \
  │_____/                       \______
  └────────────────────────────────── Time
       ramp-up              ramp-down
```

All joints are **synchronised** — they start and finish at the same time, scaled proportionally to their travel distance. This completely eliminates jerky motion.

### Running the Dynamics Script

```bash
cd ~/kerabot_ws

# Dry run (plan + plot, no robot movement)
python3 trapezoidal_dynamics.py

# Execute on the real robot
python3 trapezoidal_dynamics.py --execute

# Use Cartesian straight-line (LIN) instead of joint-space PTP
python3 trapezoidal_dynamics.py --cartesian

# Custom speed (0.0 – 1.0 scale)
python3 trapezoidal_dynamics.py --vel 0.4 --accel 0.25

# Combine flags
python3 trapezoidal_dynamics.py --execute --vel 0.3 --accel 0.2
```

### Reading the Output

The script saves a plot to `~/kerabot_ws/trapezoidal_dynamics_plot.png` and prints a table like:

```
Joint          Peak|τ|(Nm)  OK?  Peak|F|(N)  PeakJerk  Trapezoid?
──────────────────────────────────────────────────────────────────────
Revolute_1        0.0421   ✅      0.2731    0.4120      ✅ Yes
Revolute_2        0.1839   ✅      0.9374    0.3201      ✅ Yes
Revolute_3        0.2910   ✅      0.9211    0.2500      ✅ Yes
Revolute_4        0.0123   ✅      0.1762    0.4100      ✅ Yes
Revolute_5        0.0091   ✅      0.1293    0.3900      ✅ Yes
```

**Columns explained:**
| Column | Meaning |
|---|---|
| `Peak\|τ\|` | Peak 3D torque magnitude: τ = **I**·α (N·m) |
| `OK?` | Whether torque stays under the 5.0 N·m URDF limit |
| `Peak\|F\|` | Peak 3D force at link CoM: F = m·a (N) |
| `PeakJerk` | Maximum jerk (rad/s³) — lower is smoother |
| `Trapezoid?` | Whether the velocity profile has a valid cruise phase |

**Physics used:**
- Rotational: **τ** = **I** · **α**   where **I** is the full 3×3 inertia tensor from URDF
- Translational: **F** = m · **a**_com  where **a**_com includes tangential + centripetal components

---

## Motion Planning Scripts

### manual_move.py — Interactive Validated Mover

Move the robot to any position with full 4-step validation before any motion happens.

```bash
# Cartesian pose: x y z roll(deg) pitch(deg) yaw(deg)
python3 manual_move.py --pose 0.0 -0.06 0.65 0 90 0

# Direct joint angles (radians)
python3 manual_move.py --joint 0.3 -0.5 0.4 0.0 0.2

# Return to home
python3 manual_move.py --home

# Slower speed
python3 manual_move.py --pose 0.0 -0.06 0.65 0 90 0 --vel 0.3 --accel 0.2

# Dry-run (plan only, never execute)
python3 manual_move.py --pose 0.0 -0.06 0.65 0 90 0 --dry-run

# Auto-confirm (no prompt, for automated workflows)
python3 manual_move.py --pose 0.0 -0.06 0.65 0 90 0 --yes
```

**Validation steps (in order):**
1. **Input check** — workspace bounding box, joint angle range check
2. **IK / planning dry-run** — MoveIt verifies the pose is reachable; shows waypoints + duration
3. **User confirmation** — prompts `Execute? [y/N]` before any motion
4. **Post-move verification** — reads back joint states, compares to target, reports PASS/WARN

---

### trapezoidal_dynamics.py — Smooth Motion + 3D Dynamics

Main dynamics script. Plans motion with Pilz PTP, computes 3D torques and forces.

```bash
# Dry run (plan + plot, robot doesn't move)
python3 trapezoidal_dynamics.py --home

# Execute on the robot
python3 trapezoidal_dynamics.py --home --execute

# Slower speed
python3 trapezoidal_dynamics.py --home --vel 0.3 --accel 0.2

# Cartesian straight-line instead of joint-space
python3 trapezoidal_dynamics.py --home --cartesian
```

---

### smooth_motion.py — Pilz+Ruckig Comparison Plotter

Plans the motion at multiple velocity/acceleration combos and generates detailed
velocity + jerk comparison plots showing the effect of Ruckig smoothing.

```bash
# Sweep all 5 combos (dry-run, plot only)
python3 smooth_motion.py --home

# Execute the balanced combo (vel=0.5, accel=0.3)
python3 smooth_motion.py --home --execute

# Single custom combo
python3 smooth_motion.py --home --vel 0.4 --accel 0.25
```

Output files:
- `~/kerabot_ws/ruckig_comparison.png` — velocity + jerk per joint for all combos
- `~/kerabot_ws/ruckig_stats.png` — duration/jerk/velocity bar charts + recommendation

---

### compare_profiles.py — Speed/Accel Sweep

Detailed sweep: velocity trapezoidal shape + statistics comparison.

```bash
# Ensure robot is NOT at the target first:
python3 trapezoidal_dynamics.py --home
python3 compare_profiles.py
```

Output: `profile_comparison.png`, `stats_comparison.png`

---

### sweep_speeds.py — Quick Speed Sweep

Quick comparison of 5 speed presets. Most-active joint is plotted per combo.

```bash
python3 sweep_speeds.py
```

Output: `speed_sweep_plot.png`

---

### plot_trajectory.py — 4-Panel Profile Plot

Plans to the default target, plots position / velocity / acceleration / jerk for all 5 joints.

```bash
python3 plot_trajectory.py
```

Output: `trajectory_plot.png`

---

## Pilz PTP + Ruckig Smoothing

### What is this?

| Layer | What it does |
|---|---|
| **Pilz PTP** | Generates the path geometry: trapezoidal velocity shape, synchronised joints |
| **Ruckig** | Post-processes the trajectory: rounds the sharp velocity corners into S-curves by enforcing jerk limits |
| **Result** | Jerk-limited, trapezoidal-shaped motion — no mechanical shock at ramp transitions |

### Why does it matter?

Without Ruckig, a Pilz PTP trajectory has **hard edges** at the ramp-up and ramp-down points
(the corners of the trapezoid). These cause sudden changes in acceleration, which appear as
mechanical jerk and vibration in the physical robot.

Ruckig enforces your jerk limits from `joint_limits.yaml` (e.g. `max_jerk: 10.0` for Revolute_3)
to produce smooth S-curve transitions while keeping the overall trapezoidal shape.

### Where is it configured?

`src/kerabot_moveit_config/config/ompl_planning.yaml`:
```yaml
planning_plugin: pilz_industrial_motion_planner/CommandPlanner
request_adapters: >-
    default_planner_request_adapters/AddRuckigTrajectorySmoothing  # <- this line
    default_planner_request_adapters/FixWorkspaceBounds
    ...
```

The jerk limits that Ruckig uses come from `joint_limits.yaml`.

---

## Using RViz MotionPlanning Panel with Pilz

When MoveIt launches with RViz, the MotionPlanning panel defaults to OMPL.
To use Pilz (required for smooth trapezoidal profiles via the GUI):

1. In the **MotionPlanning** panel (left side), find **Planning Library**
2. Set **Pipeline** to: `pilz_industrial_motion_planner`
3. Set **Planner** to: `PTP` (or `LIN` for straight-line)
4. Drag the interactive marker to a target pose
5. Click **Plan** then **Execute**

> **Note:** Pilz requires a clear, collision-free path. If planning fails in the GUI,
> try clicking "Update" to refresh the start state, or use `python3 manual_move.py` instead.

---

```
kerabot_ws/
│
├── src/
│   ├── kerabot_description/
│   │   ├── urdf/
│   │   │   └── kerabot.urdf.xacro      ← Robot geometry + inertia
│   │   ├── meshes/                     ← STL files for visualisation
│   │   └── launch/
│   │       └── view_kerabot.launch.py  ← RViz viewer only
│   │
│   ├── kerabot_moveit_config/
│   │   ├── config/
│   │   │   ├── ompl_planning.yaml      ← Pilz PTP + Ruckig pipeline ★
│   │   │   ├── joint_limits.yaml       ← Physics-derived jerk limits ★
│   │   │   ├── moveit_controllers.yaml
│   │   │   ├── ros2_controllers.yaml
│   │   │   ├── kinematics.yaml
│   │   │   └── Robot_to_URDF_New_Pakka.srdf
│   │   └── launch/
│   │       ├── demo.launch.py          ← Full MoveIt + RViz launch ★
│   │       └── move_group.launch.py
│   │
│   └── pymoveit2/                      ← Python MoveIt2 interface
│
├── trapezoidal_dynamics.py         ← Smooth motion + 3D dynamics ★
├── manual_move.py                  ← Interactive validated mover ★ (NEW)
├── smooth_motion.py                ← Pilz+Ruckig comparison plots ★ (NEW)
├── compare_profiles.py             ← Speed/accel sweep comparison
├── move_to_pose.py                 ← Single pose command
├── move_sequence.py                ← Waypoint sequence
├── plot_trajectory.py              ← Plot joint profiles (4 panels)
├── sweep_speeds.py                 ← Compare speed scaling presets
├── fix_rviz.sh                     ← RViz display fix for WSL
└── .gitignore
```

> Files marked ★ are the most important ones to understand first.

---

## Configuration Reference

### Changing the Target Pose

In any Python script, edit:
```python
TARGET_POSITION  = [x, y, z]          # metres, in base_link frame
TARGET_QUAT_XYZW = [qx, qy, qz, qw]  # orientation quaternion
```

Common orientations:
- `[0, 0, 0, 1]` — no rotation (identity)
- `[-0.4997, -0.5003, -0.4997, 0.5003]` — end effector pointing down

### Switching Planners

| Planner | When to use | Set in |
|---|---|---|
| Pilz PTP | Smooth point-to-point (default) | `ompl_planning.yaml` or `moveit2.planner_id = "PTP"` |
| Pilz LIN | Straight Cartesian line | `moveit2.planner_id = "LIN"` |
| OMPL RRTConnect | Around obstacles | `moveit2.planner_id = "RRTConnect"` |

### Joint Limit Tuning

Edit `src/kerabot_moveit_config/config/joint_limits.yaml`:
```yaml
default_velocity_scaling_factor: 0.5    # 0.1 = very slow, 1.0 = maximum
default_acceleration_scaling_factor: 0.3
```

Rebuild after config changes:
```bash
colcon build --symlink-install --packages-select kerabot_moveit_config
```

---

## Troubleshooting

### RViz is black / not rendering (WSL2)

```bash
bash ~/kerabot_ws/fix_rviz.sh
# Or manually:
export LIBGL_ALWAYS_SOFTWARE=1
export DISPLAY=:0
```

### "Planning failed" error

```bash
# 1. Make sure MoveIt is running first:
ros2 launch kerabot_moveit_config demo.launch.py

# 2. Check if Pilz is loaded:
ros2 node list | grep move_group
ros2 param get /move_group planning_plugin

# 3. Try increasing planning time:
moveit2.allowed_planning_time = 20.0
```

### "No module named pymoveit2"

```bash
cd ~/kerabot_ws/src
git clone https://github.com/AndrejOrsula/pymoveit2.git
cd ~/kerabot_ws && colcon build --symlink-install
source install/setup.bash
```

### Torque exceeds 5 N·m limit

Reduce velocity/acceleration scaling:
```bash
python3 trapezoidal_dynamics.py --vel 0.2 --accel 0.15
```

Or lower the limits in `joint_limits.yaml` and rebuild.

### `colcon build` fails with missing packages

```bash
cd ~/kerabot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

### Pilz planning fails in RViz GUI

- In the MotionPlanning panel, set **Pipeline = pilz_industrial_motion_planner** and **Planner = PTP**
- Click **Update** to refresh the start state before planning
- Alternatively, use `python3 manual_move.py --pose ...` which does full validation before executing

### `python3 manual_move.py` fails with "Planning failed"

```bash
# The position may be outside workspace bounds:
# WS limits: x=[-0.70, 0.70]  y=[-0.70, 0.70]  z=[0.05, 1.10]

# Try a known-good position:
python3 manual_move.py --pose 0.0 -0.06 0.65 0 90 0

# Or check if MoveIt is running:
ros2 node list | grep move_group
```

---

## Changelog

### v1.2.0 — Ruckig Smoothing + Script Audit
- **Added** Ruckig jerk-limiting to Pilz primary pipeline (`ompl_planning.yaml`)
- **Added** `manual_move.py` — 4-step validated interactive mover with workspace check, IK dry-run, confirmation, and post-move verification
- **Added** `smooth_motion.py` — Pilz+Ruckig multi-speed comparison plotter (velocity + jerk panels)
- **Fixed** `move_to_pose.py` — was using OMPL pipeline; now uses Pilz PTP
- **Fixed** `move_sequence.py` — no pipeline was set; now uses Pilz PTP
- **Fixed** `plot_trajectory.py` — OMPL → Pilz PTP; added jerk panel; dark theme
- **Fixed** `sweep_speeds.py` — OMPL randomized → Pilz PTP deterministic; added jerk panel
- **Updated** `compare_profiles.py` — titles updated for Ruckig
- **Updated** README with new scripts, Ruckig explanation, and RViz Pilz fix guide

### v1.1.0 — Trapezoidal Motion + Dynamics
- **Added** `trapezoidal_dynamics.py` — Pilz PTP planner with 3D torque (τ=Iα) and force (F=ma) analysis
- **Changed** `ompl_planning.yaml` — Pilz PTP is now the default planner; OMPL kept as named fallback
- **Changed** `joint_limits.yaml` — physics-derived jerk limits per joint; scaling raised from 0.1→0.5/0.3
- **Added** `.gitignore` for ROS2 + Python projects

### v1.0.0 — Initial Release
- Full kerabot URDF with 6 links, inertial parameters, and mesh references
- MoveIt2 config with OMPL/RRTConnect + Ruckig smoothing
- Python scripts: `move_to_pose.py`, `move_sequence.py`, `plot_trajectory.py`, `sweep_speeds.py`
- RViz config and launch files
