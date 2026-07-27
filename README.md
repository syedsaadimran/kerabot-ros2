# Kerabot — 6DoF Robotic Arm Software Stack

> A complete ROS2 + MoveIt2 software stack for the **Kerabot** 6-degree-of-freedom robotic arm,
> featuring smooth trapezoidal motion planning with full 3D joint dynamics analysis (F=ma, τ=Iα).

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
5. [Smooth Trapezoidal Motion + Dynamics](#smooth-trapezoidal-motion--dynamics)
   - [What Is Trapezoidal Planning?](#what-is-trapezoidal-planning)
   - [Running the Dynamics Script](#running-the-dynamics-script)
   - [Reading the Output](#reading-the-output)
6. [Workspace Structure](#workspace-structure)
7. [Configuration Reference](#configuration-reference)
8. [Troubleshooting](#troubleshooting)
9. [Changelog](#changelog)

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
├── trapezoidal_dynamics.py        # ← Main dynamics + smooth motion script
├── move_to_pose.py                # Single pose command
├── move_sequence.py               # Chained waypoint sequence
├── plot_trajectory.py             # Plot all joint profiles from MoveIt plan
└── sweep_speeds.py                # Compare profiles at different speed scales
```

**Planning pipeline:**
```
Pilz PTP / LIN  ──►  joint_limits.yaml (vel + accel + jerk)  ──►  arm_controller
     ↑
 (trapezoidal profile guaranteed — no post-processing needed)

OMPL RRTConnect  ──►  Ruckig smoothing  ──►  arm_controller
     ↑
 (fallback for cluttered environments)
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

## Workspace Structure

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
│   │   │   ├── ompl_planning.yaml      ← Pilz PTP as default planner ★
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
├── trapezoidal_dynamics.py             ← Smooth motion + 3D dynamics ★
├── move_to_pose.py                     ← Single pose command
├── move_sequence.py                    ← Waypoint sequence
├── plot_trajectory.py                  ← Plot joint profiles
├── sweep_speeds.py                     ← Compare speed scaling
├── fix_rviz.sh                         ← RViz display fix for WSL
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

---

## Changelog

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
