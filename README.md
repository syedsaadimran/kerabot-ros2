# Kerabot — 6-DoF Robotic Arm Software Stack

> **A complete ROS 2 Humble + MoveIt 2 + Gazebo software stack for the Kerabot 6-DoF industrial robotic arm.**
> Features an active rotating 6th axis (`ee_rotation_joint`), a $329 \times 267 \times 100\text{ mm}$ ($1.2\text{ kg}$) end-effector suction payload box (`end_effector_box_link`), jerk-limited **Pilz LIN / PTP + Ruckig S-curve motion profiling**, rigid ODE physics simulation, **strict adjacent-only self-collision checking**, and a high-precision **Sticker Pick, Peel & Horizontal Place Motion Pipeline**.

---

## 📑 Table of Contents

1. [⚡ Quick Session Setup (Everyday Usage)](#-1-quick-session-setup-everyday-usage)
2. [📦 From-Scratch Installation Guide (Fresh Setup / Fork)](#-2-from-scratch-installation-guide-fresh-setup--fork)
3. [🤖 Hardware & Kinematics Specifications](#-3-hardware--kinematics-specifications)
4. [🗺️ 3D Workspace Grid & Coordinate Reference Sheet](#-4-3d-workspace-grid--coordinate-reference-sheet)
5. [🚀 Flagship Motion Pipelines & Scripts](#-5-flagship-motion-pipelines--scripts)
6. [🛡️ Safety, Dynamics & Physics Architecture](#-6-safety-dynamics--physics-architecture)
7. [🔧 Troubleshooting & Verification Commands](#-7-troubleshooting--verification-commands)

---

## ⚡ 1. Quick Session Setup (Everyday Usage)

Run these commands for every new session once your workspace is set up:

### Step 1: Clean Lingering Processes (Optional Safety Step)
If Gazebo was previously closed abruptly, run this one-liner to ensure no background processes hold the simulation ports:
```bash
killall -9 gzserver gzclient robot_state_publisher rviz2 move_group 2>/dev/null || true
```

### Step 2: Terminal 1 — Launch Gazebo + MoveIt 2 + RViz
Open your first terminal window:
```bash
source /opt/ros/humble/setup.bash
source ~/kerabot_ws/install/setup.bash
ros2 launch kerabot_moveit_config gazebo_moveit.launch.py
```
> ⏳ *Wait ~5–10 seconds until Gazebo and RViz open and you see the log output:* `You can start planning now!`

---

### Step 3: Terminal 2 — Run Motion Pipelines & Scripts
Open a **new separate terminal** and source the environment:
```bash
source /opt/ros/humble/setup.bash
source ~/kerabot_ws/install/setup.bash
```

Then execute any of the following pipelines:

#### A. Run the High-Precision Sticker Pick, Peel & Place Pipeline
```bash
# 1. Offline mathematical & kinematics dry-run test (No robot launch needed):
python3 ~/kerabot_ws/scripts/test_precise_pipeline.py --dry-run --angle 30

# 2. Live execution in Gazebo (Peel at 30° at 40% speed):
python3 ~/kerabot_ws/scripts/test_precise_pipeline.py --angle 30 --speed 0.4
```

#### B. Run the User-Configurable 3D Path Designer
```bash
# Test custom 5-step waypoints offline:
python3 ~/kerabot_ws/scripts/manual_trajectory_designer.py --dry-run
```

#### C. Run the Multi-Angle Peeling Benchmark Suite
```bash
# Runs full 6-stage benchmark across 15°, 30°, 45°, 60° peel angles and exports plots:
python3 ~/kerabot_ws/scripts/peel_place_benchmark_suite.py
```

---

## 📦 2. From-Scratch Installation Guide (Fresh Setup / Fork)

Follow this complete walkthrough if you are setting up the project on a fresh machine or forking the repository.

### Prerequisites & Operating System
* **Operating System**: **Ubuntu 22.04 LTS (Jammy Jellyfish)** or **Windows 11 with WSL2 (Ubuntu 22.04)**.
* **Architecture**: x86_64 / amd64.

---

### Step 1: System Update & Essential Tools
Open a terminal and install basic development utilities:
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl gnupg2 lsb-release git build-essential \
                    python3-pip python3-colcon-common-extensions \
                    python3-rosdep python3-vcstool
```

---

### Step 2: Install ROS 2 Humble Desktop
Set up the official ROS 2 Humble repository:
```bash
# 1. Set up locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Add the ROS 2 GPG key
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 3. Add the repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 4. Install ROS 2 Humble Desktop
sudo apt update
sudo apt install -y ros-humble-desktop
```

---

### Step 3: Install Gazebo Classic & ROS 2 Control Packages
```bash
sudo apt install -y \
    gazebo \
    libgazebo-dev \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller
```

---

### Step 4: Install MoveIt 2 & Motion Planners
```bash
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-planners-ompl \
    ros-humble-pilz-industrial-motion-planner
```

---

### Step 5: Install Python Mathematical & Robotics Dependencies
```bash
pip3 install --upgrade pip
pip3 install numpy scipy matplotlib
```

---

### Step 6: Clone and Build the Kerabot Workspace
```bash
# 1. Create the workspace structure
mkdir -p ~/kerabot_ws/src
cd ~/kerabot_ws

# 2. Clone the repository into src
git clone https://github.com/syedsaadimran/kerabot-ros2.git src

# 3. Initialize and update rosdep
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 4. Build the workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 5. Source the workspace
source install/setup.bash
```

---

### Step 7: Environment Configuration (`~/.bashrc`)
Add ROS and workspace sourcing automatically to your bash session:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/kerabot_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

*(If running on WSL2 / WSLg on Windows 11, GUI applications like Gazebo and RViz2 will open automatically through Wayland / WSLg without requiring external X-servers).*

---

## 🤖 3. Hardware & Kinematics Specifications

| Property | Value |
|:---|:---|
| **Degrees of Freedom (DoF)** | 6 active revolute joints (`Revolute_1` $\dots$ `Revolute_5` + `ee_rotation_joint`) |
| **MoveIt Planning Group** | `arm` (`base_link` $\rightarrow$ `end_effector_box_link`) |
| **End-Effector Payload Box** | $329\text{ mm} \times 267\text{ mm} \times 100\text{ mm}$ ($1.2\text{ kg}$, $z=+0.050\text{ m}$ centroid offset) |
| **Collision Safety Padding** | $+15\text{ mm}$ padding ($0.359 \times 0.297 \times 0.130\text{ m}$) around end-effector box |
| **Base Mounting Link** | `base_link` ($Z = 0.00\text{ m}$ table level) |
| **Active Controllers** | `/joint_state_broadcaster` (50 Hz), `/arm_controller` (JointTrajectoryController @ 200 Hz) |

### 6-DoF Kinematic Chain
$$\text{world} \xrightarrow{\text{fixed}} \text{base\_link} \xrightarrow{\text{Revolute\_1}} \text{L110I\_Shoulder} \xrightarrow{\text{Revolute\_2}} \text{L110I\_shoulder\_2} \xrightarrow{\text{Revolute\_3}} \text{J2J3\_Shoulder} \xrightarrow{\text{Revolute\_4}} \text{Wrist\_Motor} \xrightarrow{\text{Revolute\_5}} \text{L70IE\_Finger} \xrightarrow{\text{ee\_rotation\_joint}} \text{end\_effector\_box\_link}$$

---

## 🗺️ 4. 3D Workspace Grid & Coordinate Reference Sheet

All coordinates are in **meters** relative to the origin $(0, 0, 0)$ located at the **bottom center of `base_link` on the table floor**.

```text
                                  +Z (Up / Height)
                                   ▲
                                   │
                                   │       [ Safe Transit:   Z = +0.38m to +0.45m ]
                                   │       [ Hover Zone:     Z = +0.30m to +0.35m ]
                                   │       [ Table Contact:  Z = +0.20m to +0.22m ]
                                   │       [ Ground Limit:   Z = +0.03m (Safety Guard) ]
                    (Left)         │
                     -X ◄──────────┼──────────► +X (Right)
                                  /│
                                 / │
                                /  ▼ (Table Floor: Z = 0.00m)
                 (In Front)   -Y   +Y (Behind Robot)
```

### Coordinate Ranges & Roles
* **$X$ Axis (Left / Right)**: Range $[-0.35\text{ m}, +0.35\text{ m}]$. Pick target is at $+0.25\text{ m}$ (Right), Place target is at $-0.25\text{ m}$ (Left).
* **$Y$ Axis (Front / Back)**: Range $[-0.20\text{ m}, -0.45\text{ m}]$. **Negative $Y$ is the active front workspace** where table operations occur.
* **$Z$ Axis (Elevation)**: Range $[+0.03\text{ m}, +0.55\text{ m}]$. $Z < 0.03\text{ m}$ is strictly rejected by collision guards to protect the table surface.
* **Standard Flat Horizontal Pose**: `Roll = -172.8°, Pitch = 30.4°, Yaw = -172.0°` aligns the bottom suction face completely parallel and flat against the table.
* **Angled Peeling Pitch**: Adding peel angle $\theta$ to the Pitch angle ($[-172.8^\circ, 30.4^\circ + \theta, -172.0^\circ]$) provides precise knife-edge peel alignment.

---

## 🚀 5. Flagship Motion Pipelines & Scripts

All executable scripts reside in [`~/kerabot_ws/scripts/`](file:///wsl.localhost/Ubuntu-22.04/home/saad/kerabot_ws/scripts/):

| Script | Purpose & Key Features |
| :--- | :--- |
| [`scripts/precise_peel_place_pipeline.py`](file:///wsl.localhost/Ubuntu-22.04/home/saad/kerabot_ws/scripts/precise_peel_place_pipeline.py) | **High-Precision Motion Engine**: Exact $SE(3)$ DLS Inverse Kinematics ($<0.05\text{ mm}$ position error, $<0.01^\circ$ planar tilt error), millimetric Cartesian path generator, and S-curve jerk profiling. |
| [`scripts/test_precise_pipeline.py`](file:///wsl.localhost/Ubuntu-22.04/home/saad/kerabot_ws/scripts/test_precise_pipeline.py) | **Precision Verification Suite**: Supports `--dry-run` offline kinematics validation and live MoveIt/Gazebo execution. Exports comparative charts to `results/`. |
| [`scripts/manual_trajectory_designer.py`](file:///wsl.localhost/Ubuntu-22.04/home/saad/kerabot_ws/scripts/manual_trajectory_designer.py) | **User-Configurable Path Designer**: Allows defining custom lists of 3D $(X, Y, Z)$ waypoints, Euler angles, motion types (`LIN`/`PTP`), and speeds with offline validation. |
| [`scripts/collision_aware_planner.py`](file:///wsl.localhost/Ubuntu-22.04/home/saad/kerabot_ws/scripts/collision_aware_planner.py) | **Safety Autopilot**: Validates paths via `/check_state_validity` and automatically computes OMPL detours around obstacles and self-collisions. |
| [`scripts/peel_place_benchmark_suite.py`](file:///wsl.localhost/Ubuntu-22.04/home/saad/kerabot_ws/scripts/peel_place_benchmark_suite.py) | **Multi-Angle Peeling Benchmark**: Tests peeling dynamics across $15^\circ, 30^\circ, 45^\circ, 60^\circ$ peel angles and speeds ($0.2 \to 1.0$). |

---

## 🛡️ 6. Safety, Dynamics & Physics Architecture

### 7-Step Precision State Machine
```text
[Stage 1: Pre-Pick Hover]       (Z = +0.32m, 0.0° Flat Orientation)
           │
           ▼ (Pure Vertical Linear Descent, v <= 0.03 m/s)
[Stage 2: Flat Contact]          (Z = +0.22m, Pos Error < 0.05mm, Tilt Error < 0.01°)
           │
           ▼ (Linear Peel Vector: -cos θ, 0, sin θ + Synchronized Pitch)
[Stage 3: Angled Peeling]        (@ 15°, 30°, 45°, 60° Retraction)
           │
           ▼ (High-Clearance Transfer Arc, Z = +0.38m)
[Stage 4: Safe Transit]          (Ground clearance >= 0.38m)
           │
           ▼ (Target Pre-Place Hover)
[Stage 5A: Pre-Place Hover]      (Z = +0.32m, 0.0° Flat Orientation)
           │
           ▼ (Pure Vertical Linear Descent)
[Stage 5B: Horizontal Placement] (Z = +0.22m, Planar Tilt Error = 0.0011°)
           │
           ▼ (Pure Vertical Retraction)
[Stage 6: Lift-Off & Home]       (Z = +0.32m -> Return to HOME)
```

### Strict Self-Collision Matrix (ACM)
In `src/kerabot_moveit_config/config/Robot_to_URDF_New_Pakka.srdf`:
- The only collision exception granted to `end_effector_box_link` is its immediate mounting flange (`L70IE_Finger`).
- All other arm links (`Wrist_Motor`, `J2J3_Shoulder`, `L110I_shoulder_2`, `L110I_Shoulder`, `base_link`) have **active continuous self-collision checks** against the large payload box.

### Rigid Industrial Physics & Stiff Servo Gains
* **ODE Physics Engine (`sticker_workcell.world`)**:
  * Error Reduction Parameter (`erp: 0.8`): Snaps joint constraint errors to zero (eliminating floaty/spongy joint sag).
  * Constraint Force Mixing (`cfm: 0.00000001`): Ensures rigid steel pinning.
  * Solver iterations (`iters: 100`): High-precision constraint convergence.
* **Industrial Joint PID Gains (`ros2_controllers.yaml`)**:
  * High-stiffness closed-loop position holding ($k_p = 2000 \dots 4500\text{ N}\cdot\text{m/rad}$).

---

## 🔧 7. Troubleshooting & Verification Commands

### 1. Validate URDF Structure & Kinematic Tree
```bash
check_urdf install/Robot_to_URDF_New_Pakka_description/share/Robot_to_URDF_New_Pakka_description/urdf/Robot_to_URDF_New_Pakka.urdf
```

### 2. Verify Loaded MoveIt Planners
```bash
ros2 param get /move_group planning_pipelines
# Expected output: ['ompl', 'pilz_industrial_motion_planner']
```

### 3. Kill Stale Gazebo / ROS Processes
If a simulation launch fails with `bind: Address already in use`:
```bash
killall -9 gzserver gzclient robot_state_publisher rviz2 move_group 2>/dev/null || true
```

---

## 👥 Authors & Repository
* **Repository**: [syedsaadimran/kerabot-ros2](https://github.com/syedsaadimran/kerabot-ros2.git)
* **Branch**: `main`
* **Maintainer**: Syed Saad Imran
