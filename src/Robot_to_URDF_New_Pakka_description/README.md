# Robot_to_URDF_New_Pakka — Robot Description (6-DoF)

## Overview

| Property | Value |
|----------|-------|
| Total mass | ~34.89 kg |
| Links | 7 |
| Joints | 6 (6 active revolute joints) |
| Root link | `base_link` |
| End-effector link | `end_effector_box_link` ($329 \times 267 \times 100\text{ mm}$) |

---

## Kinematic Tree

```text
base_link
  └─ Revolute_1 [revolute]
    L110I_Shoulder
      └─ Revolute_2 [revolute]
        L110I_shoulder_2
          └─ Revolute_3 [revolute]
            J2J3_Shoulder
              └─ Revolute_4 [revolute]
                Wrist_Motor
                  └─ Revolute_5 [revolute]
                    L70IE_Finger
                      └─ ee_rotation_joint [revolute]
                        end_effector_box_link [EE Box Payload]
```

---

## Link Properties

| Link | Mass (kg) | Material | Collision Geometry |
|------|-----------|----------|-------------------|
| `base_link` | 10.528 | Steel | cylinder |
| `L110I_Shoulder` | 6.625 | Steel | mesh |
| `L110I_shoulder_2` | 9.268 | Steel | mesh |
| `J2J3_Shoulder` | 3.447 | Steel | mesh |
| `Wrist_Motor` | 2.110 | Stainless_Steel_304 | mesh |
| `L70IE_Finger` | 1.714 | Stainless_Steel_304 | mesh / stl |
| `end_effector_box_link` | 1.200 | EE_Grey | box ($0.329 \times 0.267 \times 0.100\text{ m}$) |

---

## Joint Properties

| Joint | Type | Parent → Child | Axis | Limits (rad) |
|-------|------|---------------|------|--------------|
| `Revolute_1` | revolute | `base_link` → `L110I_Shoulder` | (0,1,0) | [-2.90, +2.90] |
| `Revolute_2` | revolute | `L110I_Shoulder` → `L110I_shoulder_2` | (0,0,1) | [-2.90, +2.90] |
| `Revolute_3` | revolute | `L110I_shoulder_2` → `J2J3_Shoulder` | (-1,0,0) | [-2.90, +2.90] |
| `Revolute_4` | revolute | `J2J3_Shoulder` → `Wrist_Motor` | (0,0,1) | [-2.90, +2.90] |
| `Revolute_5` | revolute | `Wrist_Motor` → `L70IE_Finger` | (0,0,1) | [-2.90, +2.90] |
| `ee_rotation_joint` | revolute | `L70IE_Finger` → `end_effector_box_link` | (0,0,1) | [-3.14159, +3.14159] |

---

## Quick Start (ROS 2)

```bash
# 1. Build package
cd ~/kerabot_ws
colcon build --packages-select Robot_to_URDF_New_Pakka_description
source install/setup.bash

# 2. Visualize in RViz2
ros2 launch Robot_to_URDF_New_Pakka_description display.launch.py

# 3. Validate URDF structure
check_urdf install/Robot_to_URDF_New_Pakka_description/share/Robot_to_URDF_New_Pakka_description/urdf/Robot_to_URDF_New_Pakka.urdf
```