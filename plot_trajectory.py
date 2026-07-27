#!/usr/bin/env python3
"""
plot_trajectory.py — Plan a motion via MoveIt 2 (without executing it),
then plot ALL 5 joints' velocity and acceleration profiles.
"""

import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import matplotlib.pyplot as plt

from pymoveit2 import MoveIt2


JOINT_NAMES = [
    "Revolute_1",
    "Revolute_2",
    "Revolute_3",
    "Revolute_4",
    "Revolute_5",
]
BASE_LINK = "base_link"
END_EFFECTOR_LINK = "L70IE_Finger"
MOVE_GROUP_NAME = "arm"

TARGET_POSITION = [0.0, -0.0595, 0.45]
TARGET_QUAT_XYZW = [-0.4997, -0.5003, -0.4997, 0.5003]
CARTESIAN = False

VEL_SCALE = 1.0
ACCEL_SCALE = 1.0


def main():
    rclpy.init()
    node = Node("kerabot_trajectory_plotter")
    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name=BASE_LINK,
        end_effector_name=END_EFFECTOR_LINK,
        group_name=MOVE_GROUP_NAME,
        callback_group=callback_group,
    )
    moveit2.num_planning_attempts = 50
    moveit2.allowed_planning_time = 10.0
    moveit2.max_velocity = VEL_SCALE
    moveit2.max_acceleration = ACCEL_SCALE

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(1.0)

    node.get_logger().info(f"Planning to position={TARGET_POSITION}...")
    trajectory = moveit2.plan(
        position=TARGET_POSITION,
        quat_xyzw=TARGET_QUAT_XYZW,
        cartesian=CARTESIAN,
    )

    if trajectory is None:
        node.get_logger().error("Planning failed — nothing to plot.")
        rclpy.shutdown()
        return

    joint_names = trajectory.joint_names
    points = trajectory.points
    n_joints = len(joint_names)

    times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in points]

    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    for j in range(n_joints):
        positions = [p.positions[j] for p in points]
        velocities = [p.velocities[j] if p.velocities else 0.0 for p in points]
        accelerations = [p.accelerations[j] if p.accelerations else 0.0 for p in points]

        axes[0].plot(times, positions, label=joint_names[j])
        axes[1].plot(times, velocities, label=joint_names[j])
        axes[2].plot(times, accelerations, label=joint_names[j])

    node.get_logger().info(f"Got {len(points)} waypoints over {times[-1]:.2f}s. Plotting all joints...")

    axes[0].set_ylabel("Position (rad)")
    axes[0].set_title("Position vs Time (all joints)")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].grid(True)

    axes[1].set_ylabel("Velocity (rad/s)")
    axes[1].set_title("Velocity vs Time (all joints)")
    axes[1].legend(loc="upper right", fontsize=8)
    axes[1].grid(True)

    axes[2].set_ylabel("Acceleration (rad/s^2)")
    axes[2].set_xlabel("Time (s)")
    axes[2].set_title("Acceleration vs Time (all joints)")
    axes[2].legend(loc="upper right", fontsize=8)
    axes[2].grid(True)

    plt.tight_layout()
    plt.savefig("/home/saad/kerabot_ws/trajectory_plot.png", dpi=150)
    node.get_logger().info("Saved plot to ~/kerabot_ws/trajectory_plot.png")
    plt.show()

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
