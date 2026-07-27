"""
sweep_speeds.py — Plan the same target motion at several velocity/
acceleration scaling factors. Since OMPL's RRTConnect is randomized,
a different joint may do the work each time, so we detect the most
active joint independently for EACH run rather than reusing one
choice across all of them.
"""

import threading
import time
import numpy as np
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

SCALING_FACTORS_TO_TEST = [
    (1.0, 1.0),
    (0.75, 0.75),
    (0.5, 0.5),
    (0.25, 0.25),
    (0.1, 0.1),
]


def main():
    rclpy.init()
    node = Node("kerabot_speed_sweep")
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

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.0)

    fig, axes = plt.subplots(2, 1, figsize=(10, 8))

    print(f"\n{'Vel':>6} {'Accel':>6} {'Duration(s)':>12} {'PeakJerk':>10}  Active joint (per-run)")
    print("-" * 80)

    for vel_scale, accel_scale in SCALING_FACTORS_TO_TEST:
        moveit2.max_velocity = vel_scale
        moveit2.max_acceleration = accel_scale

        trajectory = moveit2.plan(
            position=TARGET_POSITION,
            quat_xyzw=TARGET_QUAT_XYZW,
            cartesian=CARTESIAN,
        )

        if trajectory is None:
            print(f"{vel_scale:>6} {accel_scale:>6}   PLANNING FAILED")
            continue

        points = trajectory.points
        n_joints = len(trajectory.joint_names)
        times = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in points])

        # Detect the most active joint for THIS specific run
        ranges = [
            max(p.positions[j] for p in points) - min(p.positions[j] for p in points)
            for j in range(n_joints)
        ]
        j = int(np.argmax(ranges))
        joint_name = trajectory.joint_names[j]

        velocities = np.array([p.velocities[j] if p.velocities else 0.0 for p in points])
        accelerations = np.array([p.accelerations[j] if p.accelerations else 0.0 for p in points])

        jerk = np.gradient(accelerations, times) if len(times) > 1 else np.array([0.0])
        peak_jerk = np.max(np.abs(jerk))
        duration = times[-1] if len(times) else 0.0

        print(f"{vel_scale:>6} {accel_scale:>6} {duration:>12.2f} {peak_jerk:>10.3f}  {joint_name}")

        label = f"vel={vel_scale}, accel={accel_scale} ({joint_name})"
        axes[0].plot(times, velocities, label=label)
        axes[1].plot(times, accelerations, label=label)

    axes[0].set_ylabel("Velocity (rad/s)")
    axes[0].set_title("Velocity vs Time (each line's most-active joint noted in legend)")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].grid(True)

    axes[1].set_ylabel("Acceleration (rad/s^2)")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_title("Acceleration vs Time")
    axes[1].legend(loc="upper right", fontsize=8)
    axes[1].grid(True)

    plt.tight_layout()
    plt.savefig("/home/saad/kerabot_ws/speed_sweep_plot.png", dpi=150)
    print("\nSaved plot to ~/kerabot_ws/speed_sweep_plot.png")
    plt.show()

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
