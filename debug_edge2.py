import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2


def main():
    rclpy.init()
    node = Node("debug_edge2")
    cb = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=["Revolute_1", "Revolute_2", "Revolute_3", "Revolute_4", "Revolute_5"],
        base_link_name="base_link",
        end_effector_name="L70IE_Finger",
        group_name="arm",
        callback_group=cb,
    )
    moveit2.pipeline_id = "pilz_industrial_motion_planner"
    moveit2.planner_id = "PTP"
    moveit2.max_velocity = 0.5
    moveit2.max_acceleration = 0.3

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.0)

    print("--- Planning home first ---")
    t_home = moveit2.plan(joint_positions=[0.0, 0.0, 0.0, 0.0, 0.0])
    print("Home plan:", "OK" if t_home is not None else "FAIL")

    print("\n--- Planning target [0.0, -2.5, 0.0, 0.0, 0.0] ---")
    t_edge = moveit2.plan(joint_positions=[0.0, -2.5, 0.0, 0.0, 0.0])
    print("Edge 2 plan:", "OK" if t_edge is not None else "FAIL")

    print("\n--- Planning target [0.0, -2.0, 0.0, 0.0, 0.0] ---")
    t_edge2 = moveit2.plan(joint_positions=[0.0, -2.0, 0.0, 0.0, 0.0])
    print("Edge 2 (-2.0) plan:", "OK" if t_edge2 is not None else "FAIL")

    print("\n--- Planning target [0.0, -1.5, 0.0, 0.0, 0.0] ---")
    t_edge3 = moveit2.plan(joint_positions=[0.0, -1.5, 0.0, 0.0, 0.0])
    print("Edge 2 (-1.5) plan:", "OK" if t_edge3 is not None else "FAIL")

    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()
