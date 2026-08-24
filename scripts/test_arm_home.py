import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def main():
    rclpy.init()
    node = Node(
        'test_arm_home',
        parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
    )
    client = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

    print('[INFO] Connecting to Gazebo arm controller...')
    if not client.wait_for_server(timeout_sec=10.0):
        print('[ERROR] Action server not found!')
        rclpy.shutdown()
        return

    home_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = [
        'Revolute_1', 'Revolute_2', 'Revolute_3',
        'Revolute_4', 'Revolute_5', 'ee_rotation_joint'
    ]

    pt = JointTrajectoryPoint()
    pt.positions = home_pose
    pt.velocities = [0.0] * 6
    pt.time_from_start = Duration(sec=3, nanosec=0)
    goal.trajectory.points.append(pt)

    print(f'[INFO] Commanding HOME pose: {home_pose}')
    send_goal_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=5.0)
    handle = send_goal_future.result()
    if not handle or not handle.accepted:
        print('[ERROR] Goal rejected!')
        rclpy.shutdown()
        return

    print('[INFO] Moving arm to HOME in Gazebo...')
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=15.0)
    print('[SUCCESS] ✅ Returned to HOME pose!')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
