import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

def main():
    rclpy.init()
    node = Node('test_arm_move')
    client = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

    print('Waiting for action server...')
    if not client.wait_for_server(timeout_sec=5.0):
        print('Action server not found!')
        return

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = [
        'Revolute_1', 'Revolute_2', 'Revolute_3',
        'Revolute_4', 'Revolute_5', 'ee_rotation_joint'
    ]

    pt = JointTrajectoryPoint()
    pt.positions = [0.0, -0.5, 0.8, 0.0, -0.3, 0.0]
    pt.velocities = [0.0] * 6
    pt.time_from_start = Duration(sec=2, nanosec=0)
    goal.trajectory.points.append(pt)

    print('Sending goal...')
    send_goal_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=5.0)
    handle = send_goal_future.result()
    if not handle or not handle.accepted:
        print('Goal rejected!')
        return

    print('Goal accepted! Executing...')
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)
    res = result_future.result()
    print('Execution finished with code:', res.result.error_code)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
