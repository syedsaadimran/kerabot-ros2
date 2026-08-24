import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint


def main():
    rclpy.init()
    node = Node(
        'test_movegroup_client',
        parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
    )
    client = ActionClient(node, MoveGroup, '/move_action')

    print('[INFO] Connecting to /move_action (MoveGroup)...')
    if not client.wait_for_server(timeout_sec=10.0):
        print('[ERROR] MoveGroup action server not found!')
        rclpy.shutdown()
        return

    goal = MoveGroup.Goal()
    goal.request.group_name = 'arm'
    goal.request.num_planning_attempts = 10
    goal.request.allowed_planning_time = 5.0
    goal.request.max_velocity_scaling_factor = 0.5
    goal.request.max_acceleration_scaling_factor = 0.5
    goal.planning_options.plan_only = False  # True = Plan, False = Plan & Execute!

    # Target joint constraints:
    target_names = ['Revolute_1', 'Revolute_2', 'Revolute_3', 'Revolute_4', 'Revolute_5', 'ee_rotation_joint']
    target_positions = [0.2, -0.4, 0.6, 0.1, -0.3, 0.2]

    constraints = Constraints()
    for name, pos in zip(target_names, target_positions):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = pos
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)

    goal.request.goal_constraints.append(constraints)

    print(f'[INFO] Sending MoveGroup Plan & Execute goal: {target_positions}')
    send_goal_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=5.0)
    handle = send_goal_future.result()
    if not handle or not handle.accepted:
        print('[ERROR] MoveGroup goal rejected!')
        rclpy.shutdown()
        return

    print('[INFO] MoveGroup goal accepted! Planning and Executing to Gazebo...')
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=20.0)
    res = result_future.result()
    if res is not None:
        val = res.result.error_code.val
        print(f'[INFO] MoveGroup result error_code: {val} (1=SUCCESS)')
    else:
        print('[WARN] Timed out waiting for MoveGroup result.')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
