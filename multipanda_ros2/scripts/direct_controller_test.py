#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from action_msgs.msg import GoalStatus
import time

def main(args=None):
    rclpy.init(args=args)
    node = Node('direct_controller_test_node')
    
    logger = node.get_logger()
    
    # Create action client for the actual controller
    controller_client = ActionClient(
        node, 
        FollowJointTrajectory, 
        '/dual_panda_arm_controller/follow_joint_trajectory'
    )
    
    logger.info("Waiting for dual_panda_arm_controller...")
    if not controller_client.wait_for_server(timeout_sec=5.0):
        logger.error("Controller action server not available!")
        node.destroy_node()
        rclpy.shutdown()
        return

    logger.info("Controller found! Creating simple trajectory...")

    # Create a simple joint trajectory
    goal = FollowJointTrajectory.Goal()
    
    # Joint names from our diagnosis
    goal.trajectory.joint_names = [
        'mj_left_joint1', 'mj_left_joint2', 'mj_left_joint3', 'mj_left_joint4',
        'mj_left_joint5', 'mj_left_joint6', 'mj_left_joint7',
        'mj_right_joint1', 'mj_right_joint2', 'mj_right_joint3', 'mj_right_joint4',
        'mj_right_joint5', 'mj_right_joint6', 'mj_right_joint7'
    ]
    
    # Current positions (from diagnosis): approximately current state
    current_positions = [
        0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785,  # left arm
        0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785   # right arm
    ]
    
    # Target: just move left_joint1 slightly
    target_positions = current_positions.copy()
    target_positions[0] = 0.2  # Move mj_left_joint1 to 0.2 rad
    
    # Create trajectory point
    point = JointTrajectoryPoint()
    point.positions = target_positions
    point.velocities = [0.0] * 14  # All joints stop at target
    point.time_from_start.sec = 2  # Take 2 seconds
    point.time_from_start.nanosec = 0
    
    goal.trajectory.points.append(point)
    
    logger.info("Sending trajectory directly to controller...")
    logger.info(f"Moving mj_left_joint1 from 0.0 to 0.2 radians")
    
    # Send goal
    send_goal_future = controller_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_goal_future)
    
    goal_handle = send_goal_future.result()
    if not goal_handle.accepted:
        logger.error('Trajectory goal rejected!')
        node.destroy_node()
        rclpy.shutdown()
        return

    logger.info('Trajectory accepted! Executing...')
    
    # Wait for result
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    
    result = result_future.result()
    
    if result.status == GoalStatus.STATUS_SUCCEEDED:
        logger.info("Direct controller execution SUCCEEDED!")
        logger.info("Movement should be visible in both RViz and MuJoCo")
    else:
        logger.error(f"Direct controller execution FAILED with status: {result.status}")
        logger.error(f"Error code: {result.result.error_code}")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()