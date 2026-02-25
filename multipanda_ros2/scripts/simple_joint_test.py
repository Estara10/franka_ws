#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from action_msgs.msg import GoalStatus

def main(args=None):
    rclpy.init(args=args)
    node = Node('simple_joint_test_node')
    
    logger = node.get_logger()
    
    # Create a single action client for the move_action server
    move_client = ActionClient(node, MoveGroup, 'move_action')
    
    logger.info("Waiting for '/move_action' server...")
    if not move_client.wait_for_server(timeout_sec=5.0):
        logger.error("Action server not available! Make sure MoveIt is running.")
        node.destroy_node()
        rclpy.shutdown()
        return

    logger.info("Action server found.")

    # --- Create a simple joint space goal for the left arm ---
    goal_msg = MoveGroup.Goal()
    goal_msg.request.group_name = 'left_arm'  # Try the other planning group
    goal_msg.request.num_planning_attempts = 5
    goal_msg.request.allowed_planning_time = 5.0
    goal_msg.request.max_velocity_scaling_factor = 0.5
    goal_msg.request.max_acceleration_scaling_factor = 0.5

    # Define joint constraints (move first joint slightly)
    constraints = Constraints()
    
    # Joint constraint for the first joint of the left arm
    joint_constraint = JointConstraint()
    joint_constraint.joint_name = 'mj_left_joint1'  # Correct joint name from diagnosis
    joint_constraint.position = 0.2  # Smaller target position in radians (~11 degrees)
    joint_constraint.tolerance_above = 0.1
    joint_constraint.tolerance_below = 0.1
    joint_constraint.weight = 1.0
    
    constraints.joint_constraints.append(joint_constraint)
    goal_msg.request.goal_constraints.append(constraints)

    logger.info("Sending joint space goal to MoveGroup...")
    logger.info(f"Target: Move 'mj_left_joint1' to 0.2 radians")
    
    send_goal_future = move_client.send_goal_async(goal_msg)
    
    # Spin until the future is complete (goal is accepted or rejected)
    rclpy.spin_until_future_complete(node, send_goal_future)
    
    goal_handle = send_goal_future.result()
    if not goal_handle.accepted:
        logger.error('Goal rejected :(')
        node.destroy_node()
        rclpy.shutdown()
        return

    logger.info('Goal accepted :)')

    # Now, wait for the result of the action
    get_result_future = goal_handle.get_result_async()
    
    logger.info("Waiting for result...")
    rclpy.spin_until_future_complete(node, get_result_future)
    
    result = get_result_future.result()
    
    if result.status == GoalStatus.STATUS_SUCCEEDED:
        logger.info("Joint movement SUCCEEDED!")
        logger.info(f"MoveIt Error Code: {result.result.error_code.val}")
    else:
        logger.error(f"Joint movement FAILED with status: {result.status}")
        logger.info(f"MoveIt Error Code: {result.result.error_code.val}")
        
        # Print some additional debug info
        if hasattr(result.result, 'error_code'):
            error_code = result.result.error_code.val
            if error_code == 99999:
                logger.error("This is a PLANNING_FAILED error")
            elif error_code == -1:
                logger.error("This is a FAILURE error")
            elif error_code == -31:
                logger.error("This is a INVALID_GROUP_NAME error")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()