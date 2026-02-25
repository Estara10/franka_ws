#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

def main(args=None):
    rclpy.init(args=args)
    node = Node('simple_move_test_node')
    
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

    # --- Create a simple goal for the left arm ---
    goal_msg = MoveGroup.Goal()
    goal_msg.request.group_name = 'left_arm'
    goal_msg.request.num_planning_attempts = 5
    goal_msg.request.allowed_planning_time = 5.0

    # Define the target pose
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'world'
    target_pose.pose.position.x = 0.4
    target_pose.pose.position.y = 0.2
    target_pose.pose.position.z = 0.5
    target_pose.pose.orientation.x = 1.0
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 0.0

    # Use PoseStamped for goal constraints
    from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint
    
    constraints = Constraints()
    
    # Position Constraint
    p_constraint = PositionConstraint()
    p_constraint.header.frame_id = target_pose.header.frame_id
    p_constraint.link_name = 'mj_left_hand'
    p_constraint.target_point_offset.x = 0.0
    p_constraint.target_point_offset.y = 0.0
    p_constraint.target_point_offset.z = 0.0
    
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose
    bounding_volume = SolidPrimitive()
    bounding_volume.type = SolidPrimitive.SPHERE
    bounding_volume.dimensions = [0.05] # 5cm tolerance sphere
    
    from geometry_msgs.msg import Pose as BoundingPose
    b_pose = BoundingPose()
    b_pose.position = target_pose.pose.position

    p_constraint.constraint_region.primitives.append(bounding_volume)
    p_constraint.constraint_region.primitive_poses.append(b_pose)
    p_constraint.weight = 1.0
    constraints.position_constraints.append(p_constraint)

    # Orientation Constraint
    o_constraint = OrientationConstraint()
    o_constraint.header.frame_id = target_pose.header.frame_id
    o_constraint.link_name = 'mj_left_hand'
    o_constraint.orientation = target_pose.pose.orientation
    o_constraint.absolute_x_axis_tolerance = 0.4 # Relaxed tolerance
    o_constraint.absolute_y_axis_tolerance = 0.4
    o_constraint.absolute_z_axis_tolerance = 0.4
    o_constraint.weight = 1.0
    constraints.orientation_constraints.append(o_constraint)

    goal_msg.request.goal_constraints.append(constraints)

    logger.info("Sending goal to MoveGroup...")
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
        logger.info("Movement SUCCEEDED!")
        logger.info(f"MoveIt Error Code: {result.result.error_code.val}")
    else:
        logger.error(f"Movement FAILED with status: {result.status}")
        logger.info(f"MoveIt Error Code: {result.result.error_code.val}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
