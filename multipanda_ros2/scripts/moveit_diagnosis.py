#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene
from sensor_msgs.msg import JointState
import time

def main(args=None):
    rclpy.init(args=args)
    node = Node('moveit_diagnosis_node')
    logger = node.get_logger()
    
    logger.info("=== MoveIt Configuration Diagnosis ===")
    
    # Check available topics
    logger.info("\n--- Step 1: Checking ROS Topics ---")
    topic_names = node.get_topic_names_and_types()
    
    joint_states_found = False
    move_group_topics = []
    planning_scene_topics = []
    
    for topic_name, topic_types in topic_names:
        if 'joint_states' in topic_name:
            logger.info(f"Found joint states topic: {topic_name}")
            joint_states_found = True
        if 'move_group' in topic_name:
            move_group_topics.append(topic_name)
        if 'planning_scene' in topic_name:
            planning_scene_topics.append(topic_name)
    
    if move_group_topics:
        logger.info("Found MoveGroup related topics:")
        for topic in move_group_topics:
            logger.info(f"  - {topic}")
    
    if not joint_states_found:
        logger.error("No joint_states topic found! This is a critical problem.")
    
    # Try to get current joint states
    logger.info("\n--- Step 2: Reading Current Joint States ---")
    joint_state_msg = None
    
    def joint_state_callback(msg):
        nonlocal joint_state_msg
        joint_state_msg = msg
    
    # Subscribe to joint states
    subscription = node.create_subscription(
        JointState,
        '/joint_states',
        joint_state_callback,
        10
    )
    
    # Wait for a message
    logger.info("Waiting for joint states message...")
    start_time = time.time()
    timeout = 5.0
    
    while joint_state_msg is None and (time.time() - start_time) < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    if joint_state_msg:
        logger.info("Successfully received joint states!")
        logger.info(f"Available joints ({len(joint_state_msg.name)}):")
        for i, joint_name in enumerate(joint_state_msg.name):
            position = joint_state_msg.position[i] if i < len(joint_state_msg.position) else "N/A"
            logger.info(f"  - {joint_name}: {position:.4f} rad")
    else:
        logger.error("Failed to receive joint states within timeout!")
    
    # Check for planning scene service
    logger.info("\n--- Step 3: Checking Planning Scene Service ---")
    planning_scene_client = node.create_client(GetPlanningScene, '/get_planning_scene')
    
    if planning_scene_client.wait_for_service(timeout_sec=2.0):
        logger.info("Planning scene service is available")
        
        # Try to get planning scene
        request = GetPlanningScene.Request()
        request.components.components = request.components.ROBOT_STATE
        
        future = planning_scene_client.call_async(request)
        
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < 3.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        if future.done():
            response = future.result()
            if response and hasattr(response, 'scene') and hasattr(response.scene, 'robot_state'):
                robot_state = response.scene.robot_state
                logger.info(f"Robot state joint names: {robot_state.joint_state.name}")
            else:
                logger.warning("Planning scene response format unexpected")
        else:
            logger.warning("Planning scene service call timed out")
    else:
        logger.error("Planning scene service not available")
    
    # Check available services
    logger.info("\n--- Step 4: Available MoveIt Services ---")
    service_names = node.get_service_names_and_types()
    moveit_services = [name for name, _ in service_names if 'move_group' in name or 'planning' in name]
    
    if moveit_services:
        logger.info("Found MoveIt services:")
        for service in moveit_services:
            logger.info(f"  - {service}")
    else:
        logger.warning("No MoveIt services found")
    
    logger.info("\n=== Diagnosis Complete ===")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()