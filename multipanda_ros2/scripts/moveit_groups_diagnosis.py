#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node = Node('moveit_groups_diagnosis')
    logger = node.get_logger()
    
    logger.info("=== MoveIt Planning Groups Diagnosis ===")
    
    # Check move_group parameters to find planning groups
    logger.info("Checking /move_group parameters...")
    
    # Get parameter names from move_group node
    param_client = node.create_client(
        rclpy.parameter.get_parameter_service_type(),
        '/move_group/get_parameters'
    )
    
    if param_client.wait_for_service(timeout_sec=3.0):
        logger.info("Found /move_group parameter service")
        
        # Try to get robot_description parameter
        from rcl_interfaces.srv import GetParameters
        request = GetParameters.Request()
        request.names = [
            'robot_description_planning.group_names',
            'robot_description_planning',
            'group_names',
            'planning_plugin',
            'planning_adapters'
        ]
        
        future = param_client.call_async(request)
        
        import time
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < 3.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        if future.done():
            response = future.result()
            for i, param_name in enumerate(request.names):
                if i < len(response.values):
                    param_value = response.values[i]
                    logger.info(f"Parameter {param_name}: {param_value}")
                else:
                    logger.info(f"Parameter {param_name}: Not found")
    else:
        logger.error("Could not connect to /move_group parameter service")
    
    # List all parameters of move_group
    logger.info("\nListing all /move_group parameters...")
    
    from rcl_interfaces.srv import ListParameters
    list_client = node.create_client(ListParameters, '/move_group/list_parameters')
    
    if list_client.wait_for_service(timeout_sec=3.0):
        list_request = ListParameters.Request()
        list_future = list_client.call_async(list_request)
        
        start_time = time.time()
        while not list_future.done() and (time.time() - start_time) < 3.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        if list_future.done():
            list_response = list_future.result()
            logger.info(f"Found {len(list_response.result.names)} parameters")
            
            # Look for group-related parameters
            group_params = [name for name in list_response.result.names if 'group' in name.lower()]
            arm_params = [name for name in list_response.result.names if 'arm' in name.lower()]
            
            if group_params:
                logger.info("Group-related parameters:")
                for param in group_params[:10]:  # Show first 10
                    logger.info(f"  - {param}")
            
            if arm_params:
                logger.info("Arm-related parameters:")
                for param in arm_params[:10]:  # Show first 10
                    logger.info(f"  - {param}")
    
    # Alternative approach: check robot description
    logger.info("\nChecking robot_description...")
    
    # Subscribe to robot_description parameter
    from rcl_interfaces.msg import ParameterValue
    desc_client = node.create_client(GetParameters, '/robot_state_publisher/get_parameters')
    
    if desc_client.wait_for_service(timeout_sec=2.0):
        desc_request = GetParameters.Request()
        desc_request.names = ['robot_description']
        
        desc_future = desc_client.call_async(desc_request)
        start_time = time.time()
        while not desc_future.done() and (time.time() - start_time) < 3.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        if desc_future.done():
            desc_response = desc_future.result()
            if desc_response.values and desc_response.values[0].string_value:
                robot_desc = desc_response.values[0].string_value
                logger.info(f"Robot description length: {len(robot_desc)} characters")
                
                # Look for group definitions in URDF/SRDF
                if 'group' in robot_desc.lower():
                    logger.info("Found 'group' references in robot description")
                if 'left_arm' in robot_desc.lower():
                    logger.info("Found 'left_arm' in robot description")
                if 'right_arm' in robot_desc.lower():
                    logger.info("Found 'right_arm' in robot description")
    
    logger.info("\n=== Diagnosis Complete ===")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()