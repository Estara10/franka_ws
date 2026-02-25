#!/usr/bin/env python3
import rclpy
import asyncio
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

class DualArmTaskNode(Node):
    def __init__(self):
        super().__init__('dual_arm_task_node')
        
        # Action Clients
        # Both arms use the same MoveGroup action server "/move_action"
        # The distinction is made in the goal request (group_name="panda_1" or "panda_2")
        self.left_arm_client = ActionClient(self, MoveGroup, 'move_action')#建立机械臂与move_action的连接
        self.right_arm_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Left Gripper
        # Use the correct MuJoCo gripper action server name
        self.left_gripper_client = ActionClient(self, GripperCommand, 'mj_left_gripper_sim_node/gripper_action')
        
        # Right Gripper
        # Use the correct MuJoCo gripper action server name
        self.right_gripper_client = ActionClient(self, GripperCommand, 'mj_right_gripper_sim_node/gripper_action')

    def wait_for_servers(self):
        self.get_logger().info('Waiting for action servers...')
        
        clients = [
            ('Left Arm', self.left_arm_client),
            ('Right Arm', self.right_arm_client),
            ('Left Gripper', self.left_gripper_client),
            ('Right Gripper', self.right_gripper_client)
        ]
        
        for name, client in clients:
            if not client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(f'{name} action server not available!')
                return False
            self.get_logger().info(f'{name} action server available.')
            
        self.get_logger().info('All action servers represent!')
        return True

    async def control_gripper_async(self, side: str, position: float, max_effort: float):#夹爪控制函数，side参数指定左右夹爪，position是目标位置，max_effort是最大努力
        if side == 'left':
            client = self.left_gripper_client
        elif side == 'right':
            client = self.right_gripper_client
        else:
            self.get_logger().error(f"Invalid gripper side: {side}")
            return False

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        
        self.get_logger().info(f"Sending gripper command to {side} gripper: pos={position}, effort={max_effort}")
        
        # Send goal
        goal_handle = await client.send_goal_async(goal)
        if not goal_handle.accepted:
            self.get_logger().error(f"Gripper goal rejected for {side}")
            return False

        self.get_logger().info(f"Gripper goal for {side} accepted. Waiting for result...")
        result = await goal_handle.get_result_async()
        
        # Check result status
        res = result.result
        reached = getattr(res, 'reached_goal', None)
        stalled = getattr(res, 'stalled', None)
        success_field = getattr(res, 'success', None)

        if result.status == GoalStatus.STATUS_SUCCEEDED and (reached is True or stalled is True or success_field is True):
            self.get_logger().info(f"{side} gripper action succeeded.")
            return True
        else:
            self.get_logger().warn(f"{side} gripper action did not succeed. Status: {result.status}")
            return False

    async def sync_grasp(self, width: float, effort: float):
        """
        Simultaneously control both grippers.
        width: target width (position)
        effort: max effort
        """
        self.get_logger().info(f"Starting synchronized grasp: width={width}, effort={effort}")
        await asyncio.gather(
            self.control_gripper_async('left', width, effort),
            self.control_gripper_async('right', width, effort)
        )
        self.get_logger().info("Synchronized grasp completed.")

    def create_pose_goal(self, side: str, pose: PoseStamped):
        """
        Create a MoveGroup action goal for a specific arm pose.
        """
        goal = MoveGroup.Goal()
        
        # Determine group name and end-effector link based on side
        if side == 'left':
            # Use correct MuJoCo planning group names
            group_name = 'mj_left_arm'
            # end-effector link name in this MuJoCo setup
            link_name = 'mj_left_hand'
        elif side == 'right':
            group_name = 'mj_right_arm'
            link_name = 'mj_right_hand'
        else:
            self.get_logger().error(f"Invalid arm side: {side}")
            return None

        goal.request.group_name = group_name
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        
        # Create constraints
        constraints = Constraints()
        constraints.name = "goal_constraints"

        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header = pose.header
        pos_constraint.link_name = link_name
        
        # Define a small tolerance region (sphere)
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        # Loosen tolerance to 20mm to help planning succeed
        primitive.dimensions = [0.02]
        
        volume = BoundingVolume()
        volume.primitives.append(primitive)
        volume.primitive_poses.append(pose.pose)
        
        pos_constraint.constraint_region = volume
        pos_constraint.weight = 1.0
        
        # Orientation constraint
        ori_constraint = OrientationConstraint()
        ori_constraint.header = pose.header
        ori_constraint.link_name = link_name
        ori_constraint.orientation = pose.pose.orientation
        # Relax orientation tolerances to 0.2 rad
        ori_constraint.absolute_x_axis_tolerance = 0.2
        ori_constraint.absolute_y_axis_tolerance = 0.2
        ori_constraint.absolute_z_axis_tolerance = 0.2
        ori_constraint.weight = 1.0
        
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        
        goal.request.goal_constraints.append(constraints)
        
        return goal

    async def move_arm_to_pose_async(self, side: str, pose: PoseStamped):#机械臂移动函数，side参数指定左右机械臂，pose是目标位姿
        """
        Asynchronously move an arm to a target pose.
        """
        if side == 'left':
            client = self.left_arm_client
        elif side == 'right':
            client = self.right_arm_client
        else:
            self.get_logger().error(f"Invalid arm side: {side}")
            return False
            
        goal = self.create_pose_goal(side, pose)
        if not goal:
            return False
        
        self.get_logger().info(f"Sending move goal to {side} arm...")
        
        # Send goal
        goal_handle = await client.send_goal_async(goal)
        
        if not goal_handle.accepted:
            self.get_logger().error(f"Move goal rejected for {side} arm")
            return False
        
        self.get_logger().info(f"Goal accepted for {side} arm. Waiting for result...")
        
        # Get result
        result = await goal_handle.get_result_async()
        
        # Check result status (MoveIt error codes)
        if result.status == GoalStatus.STATUS_SUCCEEDED and result.result.error_code.val == 1:
            self.get_logger().info(f"{side} arm move SUCCEEDED")
            return True
        else:
            code = getattr(getattr(result.result, 'error_code', None), 'val', 'unknown')
            self.get_logger().error(f"{side} arm move FAILED with error code: {code} and status: {result.status}")
            return False

    async def sync_move_arms(self, left_pose: PoseStamped, right_pose: PoseStamped):
        """
        Simultaneously move both arms to target poses.
        """
        self.get_logger().info("Starting synchronized arm movement...")
        results = await asyncio.gather(
            self.move_arm_to_pose_async('left', left_pose),
            self.move_arm_to_pose_async('right', right_pose)
        )
        
        if all(results):
            self.get_logger().info("Synchronized movement COMPLETED successfully.")
            return True
        else:
            self.get_logger().warn("Synchronized movement FINISHED with errors.")
            return False

    async def execute_task_flow(self):
        """
        Execute the full dual-arm manipulation task sequence.
        """
        self.get_logger().info("=== Starting Dual Arm Task Flow ===")

        # Common orientation for gripper pointing down
        orientation = PoseStamped().pose.orientation
        orientation.x = 1.0
        orientation.y = 0.0
        orientation.z = 0.0
        orientation.w = 0.0

        # 1. Initial State: Open Grippers
        self.get_logger().info(">> Phase 1: Initializing - Opening Grippers")
        await self.sync_grasp(0.04, 10.0)
        
        # 2. Pre-grasp Pose
        self.get_logger().info(">> Phase 2: Moving to Pre-grasp Pose")
        
        # Left Arm Pre-grasp (x=0.5, y=0.1, z=0.55)
        left_pre_grasp = PoseStamped()
        left_pre_grasp.header.frame_id = "world"
        left_pre_grasp.pose.position.x = 0.5
        left_pre_grasp.pose.position.y = 0.1
        left_pre_grasp.pose.position.z = 0.55
        left_pre_grasp.pose.orientation = orientation

        # Right Arm Pre-grasp (x=0.5, y=-0.1, z=0.55)
        right_pre_grasp = PoseStamped()
        right_pre_grasp.header.frame_id = "world"
        right_pre_grasp.pose.position.x = 0.5
        right_pre_grasp.pose.position.y = -0.1
        right_pre_grasp.pose.position.z = 0.55
        right_pre_grasp.pose.orientation = orientation

        if not await self.sync_move_arms(left_pre_grasp, right_pre_grasp):
             self.get_logger().error("Failed during Pre-grasp move")
             return

        # 3. Grasp Pose (Descend)
        self.get_logger().info(">> Phase 3: Descending to Grasp Pose")
        
        # Left Arm Grasp (x=0.5, y=0.1, z=0.45)
        left_grasp_pose = PoseStamped()
        left_grasp_pose.header = left_pre_grasp.header
        left_grasp_pose.pose.position.x = 0.5
        left_grasp_pose.pose.position.y = 0.1
        left_grasp_pose.pose.position.z = 0.45
        left_grasp_pose.pose.orientation = orientation

        # Right Arm Grasp (x=0.5, y=-0.1, z=0.45)
        right_grasp_pose = PoseStamped()
        right_grasp_pose.header = right_pre_grasp.header
        right_grasp_pose.pose.position.x = 0.5
        right_grasp_pose.pose.position.y = -0.1
        right_grasp_pose.pose.position.z = 0.45
        right_grasp_pose.pose.orientation = orientation

        if not await self.sync_move_arms(left_grasp_pose, right_grasp_pose):
             self.get_logger().error("Failed during Descent move")
             return

        # 4. Close Grippers
        self.get_logger().info(">> Phase 4: Closing Grippers")
        await self.sync_grasp(0.015, 30.0)
        
        self.get_logger().info("Waiting for contact stability (2.0s)...")
        await asyncio.sleep(2.0)

        # 5. Lift Object
        self.get_logger().info(">> Phase 5: Lifting Object")
        
        # Left Arm Lift (x=0.5, y=0.1, z=0.60)
        left_lift_pose = PoseStamped()
        left_lift_pose.header = left_grasp_pose.header
        left_lift_pose.pose.position.x = 0.5
        left_lift_pose.pose.position.y = 0.1
        left_lift_pose.pose.position.z = 0.60
        left_lift_pose.pose.orientation = orientation

        # Right Arm Lift (x=0.5, y=-0.1, z=0.60)
        right_lift_pose = PoseStamped()
        right_lift_pose.header = right_grasp_pose.header
        right_lift_pose.pose.position.x = 0.5
        right_lift_pose.pose.position.y = -0.1
        right_lift_pose.pose.position.z = 0.60
        right_lift_pose.pose.orientation = orientation

        if not await self.sync_move_arms(left_lift_pose, right_lift_pose):
             self.get_logger().error("Failed during Lift move")
             return

        # 6. Transport Object (Horizontal X+0.1)
        self.get_logger().info(">> Phase 6: Transporting Object (Horizontal X+10cm)")
        
        # Left Arm Transport (x=0.6, y=0.1, z=0.60)
        left_transport_pose = PoseStamped()
        left_transport_pose.header = left_lift_pose.header
        left_transport_pose.pose.position.x = 0.6
        left_transport_pose.pose.position.y = 0.1
        left_transport_pose.pose.position.z = 0.60
        left_transport_pose.pose.orientation = orientation

        # Right Arm Transport (x=0.6, y=-0.1, z=0.60)
        right_transport_pose = PoseStamped()
        right_transport_pose.header = right_lift_pose.header
        right_transport_pose.pose.position.x = 0.6
        right_transport_pose.pose.position.y = -0.1
        right_transport_pose.pose.position.z = 0.60
        right_transport_pose.pose.orientation = orientation

        if not await self.sync_move_arms(left_transport_pose, right_transport_pose):
             self.get_logger().error("Failed during Transport move")
             return

        # 7. Descend and Release
        self.get_logger().info(">> Phase 7: Descending and Releasing")
        
        # Descend back to z=0.45
        left_release_pose = PoseStamped()
        left_release_pose.header = left_transport_pose.header
        left_release_pose.pose.position.x = 0.6
        left_release_pose.pose.position.y = 0.1
        left_release_pose.pose.position.z = 0.45
        left_release_pose.pose.orientation = orientation

        right_release_pose = PoseStamped()
        right_release_pose.header = right_transport_pose.header
        right_release_pose.pose.position.x = 0.6
        right_release_pose.pose.position.y = -0.1
        right_release_pose.pose.position.z = 0.45
        right_release_pose.pose.orientation = orientation

        if not await self.sync_move_arms(left_release_pose, right_release_pose):
             self.get_logger().error("Failed during Release Descent move")
             return

        # Open Grippers
        self.get_logger().info("Opening grippers...")
        await self.sync_grasp(0.04, 10.0)
        
        # Retreat (Optional, move up slightly to clear object)
        self.get_logger().info(">> Phase 8: Retreating (Moving Up)")
        left_retreat_pose = PoseStamped()
        left_retreat_pose.header = left_release_pose.header
        left_retreat_pose.pose.position = left_release_pose.pose.position
        left_retreat_pose.pose.position.z = 0.60
        left_retreat_pose.pose.orientation = orientation

        right_retreat_pose = PoseStamped()
        right_retreat_pose.header = right_release_pose.header
        right_retreat_pose.pose.position = right_release_pose.pose.position
        right_retreat_pose.pose.position.z = 0.60
        right_retreat_pose.pose.orientation = orientation
        
        await self.sync_move_arms(left_retreat_pose, right_retreat_pose)
        
        self.get_logger().info("=== Task Flow Completed Successfully ===")

def main(args=None):
    rclpy.init(args=args)
    node = DualArmTaskNode()

    # Use a MultiThreadedExecutor to spin the node in a separate thread
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    # Start the executor in a background thread
    import threading
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    if node.wait_for_servers():
        node.get_logger().info('Ready for tasks.')
        
        # Now that the node is spinning in the background, we can run our async code
        loop = asyncio.get_event_loop()
        try:
            loop.run_until_complete(node.execute_task_flow())
        except KeyboardInterrupt:
            node.get_logger().info('KeyboardInterrupt, shutting down.')
        except Exception as e:
            node.get_logger().error(f"Exception in task flow: {e}")
    else:
        node.get_logger().error('Failed to connect to action servers.')

    # Cleanup
    node.get_logger().info("Shutting down...")
    node.destroy_node()
    rclpy.shutdown()
    executor_thread.join() # Wait for the executor thread to finish

if __name__ == '__main__':
    main()
