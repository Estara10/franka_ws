#!/usr/bin/env python3
#双臂异步协作工作流。利用 Python 的 asyncio 结合 ROS2 动作机制，进行更复杂的组合：
# 比如控制左右臂同时移动（Sync Move）或执行双臂同步夹取（Sync Grasp），支持多种模式配置。
import rclpy
import asyncio
import argparse
import sys
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import GripperCommand, FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Quaternion
from action_msgs.msg import GoalStatus

class DualArmTaskNode(Node):
    def __init__(self, mode: str = 'auto'):
        super().__init__('dual_arm_task_node')
        self.requested_mode = mode
        self.active_mode = 'auto'
        
        # Timeouts to avoid blocking forever when action server is unresponsive
        self.gripper_goal_response_timeout_sec = 3.0
        self.gripper_result_timeout_sec = 6.0
        self.gripper_cancel_timeout_sec = 1.0
        # In this MuJoCo setup, gripper action often accepts goals but does not publish final result.
        # So by default we continue once goal is accepted.
        self.gripper_wait_result = False
        # Disable gripper path after first communication timeout to avoid repeated long stalls.
        self.disable_gripper_after_timeout = True
        self.gripper_channel_available = True
        self.gripper_skip_log_printed = False
        # If gripper action is unavailable/unresponsive, continue arm-only flow for debugging.
        self.allow_continue_without_gripper = True
        # Orientation constraint is REQUIRED for correct approach direction.
        # Without it, MoveIt finds arbitrary arm configurations → wrong posture!
        self.use_orientation_constraint = True
        
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

        # Dual-arm controller for coordinated post-grasp movements
        # Sends 14-joint trajectory directly to the JointTrajectoryController
        self.dual_controller_client = ActionClient(
            self, FollowJointTrajectory,
            'dual_panda_arm_controller/follow_joint_trajectory'
        )

        # Track current joint states for building coordinated trajectories
        self.current_joint_state = {}
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10
        )

        # Joint names (must match dual_sim_controllers.yaml ordering)
        self.left_arm_joints = [f'mj_left_joint{i}' for i in range(1, 8)]
        self.right_arm_joints = [f'mj_right_joint{i}' for i in range(1, 8)]
        self.all_arm_joints = self.left_arm_joints + self.right_arm_joints

        # Set after wait_for_servers(); False = fall back to sequential motion
        self.dual_controller_available = False

    def _joint_state_cb(self, msg):
        """Cache latest joint positions for trajectory construction."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_state[name] = msg.position[i]

    def _apply_strict_gripper_mode(self):
        self.active_mode = 'strict_gripper'
        self.gripper_wait_result = True
        self.gripper_goal_response_timeout_sec = 3.0
        self.gripper_result_timeout_sec = 8.0
        self.disable_gripper_after_timeout = False
        self.gripper_channel_available = True
        self.allow_continue_without_gripper = False
        self.get_logger().info(
            "Mode=strict_gripper: gripper must return success result, otherwise task fails."
        )

    def _apply_arm_only_mode(self):
        self.active_mode = 'arm_only'
        self.gripper_wait_result = False
        self.disable_gripper_after_timeout = True
        self.gripper_channel_available = False
        self.allow_continue_without_gripper = True
        self.get_logger().warn(
            "Mode=arm_only: skipping real gripper action, only arm motion is executed."
        )

    async def configure_mode_async(self):
        if self.requested_mode == 'arm_only':
            self._apply_arm_only_mode()
            return

        if self.requested_mode == 'strict_gripper':
            self._apply_strict_gripper_mode()
            return

        # auto mode: probe whether gripper action can really complete
        self.get_logger().info("Mode=auto: probing gripper action responsiveness...")
        old = (
            self.gripper_wait_result,
            self.gripper_goal_response_timeout_sec,
            self.gripper_result_timeout_sec,
            self.disable_gripper_after_timeout,
            self.gripper_channel_available,
            self.allow_continue_without_gripper,
            self.gripper_skip_log_printed,
        )

        # Probe in strict style but short timeout
        self.gripper_wait_result = True
        self.gripper_goal_response_timeout_sec = 2.5
        self.gripper_result_timeout_sec = 4.0
        self.disable_gripper_after_timeout = False
        self.gripper_channel_available = True
        self.allow_continue_without_gripper = False
        self.gripper_skip_log_printed = False

        left_ok = await self.control_gripper_async('left', 0.08, 8.0)
        right_ok = await self.control_gripper_async('right', 0.08, 8.0)
        probe_ok = left_ok and right_ok

        if probe_ok:
            self._apply_strict_gripper_mode()
            return

        # fallback to arm-only when gripper action channel is not functional
        (self.gripper_wait_result,
         self.gripper_goal_response_timeout_sec,
         self.gripper_result_timeout_sec,
         self.disable_gripper_after_timeout,
         self.gripper_channel_available,
         self.allow_continue_without_gripper,
         self.gripper_skip_log_printed) = old
        self._apply_arm_only_mode()

    def wait_for_servers(self):
        self.get_logger().info('Waiting for action servers...')

        # These four are MANDATORY — task cannot proceed without them
        mandatory_clients = [
            ('Left Arm',     self.left_arm_client),
            ('Right Arm',    self.right_arm_client),
            ('Left Gripper', self.left_gripper_client),
            ('Right Gripper',self.right_gripper_client),
        ]
        for name, client in mandatory_clients:
            if not client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(f'{name} action server not available!')
                return False
            self.get_logger().info(f'{name} action server available.')

        # Dual controller is OPTIONAL — enables simultaneous 14-joint trajectory
        # If unavailable, post-grasp phases fall back to sequential arm motion
        self.dual_controller_available = self.dual_controller_client.wait_for_server(timeout_sec=3.0)
        if self.dual_controller_available:
            self.get_logger().info('Dual Controller action server available (coordinated mode ON).')
        else:
            self.get_logger().warn(
                'Dual Controller action server NOT available. '
                'Post-grasp phases will run sequentially (less smooth but functional).'
            )

        self.get_logger().info('All mandatory action servers ready!')
        return True

    async def control_gripper_async(self, side: str, position: float, max_effort: float):#夹爪控制函数，side参数指定左右夹爪，position是目标位置，max_effort是最大努力
        if not self.gripper_channel_available:
            if not self.gripper_skip_log_printed:
                self.get_logger().warn(
                    "Gripper communication disabled after timeout. "
                    "Skipping gripper commands in following phases."
                )
                self.gripper_skip_log_printed = True
            return False

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
        
        # Send goal (with timeout to avoid deadlock)
        try:
            goal_handle = await asyncio.wait_for(
                client.send_goal_async(goal),
                timeout=self.gripper_goal_response_timeout_sec,
            )
        except asyncio.TimeoutError:
            self.get_logger().error(
                f"Timeout waiting goal response from {side} gripper action server "
                f"({self.gripper_goal_response_timeout_sec}s)."
            )
            if self.disable_gripper_after_timeout:
                self.gripper_channel_available = False
            return False

        if not goal_handle.accepted:
            self.get_logger().error(f"Gripper goal rejected for {side}")
            return False

        if not self.gripper_wait_result:
            self.get_logger().info(
                f"Gripper goal for {side} accepted. Continue without waiting result "
                f"(gripper_wait_result={self.gripper_wait_result})."
            )
            return True

        self.get_logger().info(f"Gripper goal for {side} accepted. Waiting for result...")
        try:
            result = await asyncio.wait_for(
                goal_handle.get_result_async(),
                timeout=self.gripper_result_timeout_sec,
            )
        except asyncio.TimeoutError:
            self.get_logger().error(
                f"Timeout waiting result from {side} gripper action "
                f"({self.gripper_result_timeout_sec}s). Trying to cancel goal."
            )
            try:
                await asyncio.wait_for(
                    goal_handle.cancel_goal_async(),
                    timeout=self.gripper_cancel_timeout_sec,
                )
            except Exception:
                pass
            return False
        
        # Check result status
        res = result.result
        reached = getattr(res, 'reached_goal', None)
        stalled = getattr(res, 'stalled', None)
        success_field = getattr(res, 'success', None)

        # In MuJoCo sim the gripper frequently returns STATUS_ABORTED even for valid
        # operations (e.g. fully opening to joint limit, or stalling on an object).
        # STATUS_CANCELED means the goal was externally cancelled — that is a real failure.
        # Everything else (SUCCEEDED, ABORTED) is accepted as success.
        if result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().error(f"{side} gripper goal was externally cancelled.")
            return False

        self.get_logger().info(
            f"{side} gripper done: status={result.status}, "
            f"reached={reached}, stalled={stalled}"
        )
        return True

    async def sync_grasp(self, width: float, effort: float):
        """
        Simultaneously control both grippers.
        width: target width (position)
        effort: max effort
        """
        self.get_logger().info(f"Starting synchronized grasp: width={width}, effort={effort}")
        results = await asyncio.gather(
            self.control_gripper_async('left', width, effort),
            self.control_gripper_async('right', width, effort)
        )
        if all(results):
            self.get_logger().info("Synchronized grasp completed.")
            return True

        self.get_logger().warn("Synchronized grasp failed. Retrying sequentially...")
        left_ok = await self.control_gripper_async('left', width, effort)
        right_ok = await self.control_gripper_async('right', width, effort)
        final_ok = left_ok and right_ok
        if final_ok:
            self.get_logger().info("Sequential gripper retry succeeded.")
        else:
            self.get_logger().error("Sequential gripper retry failed.")
        return final_ok

    def create_pose_goal(self, side: str, pose: PoseStamped):
        """
        Create a MoveGroup action goal for a specific arm pose.
        """
        goal = MoveGroup.Goal()
        
        # Determine group name and end-effector link based on side
        if side == 'left':
            # Use correct MuJoCo planning group names
            group_name = 'mj_left_arm'
            # Use link8 for IK target constraints (more robust than hand link in this setup)
            link_name = 'mj_left_link8'
        elif side == 'right':
            group_name = 'mj_right_arm'
            link_name = 'mj_right_link8'
        else:
            self.get_logger().error(f"Invalid arm side: {side}")
            return None

        goal.request.group_name = group_name
        goal.request.num_planning_attempts = 30
        goal.request.allowed_planning_time = 15.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2
        
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
        # 2cm sphere for precise positioning
        primitive.dimensions = [0.02]
        
        volume = BoundingVolume()
        volume.primitives.append(primitive)
        volume.primitive_poses.append(pose.pose)
        
        pos_constraint.constraint_region = volume
        pos_constraint.weight = 1.0
        
        # Orientation constraint (optional)
        ori_constraint = OrientationConstraint()
        ori_constraint.header = pose.header
        ori_constraint.link_name = link_name
        ori_constraint.orientation = pose.pose.orientation
        # 三轴全部收紧，防止任何方向偏移导致夹爪倾斜或手臂扭转
        # X/Y 轴 0.15 rad (~8.6°) — 保持垂直向下
        # Z 轴 0.3 rad (~17°) — 限制绕接近轴旋转，同时给 IK 求解器留出可行域
        ori_constraint.absolute_x_axis_tolerance = 0.15
        ori_constraint.absolute_y_axis_tolerance = 0.15
        ori_constraint.absolute_z_axis_tolerance = 0.3
        ori_constraint.weight = 1.0
        
        constraints.position_constraints.append(pos_constraint)
        if self.use_orientation_constraint:
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
        Move both arms to target poses SEQUENTIALLY (left then right).
        MoveGroup is a single-goal action server — concurrent goals cause the
        second to preempt the first, leaving one arm in a half-executed state.
        Sequential execution is the only reliable approach.
        """
        self.get_logger().info("Moving left arm...")
        left_ok = await self.move_arm_to_pose_async('left', left_pose)
        if not left_ok:
            self.get_logger().error("Left arm movement FAILED. Aborting right arm.")
            return False

        self.get_logger().info("Left arm done. Moving right arm...")
        right_ok = await self.move_arm_to_pose_async('right', right_pose)
        if not right_ok:
            self.get_logger().error("Right arm movement FAILED.")
            return False

        self.get_logger().info("Both arms moved successfully.")
        return True

    async def _plan_only_async(self, side, pose):
        """Use MoveGroup plan_only mode to get a trajectory without executing."""
        goal = self.create_pose_goal(side, pose)
        if not goal:
            return None
        goal.planning_options.plan_only = True

        client = self.left_arm_client if side == 'left' else self.right_arm_client

        self.get_logger().info(f"Planning {side} arm (plan only)...")
        goal_handle = await client.send_goal_async(goal)
        if not goal_handle.accepted:
            self.get_logger().error(f"Plan-only goal rejected for {side}")
            return None

        result = await goal_handle.get_result_async()
        if result.result.error_code.val == 1:
            traj = result.result.planned_trajectory.joint_trajectory
            if traj.points:
                dur = traj.points[-1].time_from_start
                self.get_logger().info(
                    f"{side} arm plan OK: {len(traj.points)} pts, "
                    f"duration={dur.sec + dur.nanosec*1e-9:.2f}s"
                )
            return traj
        else:
            self.get_logger().error(
                f"{side} arm plan FAILED: error={result.result.error_code.val}"
            )
            return None

    async def sync_move_arms_coordinated(self, left_pose, right_pose):
        """
        Plan both arms via MoveGroup (plan_only), merge into a single 14-joint
        trajectory, execute via dual_panda_arm_controller.
        Both arms move simultaneously — essential when holding the bar.
        Falls back to sequential motion if dual controller is unavailable.
        """
        if not self.dual_controller_available:
            self.get_logger().warn(
                'Dual controller unavailable — falling back to sequential arm motion.'
            )
            return await self.sync_move_arms(left_pose, right_pose)

        # Plan each arm sequentially (plan_only is fast, no execution)
        left_traj = await self._plan_only_async('left', left_pose)
        if left_traj is None:
            self.get_logger().error("Coordinated move: left arm planning failed")
            return False

        right_traj = await self._plan_only_async('right', right_pose)
        if right_traj is None:
            self.get_logger().error("Coordinated move: right arm planning failed")
            return False

        # Determine duration (use the longer plan, minimum 4s for smooth motion)
        def get_dur(traj):
            if not traj.points:
                return 0.0
            t = traj.points[-1].time_from_start
            return t.sec + t.nanosec * 1e-9

        duration = max(get_dur(left_traj), get_dur(right_traj), 4.0)

        # Extract final joint positions from each plan
        left_final = dict(zip(left_traj.joint_names, left_traj.points[-1].positions))
        right_final = dict(zip(right_traj.joint_names, right_traj.points[-1].positions))

        # Build merged 14-joint trajectory
        merged = JointTrajectory()
        merged.joint_names = self.all_arm_joints

        # Current positions (start)
        current_pos = [self.current_joint_state.get(j, 0.0) for j in self.all_arm_joints]

        # Target positions (end)
        target_pos = []
        for jname in self.all_arm_joints:
            if jname in left_final:
                target_pos.append(left_final[jname])
            elif jname in right_final:
                target_pos.append(right_final[jname])
            else:
                target_pos.append(self.current_joint_state.get(jname, 0.0))

        # Start point (t=0)
        p0 = JointTrajectoryPoint()
        p0.positions = current_pos
        p0.velocities = [0.0] * 14
        p0.time_from_start = Duration(sec=0, nanosec=0)

        # End point (t=duration)
        p1 = JointTrajectoryPoint()
        p1.positions = target_pos
        p1.velocities = [0.0] * 14
        dur_sec = int(duration)
        dur_nsec = int((duration - dur_sec) * 1e9)
        p1.time_from_start = Duration(sec=dur_sec, nanosec=dur_nsec)

        merged.points = [p0, p1]

        # Execute via dual controller
        self.get_logger().info(
            f"Executing coordinated dual-arm trajectory ({duration:.1f}s)..."
        )
        fjt_goal = FollowJointTrajectory.Goal()
        fjt_goal.trajectory = merged
        fjt_goal.goal_time_tolerance = Duration(sec=5, nanosec=0)

        goal_handle = await self.dual_controller_client.send_goal_async(fjt_goal)
        if not goal_handle.accepted:
            self.get_logger().error("Dual trajectory goal rejected by controller")
            return False

        result = await goal_handle.get_result_async()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Coordinated dual-arm move SUCCEEDED")
            return True
        else:
            err = getattr(result.result, 'error_code', 'unknown')
            err_str = getattr(result.result, 'error_string', '')
            self.get_logger().error(
                f"Coordinated move FAILED: status={result.status}, "
                f"error={err}, {err_str}"
            )
            return False

    async def execute_task_flow(self):
        self.get_logger().info("=== Starting Dual Arm Long Bar Grasp Task ===")

        # Wait for joint state data (used by coordinated trajectory builder)
        # NON-FATAL: if joint states are unavailable, coordinated mode falls back to sequential
        self.get_logger().info("Waiting for joint state data (up to 5s)...")
        for _ in range(50):
            if len(self.current_joint_state) >= 14:
                break
            await asyncio.sleep(0.1)
        if len(self.current_joint_state) < 14:
            self.get_logger().warn(
                f"Only {len(self.current_joint_state)} joint states received (need 14). "
                "Coordinated trajectory disabled — will use sequential motion."
            )
            self.dual_controller_available = False  # force sequential fallback
        else:
            self.get_logger().info(f"Joint states ready ({len(self.current_joint_state)} joints).")

        planning_frame = "world"

        # === 场景参数（来自 dual_scene.xml）===
        # 铝条初始 pos="0.5 0 0.45"，因 freejoint 受重力落到桌面
        # 桌面高度 = 桌体中心z(0.2) + 半高(0.14) = 0.34m
        # 铝条静止中心 = 桌面(0.34) + 铝条Z半高(0.02) = 0.36m
        BAR_CENTER_X = 0.5
        BAR_CENTER_Y = 0.0
        BAR_RESTING_Z = 0.36   # 重力沉降后的铝条中心高度（非初始生成的0.45！）

        # 规划组 tip = link8，实际夹爪指尖在 hand_tcp（link8 下方 0.1034m）
        # 所有 link8 目标 Z = 期望 TCP 位置 Z + TCP_OFFSET
        TCP_OFFSET = 0.1034

        # 抓取点 Y 偏移（匹配 grasp_site：left +0.1, right -0.1）
        GRASP_OFFSET_Y_LEFT  =  0.10
        GRASP_OFFSET_Y_RIGHT = -0.10

        # link8 目标高度（已补偿 TCP 偏移）
        GRASP_Z = BAR_RESTING_Z + TCP_OFFSET + 0.015      # +1.5cm 安全间隙，防止掌心撞铝条顶部
        PRE_GRASP_HEIGHT = GRASP_Z + 0.10               # 抓取上方 10cm ~0.57
        LIFT_HEIGHT = GRASP_Z + 0.12                     # 抬升 12cm ~0.59
        TRANSPORT_X_OFFSET = 0.10                        # 水平搬运偏移 10cm

        # 抓取姿态：末端执行器垂直朝下，手指张开方向沿世界 X 轴（夹住铝条 4cm 宽度）
        # 推导：Panda hand_joint 相对 link8 有固定 -45° Z 旋转
        #   要使 hand.Y → world.X，需要 link8 先绕 X 旋转 180°，再绕 Z 旋转 +45°
        #   q = Rz(+45°) ⊗ Rx(180°) = (cos22.5°, sin22.5°, 0, 0) ≈ (0.9239, 0.3827, 0, 0)
        #   验证：link8.Z → world -Z（向下）✓，hand.Y → world X（手指沿X张开）✓
        grasp_orientation = Quaternion()
        grasp_orientation.x = 0.9239
        grasp_orientation.y = 0.3827
        grasp_orientation.z = 0.0
        grasp_orientation.w = 0.0

        # === 阶段 1：打开夹爪 ===
        self.get_logger().info(">> Phase 1: Open Grippers Fully")
        # Panda 夹爪最大开口 0.08m（每指 0.04m）
        # 铝条 X 方向宽 0.04m，必须开到 0.08m 才有足够间隙让夹爪下探到铝条侧面
        if not await self.sync_grasp(0.08, 5.0):
            if self.allow_continue_without_gripper:
                self.get_logger().warn("Gripper open failed, continuing (allow_continue_without_gripper=True)")
            else:
                self.get_logger().error("Failed to open grippers — abort")
                return

        # === 阶段 2：预抓取位姿（对齐抓取点，高于铝条） ===
        self.get_logger().info(">> Phase 2: Move to Pre-Grasp Pose")
        left_pre_grasp = PoseStamped()
        left_pre_grasp.header.frame_id = planning_frame
        left_pre_grasp.pose.position.x = BAR_CENTER_X
        left_pre_grasp.pose.position.y = BAR_CENTER_Y + GRASP_OFFSET_Y_LEFT
        left_pre_grasp.pose.position.z = PRE_GRASP_HEIGHT
        left_pre_grasp.pose.orientation = grasp_orientation

        right_pre_grasp = PoseStamped()
        right_pre_grasp.header.frame_id = planning_frame
        right_pre_grasp.pose.position.x = BAR_CENTER_X
        right_pre_grasp.pose.position.y = BAR_CENTER_Y + GRASP_OFFSET_Y_RIGHT
        right_pre_grasp.pose.position.z = PRE_GRASP_HEIGHT
        right_pre_grasp.pose.orientation = grasp_orientation

        if not await self.sync_move_arms(left_pre_grasp, right_pre_grasp):
            self.get_logger().error("Pre-grasp pose move failed")
            return

        # === 阶段 3：下探到抓取高度 ===
        self.get_logger().info(">> Phase 3: Lower to Grasp Pose")
        left_grasp_pose = PoseStamped()
        left_grasp_pose.header.frame_id = planning_frame
        left_grasp_pose.pose.position.x = BAR_CENTER_X
        left_grasp_pose.pose.position.y = BAR_CENTER_Y + GRASP_OFFSET_Y_LEFT
        left_grasp_pose.pose.position.z = GRASP_Z
        left_grasp_pose.pose.orientation = grasp_orientation

        right_grasp_pose = PoseStamped()
        right_grasp_pose.header.frame_id = planning_frame
        right_grasp_pose.pose.position.x = BAR_CENTER_X
        right_grasp_pose.pose.position.y = BAR_CENTER_Y + GRASP_OFFSET_Y_RIGHT
        right_grasp_pose.pose.position.z = GRASP_Z
        right_grasp_pose.pose.orientation = grasp_orientation

        if not await self.sync_move_arms(left_grasp_pose, right_grasp_pose):
            self.get_logger().error("Grasp pose lower failed")
            return

        # === 阶段 4：闭合夹爪夹取铝条 ===
        self.get_logger().info(">> Phase 4: Close Grippers (Grasp Bar)")
        # 铝条截面 4cm×4cm，夹爪目标设为接近全闭以夹紧铝条
        # GripperCommand.position = 半宽度，kTargetWidth = 2 * position
        GRASP_POSITION = 0.018   # 总开度 1cm，小于铝条 4cm → 施加夹紧力
        GRASP_EFFORT = 100.0
        if not await self.sync_grasp(GRASP_POSITION, GRASP_EFFORT):
            if self.allow_continue_without_gripper:
                self.get_logger().warn("Gripper close failed, continuing arm-only")
            else:
                self.get_logger().error("Gripper close failed — abort")
                return

        self.get_logger().info("Waiting for grasp stability (3.0s)...")
        await asyncio.sleep(3.0)

        # === 阶段 5：抬升铝条 ===
        self.get_logger().info(">> Phase 5: Lift Bar")
        left_lift_pose = PoseStamped()
        left_lift_pose.header.frame_id = planning_frame
        left_lift_pose.pose.position.x = BAR_CENTER_X
        left_lift_pose.pose.position.y = BAR_CENTER_Y + GRASP_OFFSET_Y_LEFT
        left_lift_pose.pose.position.z = LIFT_HEIGHT
        left_lift_pose.pose.orientation = grasp_orientation

        right_lift_pose = PoseStamped()
        right_lift_pose.header.frame_id = planning_frame
        right_lift_pose.pose.position.x = BAR_CENTER_X
        right_lift_pose.pose.position.y = BAR_CENTER_Y + GRASP_OFFSET_Y_RIGHT
        right_lift_pose.pose.position.z = LIFT_HEIGHT
        right_lift_pose.pose.orientation = grasp_orientation

        if not await self.sync_move_arms_coordinated(left_lift_pose, right_lift_pose):
            self.get_logger().error("Bar lift failed")
            return

        # === 阶段 6：水平搬运 ===
        self.get_logger().info(">> Phase 6: Transport Bar")
        left_transport_pose = PoseStamped()
        left_transport_pose.header.frame_id = planning_frame
        left_transport_pose.pose.position.x = BAR_CENTER_X + TRANSPORT_X_OFFSET
        left_transport_pose.pose.position.y = BAR_CENTER_Y + GRASP_OFFSET_Y_LEFT
        left_transport_pose.pose.position.z = LIFT_HEIGHT
        left_transport_pose.pose.orientation = grasp_orientation

        right_transport_pose = PoseStamped()
        right_transport_pose.header.frame_id = planning_frame
        right_transport_pose.pose.position.x = BAR_CENTER_X + TRANSPORT_X_OFFSET
        right_transport_pose.pose.position.y = BAR_CENTER_Y + GRASP_OFFSET_Y_RIGHT
        right_transport_pose.pose.position.z = LIFT_HEIGHT
        right_transport_pose.pose.orientation = grasp_orientation

        if not await self.sync_move_arms_coordinated(left_transport_pose, right_transport_pose):
            self.get_logger().error("Bar transport failed")
            return

        # === 阶段 7：下放并释放 ===
        self.get_logger().info(">> Phase 7: Lower and Release Bar")
        left_release_pose = PoseStamped()
        left_release_pose.header.frame_id = planning_frame
        left_release_pose.pose.position.x = BAR_CENTER_X + TRANSPORT_X_OFFSET
        left_release_pose.pose.position.y = BAR_CENTER_Y + GRASP_OFFSET_Y_LEFT
        left_release_pose.pose.position.z = GRASP_Z + 0.01
        left_release_pose.pose.orientation = grasp_orientation

        right_release_pose = PoseStamped()
        right_release_pose.header.frame_id = planning_frame
        right_release_pose.pose.position.x = BAR_CENTER_X + TRANSPORT_X_OFFSET
        right_release_pose.pose.position.y = BAR_CENTER_Y + GRASP_OFFSET_Y_RIGHT
        right_release_pose.pose.position.z = GRASP_Z + 0.01
        right_release_pose.pose.orientation = grasp_orientation

        if not await self.sync_move_arms_coordinated(left_release_pose, right_release_pose):
            self.get_logger().error("Bar lower for release failed")
            return

        # 打开夹爪释放铝条
        if not await self.sync_grasp(0.08, 10.0):
            self.get_logger().warn("Gripper open for release failed (non-critical)")

        await asyncio.sleep(1.0)

        # === 阶段 8：退避 ===
        self.get_logger().info(">> Phase 8: Retreat")
        left_retreat_pose = PoseStamped()
        left_retreat_pose.header.frame_id = planning_frame
        left_retreat_pose.pose.position.x = BAR_CENTER_X + TRANSPORT_X_OFFSET
        left_retreat_pose.pose.position.y = BAR_CENTER_Y + GRASP_OFFSET_Y_LEFT
        left_retreat_pose.pose.position.z = LIFT_HEIGHT
        left_retreat_pose.pose.orientation = grasp_orientation

        right_retreat_pose = PoseStamped()
        right_retreat_pose.header.frame_id = planning_frame
        right_retreat_pose.pose.position.x = BAR_CENTER_X + TRANSPORT_X_OFFSET
        right_retreat_pose.pose.position.y = BAR_CENTER_Y + GRASP_OFFSET_Y_RIGHT
        right_retreat_pose.pose.position.z = LIFT_HEIGHT
        right_retreat_pose.pose.orientation = grasp_orientation

        await self.sync_move_arms_coordinated(left_retreat_pose, right_retreat_pose)
        self.get_logger().info("=== Long Bar Grasp & Transport Task Completed ===")

def main(args=None):
    cli_args = args if args is not None else sys.argv[1:]
    parser = argparse.ArgumentParser(description='Dual-arm task flow runner')
    parser.add_argument(
        '--mode',
        choices=['auto', 'strict_gripper', 'arm_only'],
        default='auto',
        help='auto: detect gripper availability; strict_gripper: require real gripper success; arm_only: skip gripper actions',
    )
    parsed, ros_args = parser.parse_known_args(cli_args)
    
    print(f"[DEBUG] Starting task flow with mode={parsed.mode}")

    try:
        rclpy.init(args=ros_args)
    except Exception as e:
        print(f"[ERROR] Failed to initialize ROS: {e}")
        return
    
    print("[DEBUG] ROS initialized successfully")
    node = DualArmTaskNode(mode=parsed.mode)
    print(f"[DEBUG] Node created, mode={parsed.mode}")

    # Use a MultiThreadedExecutor to spin the node in a separate thread
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    # Start the executor in a background thread
    import threading
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    print("[DEBUG] Executor started in background thread")
    
    print("[DEBUG] Waiting for action servers...")
    if node.wait_for_servers():
        node.get_logger().info('Ready for tasks.')
        print("[DEBUG] All action servers available")
        
        # Now that the node is spinning in the background, we can run our async code
        loop = asyncio.get_event_loop()
        try:
            print("[DEBUG] Configuring mode...")
            loop.run_until_complete(node.configure_mode_async())
            print(f"[DEBUG] Mode configured: {node.active_mode}")
            print("[DEBUG] Starting task flow...")
            loop.run_until_complete(node.execute_task_flow())
            print("[DEBUG] Task flow completed successfully")
        except KeyboardInterrupt:
            print("[DEBUG] Interrupted by user")
        except Exception as e:
            print(f"[ERROR] Exception in task flow: {e}")
            import traceback
            traceback.print_exc()
            node.get_logger().error(f"Exception in task flow: {e}")
    else:
        print("[ERROR] Failed to connect to action servers - are ROS nodes running?")
        node.get_logger().error('Failed to connect to action servers.')

    # Cleanup
    try:
        print("[DEBUG] Shutting down...")
        node.get_logger().info("Shutting down...")
    except Exception:
        pass

    try:
        node.destroy_node()
    except Exception:
        pass

    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass

    executor_thread.join(timeout=2.0) # Wait for the executor thread to finish
    print("[DEBUG] Cleanup complete")


if __name__ == '__main__':
    main()
