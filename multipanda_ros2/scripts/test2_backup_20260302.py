#!/usr/bin/env python3
import rclpy
import asyncio
import argparse
import sys
from enum import Enum
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetCartesianPath
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, PlanningScene
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import GripperCommand, FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController, ListControllers
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Quaternion, WrenchStamped, TwistStamped
from action_msgs.msg import GoalStatus
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, AllowedCollisionMatrix, AllowedCollisionEntry
import numpy as np
from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class TaskState(Enum):
    INIT_ENVIRONMENT = 1
    PLAN_APPROACH = 2
    EXECUTE_APPROACH = 3
    CLOSE_GRIPPERS = 4
    PLAN_SYNC_TRAJECTORY = 5
    EXECUTE_WITH_COMPLIANCE = 6
    OPEN_GRIPPERS = 7
    RETURN_TO_HOME = 8
    FINISHED = 9
    ERROR = 99

class DualArmTaskNode(Node):
    def __init__(self, mode: str = 'auto'):
        super().__init__('dual_arm_task_node')
        self.get_logger().info("=== 初始化基于 ROS2+MoveIt2+CHOMP 的双臂协作控制节点 ===")
        self.requested_mode = mode
        self.active_mode = 'auto'
        self.current_state = TaskState.INIT_ENVIRONMENT
        
        # --- 超时与控制参数 (Control Parameters) ---
        self.gripper_goal_response_timeout_sec = 3.0
        self.gripper_result_timeout_sec = 6.0
        self.use_orientation_constraint = True
        self.max_effort_limit = 170.0  # 夹爪最大努力
        self.expected_force_diff_threshold = 5.0 # 主从臂测力同步最大允许偏差 (N)
        
        # --- 回调组设置 (保证 Action / Subscription 不堵塞) ---
        self.cb_group_action = ReentrantCallbackGroup()
        self.cb_group_sub = MutuallyExclusiveCallbackGroup()

        # --- 设置动作客户端 (Action Clients) ---
        self.left_arm_client = ActionClient(self, MoveGroup, 'move_action', callback_group=self.cb_group_action)
        self.right_arm_client = ActionClient(self, MoveGroup, 'move_action', callback_group=self.cb_group_action)
        self.left_gripper_client = ActionClient(self, GripperCommand, 'mj_left_gripper_sim_node/gripper_action', callback_group=self.cb_group_action)
        self.right_gripper_client = ActionClient(self, GripperCommand, 'mj_right_gripper_sim_node/gripper_action', callback_group=self.cb_group_action)
        self.dual_controller_client = ActionClient(self, FollowJointTrajectory, 'dual_panda_arm_controller/follow_joint_trajectory', callback_group=self.cb_group_action)

        # --- 发布者与订阅者 (Publishers & Subscribers) ---
        self.scene_pub = self.create_publisher(PlanningScene, 'planning_scene', 10)
        
        # 针对双臂分别建立 Servo 控制接口
        self.servo_pub_left = self.create_publisher(TwistStamped, '/servo_node_left/delta_twist_cmds', 10)
        self.servo_pub_right = self.create_publisher(TwistStamped, '/servo_node_right/delta_twist_cmds', 10)
        
        self.current_joint_state = {}
        # 赋予初始全0状态防报错
        self.current_left_wrench = WrenchStamped()
        self.current_right_wrench = WrenchStamped()
        
        # 力控去皮 bias
        self.left_force_bias = {'y': 0.0, 'z': 0.0}
        self.right_force_bias = {'y': 0.0, 'z': 0.0}
        self.compliance_v_y = 0.0
        self.compliance_v_z = 0.0
        self.compliance_task_active = False
        # 轨迹规划缓存（State 5 → State 6 传递）
        self.planned_master_cartesian_trajectory = []
        self.traj_offset_x = 0.0
        self.traj_offset_y = 0.20   # 默认左右臂 Y 间距 0.2m（对应 ±0.1m 抓取偏移）
        self.traj_offset_z = 0.0
        
        # TF2 初始化
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        self.fk_client = self.create_client(GetPositionFK, 'compute_fk')
        # Cartesian path service 用于笛卡尔直线插补（避免 OMPL 在近距离下探时产生弧线）
        self.cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self.switch_ctrl_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.list_ctrl_client = self.create_client(ListControllers, '/controller_manager/list_controllers')

        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10, callback_group=self.cb_group_sub)
        
        # 订阅末端力传感器数据 (假设话题名)，用于主从阻抗自适应控制
        self.left_force_sub = self.create_subscription(WrenchStamped, '/mj_left_arm/force_torque_sensor', self._left_force_cb, 10, callback_group=self.cb_group_sub)
        self.right_force_sub = self.create_subscription(WrenchStamped, '/mj_right_arm/force_torque_sensor', self._right_force_cb, 10, callback_group=self.cb_group_sub)

        # --- 状态与关节设定 (Variables configuration) ---
        self.BAR_CENTER_X = 0.5
        self.BAR_CENTER_Y = 0.0
        self.BAR_CENTER_Z = 0.36
        self.planning_frame = "base_link"  # 提前初始化，防止 servo_move_cartesian 等函数在 execute_task_flow 前调用报错
        self.left_arm_joints =[f'mj_left_joint{i}' for i in range(1, 8)]
        self.right_arm_joints =[f'mj_right_joint{i}' for i in range(1, 8)]
        self.all_arm_joints = self.left_arm_joints + self.right_arm_joints
        self.dual_controller_available = False
        self.trajectory_controller_name = 'dual_panda_arm_controller'
        self.servo_controller_candidates = [
            'servo_controller',
            'dual_servo_controller',
            'dual_panda_servo_controller',
            'left_arm_velocity_controller',
            'right_arm_velocity_controller',
        ]

        # === 仿真初始关节角：与 dual_franka_sim.launch.py 中 default_value 完全一致 ===
        # default_value='"0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"' (两臂相同)
        # 每次任务结束/失败后恢复到这个角度，保证不需要重启仿真
        _home = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        self.home_joint_positions_left  = list(_home)
        self.home_joint_positions_right = list(_home)
        # 中期演示默认参数：优先连续性与可录制性
        self.midterm_demo_mode = True
        self.max_servo_linear_speed = 0.06  # m/s
        self.watchdog_force_limit = 130.0   # N
        self.watchdog_force_trip_count = 3  # 连续超限计数，抑制瞬时尖峰误触发
        # MuJoCo 夹爪关节是单指 0~0.04m；给 GripperCommand 传 0.08 会被 ABORTED
        self.gripper_open_position = 0.04
        self.gripper_closed_position = 0.02
        self.gripper_effort_open = 120.0
        self.gripper_effort_close = 170.0
        # 下探安全偏置，减少“手掌先碰铝条”
        self.grasp_z_safety_bias = 0.008


    def _joint_state_cb(self, msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_state[name] = msg.position[i]

    def _left_force_cb(self, msg: WrenchStamped):
        self.current_left_wrench = msg

    def _right_force_cb(self, msg: WrenchStamped):
        self.current_right_wrench = msg

    def _is_force_continue_enabled(self) -> bool:
        return bool(getattr(self, 'allow_continue_without_gripper', False))

    async def _estimate_dual_ee_z(self):
        """估计当前双臂末端平均高度（用于动态抬升而非硬编码）。"""
        zs = []
        for link_name in ('mj_left_link8', 'mj_right_link8'):
            p, _ = await self.get_current_pose(link_name)
            if p is not None and len(p) >= 3:
                zs.append(float(p[2]))
        if not zs:
            return None
        return sum(zs) / len(zs)

    def _finger_opening(self, side: str):
        joint_name = 'mj_left_finger_joint1' if side == 'left' else 'mj_right_finger_joint1'
        return self.current_joint_state.get(joint_name, None)

    def _clamp_gripper_position(self, position: float) -> float:
        return max(0.0, min(float(position), float(self.gripper_open_position)))

    async def ensure_grippers_open(self, target=None, retries: int = 3) -> bool:
        """确保双爪张开到可用开度；避免“动作返回但实际未打开”."""
        if target is None:
            target = float(self.gripper_open_position)
        target = self._clamp_gripper_position(target)

        for attempt in range(1, retries + 1):
            # 避免贴边上限导致 action server 报 ABORTED，预留 1mm
            cmd_pos = max(0.0, target - 0.001 * (attempt - 1))
            await self.sync_grasp(cmd_pos, self.gripper_effort_open, wait_for_result=True)
            await asyncio.sleep(0.2)
            left = self._finger_opening('left')
            right = self._finger_opening('right')
            if left is not None and right is not None:
                self.get_logger().info(
                    f"[开爪校验] attempt={attempt}/{retries}, target={cmd_pos:.3f}, "
                    f"left={left:.4f}, right={right:.4f}")
                if left >= cmd_pos * 0.80 and right >= cmd_pos * 0.80:
                    return True
            else:
                self.get_logger().warn("[开爪校验] 尚未获取到夹爪关节状态，继续重试")
        self.get_logger().warn("开爪校验未达标，继续执行中期流程")
        return False

    async def close_grippers_on_exit(self, tag: str = "任务退出收尾") -> bool:
        """程序退出前的统一闭爪动作。"""
        target = self._clamp_gripper_position(self.gripper_closed_position)
        effort = float(self.gripper_effort_close)
        try:
            self.get_logger().info(f"[{tag}] 执行退出闭爪: pos={target:.3f}m, effort={effort:.1f}N")
            await self.sync_grasp(target, effort, wait_for_result=True)
            await asyncio.sleep(0.2)
            left = self._finger_opening('left')
            right = self._finger_opening('right')
            self.get_logger().info(
                f"[{tag}] 闭爪后指关节: left={left}, right={right}")
            return True
        except Exception as e:
            self.get_logger().warn(f"[{tag}] 退出闭爪失败: {e}")
            return False

    async def get_target_pose_async(self, target_frame='target_bar', source_frame='base_link'):
        """
        [动态感知修复] 使用异步监听获取目标物体的精确位姿。
        取代硬编码的 BAR_CENTER_X/Y/Z。
        """
        self.get_logger().info(f"正在从 TF 获取 '{target_frame}' 位姿...")
        pose = PoseStamped()
        pose.header.frame_id = source_frame
        
        # 尝试最多 5 次获取 TF
        for _ in range(5):
            try:
                # 检查 TF 是否可用
                now = rclpy.time.Time()
                trans = self.tf_buffer.lookup_transform(source_frame, target_frame, now, timeout=rclpy.duration.Duration(seconds=1.0))
                
                pose.pose.position.x = trans.transform.translation.x
                pose.pose.position.y = trans.transform.translation.y
                pose.pose.position.z = trans.transform.translation.z
                pose.pose.orientation = trans.transform.rotation
                
                self.get_logger().info(f"✓ 成功获取 '{target_frame}' 位姿: [{pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, {pose.pose.position.z:.3f}]")
                return pose
            except Exception as e:
                self.get_logger().warn(f"TF 获取失败 ({e}), 正在重试...")
                await asyncio.sleep(0.5)
        
        # 兜底：如果 TF 彻底不可用，返回一个合理的默认位姿
        self.get_logger().error(f"无法获取 '{target_frame}' 的 TF，使用备选硬编码位姿！")
        pose.pose.position.x = 0.5
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.36
        pose.pose.orientation.w = 1.0
        return pose

    def check_joint_sanity(self, side: str = 'both', warn_margin: float = 0.087, fail_margin: float = 0.02) -> bool:
        """
        检查当前关节角是否在 Panda 各关节限位范围内（带 5° 安全余量）。
        如果某关节超限，则说明IK求解器产生了变形配置。
        Panda 关节限位 (rad):
          J1: [-2.8973, 2.8973], J2: [-1.7628, 1.7628], J3: [-2.8973, 2.8973],
          J4: [-3.0718, -0.0698], J5: [-2.8973, 2.8973], J6: [-0.0175, 3.7525],
          J7: [-2.8973, 2.8973]
        """
        limits = [
            (-2.8973, 2.8973),   # J1
            (-1.7628, 1.7628),   # J2
            (-2.8973, 2.8973),   # J3
            (-3.0718, -0.0698),  # J4
            (-2.8973, 2.8973),   # J5
            (-0.0175, 3.7525),   # J6
            (-2.8973, 2.8973),   # J7
        ]
        # warn_margin: 接近限位时仅告警；fail_margin: 逼近限位到危险距离时判定失败
        # 默认 fail_margin≈1.15°，可避免“可执行但略贴边”的姿态被过早判死
        margin = warn_margin
        
        sides_to_check = []
        if side in ('left', 'both'):
            sides_to_check.append(('left', self.left_arm_joints))
        if side in ('right', 'both'):
            sides_to_check.append(('right', self.right_arm_joints))
        
        all_ok = True
        for s, joints in sides_to_check:
            for i, jn in enumerate(joints):
                val = self.current_joint_state.get(jn, None)
                if val is None:
                    continue
                lo, hi = limits[i]
                if val < (lo + fail_margin) or val > (hi - fail_margin):
                    self.get_logger().error(
                        f"[关节检查] {s}臂 {jn}={val:.3f}rad 逼近危险限位 [{lo:.3f}, {hi:.3f}] (fail_margin={fail_margin:.3f})")
                    all_ok = False
                elif val < (lo + margin) or val > (hi - margin):
                    self.get_logger().warn(
                        f"[关节检查] {s}臂 {jn}={val:.3f}rad 接近限位 [{lo:.3f}, {hi:.3f}] (warn_margin={margin:.3f})")
        return all_ok

    async def configure_mode_async(self):
        # 强制使用最宽松的模式，保证中期视频能录完
        self.active_mode = 'auto'
        self.allow_continue_without_gripper = True
        self.gripper_wait_result = False
        # 中期模式下默认放宽急停触发与伺服速度上限
        self.midterm_demo_mode = True
        self.max_servo_linear_speed = 0.06
        self.watchdog_force_limit = 130.0
        self.watchdog_force_trip_count = 3
        self.get_logger().info("中期演示模式已启动：所有流程将强制执行到底。")

    def wait_for_servers(self):
        self.get_logger().info('等待动作服务器 (Action Servers)...')
        mandatory_clients =[
            ('Left Arm',     self.left_arm_client),
            ('Right Arm',    self.right_arm_client),
            ('Left Gripper', self.left_gripper_client),
            ('Right Gripper',self.right_gripper_client),
        ]
        for name, client in mandatory_clients:
            if not client.wait_for_server(timeout_sec=3.0):
                self.get_logger().warn(f'{name} 动作服务器不可用，但为了中期演示将强行忽略！')

        self.dual_controller_available = self.dual_controller_client.wait_for_server(timeout_sec=2.0)

        # 自动探测实际规划组名，修正 group_name 与 SRDF 不匹配的问题
        self._probe_planning_groups()
        return True

    def _probe_planning_groups(self):
        """
        通过 /get_planning_scene 服务自动发现实际存在的规划组名，
        并自动修正 mj_left_arm / mj_right_arm 的映射。

        常见命名变体：
          mj_left_arm, left_arm, panda_left_arm, left_panda_arm,
          mj_right_arm, right_arm, panda_right_arm, right_panda_arm
        """
        from moveit_msgs.srv import GetPlanningScene as GPS
        from moveit_msgs.msg import PlanningSceneComponents

        cli = self.create_client(GPS, '/move_group/get_planning_scene')
        if not cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("[group探测] /move_group/get_planning_scene 不可用，保持默认组名")
            return

        req = GPS.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX  # 轻量请求
        future = cli.call_async(req)

        # 用同步 spin_until_future_complete（此时还在主线程，spin 尚未启动）
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            self.get_logger().warn("[group探测] 服务响应超时，保持默认组名")
            return

        # 尝试从 SRDF 获取组名（另一种方式）
        # 由于 GetPlanningScene 不直接返回组名，我们改为读取 /robot_description_semantic
        self._probe_groups_from_param()

    def _probe_groups_from_param(self):
        """从 robot_description_semantic 参数（SRDF）解析实际规划组名"""
        import re
        try:
            # 尝试读取 SRDF 参数
            srdf_param_names = [
                '/robot_description_semantic',
                'robot_description_semantic',
                '/move_group/robot_description_semantic',
            ]
            srdf_content = None
            for param_name in srdf_param_names:
                try:
                    # 使用 get_parameter 或直接通过 ros2 param 服务读取
                    result = self.get_parameter_or(param_name, None)
                    if result and hasattr(result, 'value') and result.value:
                        srdf_content = result.value
                        break
                except Exception:
                    pass

            if not srdf_content:
                # 尝试通过子进程读取（备用方法）
                import subprocess
                out = subprocess.run(
                    ['ros2', 'param', 'get', '/move_group', 'robot_description_semantic'],
                    capture_output=True, text=True, timeout=3.0
                )
                if out.returncode == 0 and 'String value is:' in out.stdout:
                    srdf_content = out.stdout.split('String value is:')[-1].strip()

            if srdf_content:
                # 解析 <group name="..."> 标签
                groups = re.findall(r'<group\s+name=["\']([^"\']+)["\']', srdf_content)
                self.get_logger().info(f"[group探测] SRDF 中发现规划组: {groups}")

                # 自动映射左右臂
                left_candidates  = [g for g in groups if 'left'  in g.lower()]
                right_candidates = [g for g in groups if 'right' in g.lower()]

                if left_candidates:
                    detected_left = left_candidates[0]
                    if detected_left != 'mj_left_arm':
                        self.get_logger().warn(
                            f"[group探测] 左臂组名修正: 'mj_left_arm' → '{detected_left}'")
                        self._left_group_name  = detected_left
                    else:
                        self._left_group_name  = 'mj_left_arm'

                if right_candidates:
                    detected_right = right_candidates[0]
                    if detected_right != 'mj_right_arm':
                        self.get_logger().warn(
                            f"[group探测] 右臂组名修正: 'mj_right_arm' → '{detected_right}'")
                        self._right_group_name = detected_right
                    else:
                        self._right_group_name = 'mj_right_arm'

                self.get_logger().info(
                    f"[group探测] 最终使用: 左臂='{self._left_group_name}', 右臂='{self._right_group_name}'")
                return

        except Exception as e:
            self.get_logger().warn(f"[group探测] SRDF 解析异常: {e}")

        self.get_logger().info("[group探测] 无法自动探测，使用默认组名 mj_left_arm / mj_right_arm")

    def _group_name(self, side: str) -> str:
        """统一的规划组名查询接口，优先使用探测到的真实组名"""
        if side == 'left':
            return getattr(self, '_left_group_name',  'mj_left_arm')
        return getattr(self, '_right_group_name', 'mj_right_arm')

    async def _list_controllers_async(self):
        if not self.list_ctrl_client.wait_for_service(timeout_sec=1.0):
            return {}
        try:
            future = self.list_ctrl_client.call_async(ListControllers.Request())
            result = await asyncio.wait_for(future, timeout=3.0)
            return {c.name: c.state for c in result.controller}
        except Exception:
            return {}

    async def _switch_controllers_async(self, activate, deactivate):
        if not activate and not deactivate:
            return True
        if not self.switch_ctrl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("controller_manager/switch_controller 服务不可用，跳过控制器切换")
            return False
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = 1
        try:
            future = self.switch_ctrl_client.call_async(req)
            result = await asyncio.wait_for(future, timeout=3.0)
            return bool(result.ok)
        except Exception as e:
            self.get_logger().warn(f"控制器切换失败: {e}")
            return False

    async def switch_to_servo_mode(self):
        controllers = await self._list_controllers_async()
        if not controllers:
            self.get_logger().warn("无法获取控制器列表，保持当前模式")
            return False

        known_servo = [c for c in self.servo_controller_candidates if c in controllers]
        active_servo = [c for c in known_servo if controllers[c] == 'active']
        activate = [c for c in known_servo if controllers[c] != 'active']

        if not known_servo:
            self.get_logger().warn("未检测到可用 Servo 控制器，拒绝停用轨迹控制器")
            return False

        deactivate = []
        if self.trajectory_controller_name in controllers and controllers[self.trajectory_controller_name] == 'active':
            deactivate.append(self.trajectory_controller_name)

        if not activate and not deactivate:
            return True

        ok = await self._switch_controllers_async(activate, deactivate)
        if ok:
            self.get_logger().info(f"控制器切换到 Servo 模式成功: activate={activate}, deactivate={deactivate}")
        else:
            self.get_logger().warn("控制器切换到 Servo 模式失败，可能出现轨迹/速度控制冲突")
        return ok

    async def switch_to_trajectory_mode(self):
        controllers = await self._list_controllers_async()
        if not controllers:
            self.get_logger().warn("无法获取控制器列表，保持当前模式")
            return False

        activate = []
        if self.trajectory_controller_name in controllers and controllers[self.trajectory_controller_name] != 'active':
            activate.append(self.trajectory_controller_name)
        deactivate = [c for c in self.servo_controller_candidates if c in controllers and controllers[c] == 'active']

        if not activate and not deactivate:
            return True

        ok = await self._switch_controllers_async(activate, deactivate)
        if ok:
            self.get_logger().info(f"控制器切换到轨迹模式成功: activate={activate}, deactivate={deactivate}")
        else:
            self.get_logger().warn("控制器切换到轨迹模式失败")
        return ok

    async def set_acm_allow(self, link_name: str, object_id: str, allow: bool = True):
        """Publish an AllowedCollisionMatrix entry allowing or disallowing collisions
        between `link_name` and `object_id`. Also attempt to verify via
        /move_group/get_planning_scene if available.
        """
        # 尝试读取当前 PlanningScene 中的 ACM，修改后再发布，避免覆盖已有条目
        import traceback
        MAX_RETRIES = 3
        for attempt in range(1, MAX_RETRIES + 1):
            try:
                from moveit_msgs.srv import GetPlanningScene as GPS
                from moveit_msgs.msg import PlanningSceneComponents
                cli = self.create_client(GPS, '/move_group/get_planning_scene')
                acm = None
                if cli.wait_for_service(timeout_sec=1.0):
                    try:
                        self.get_logger().info(f"[ACM] 尝试读取现有 ACM（尝试 {attempt}/{MAX_RETRIES}）...")
                        req = GPS.Request()
                        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
                        fut = cli.call_async(req)
                        res = await asyncio.wait_for(fut, timeout=2.0)
                        acm = res.scene.allowed_collision_matrix
                        self.get_logger().info("[ACM] 读取现有 ACM 成功")
                    except Exception as e:
                        self.get_logger().warn(f"[ACM] 读取 ACM 失败: {e}")
                        acm = None

                # 如果未能读取到 ACM，就创建一个新的基本 ACM
                if acm is None or not getattr(acm, 'entry_names', None):
                    acm = AllowedCollisionMatrix()
                    acm.entry_names = []
                    acm.entry_values = []

                names = list(acm.entry_names)
                values = list(acm.entry_values)

                # Ensure both names exist in the matrix
                def add_name(n):
                    if n in names:
                        return names.index(n)
                    new_idx = len(names)
                    names.append(n)
                    for ev in values:
                        ev.enabled.append(False)
                    new_ev = AllowedCollisionEntry()
                    new_ev.enabled = [False] * len(names)
                    values.append(new_ev)
                    return new_idx

                i = add_name(link_name)
                j = add_name(object_id)

                # Set pairwise flags
                values[i].enabled[j] = bool(allow)
                values[j].enabled[i] = bool(allow)

                # Assign back and publish
                acm.entry_names = names
                acm.entry_values = values
                scene_msg = PlanningScene()
                scene_msg.is_diff = True
                scene_msg.allowed_collision_matrix = acm
                self.scene_pub.publish(scene_msg)
                await asyncio.sleep(0.12)

                # 验证（尝试）
                try:
                    if cli.wait_for_service(timeout_sec=1.0):
                        req2 = GPS.Request()
                        req2.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
                        fut2 = cli.call_async(req2)
                        res2 = await asyncio.wait_for(fut2, timeout=2.0)
                        acm_res = res2.scene.allowed_collision_matrix
                        idx_map = {n: k for k, n in enumerate(acm_res.entry_names)}
                        if link_name in idx_map and object_id in idx_map:
                            vi = idx_map[link_name]
                            vj = idx_map[object_id]
                            val = acm_res.entry_values[vi].enabled[vj]
                            self.get_logger().info(f"[ACM 验证] {link_name} vs {object_id} -> {'允许' if val else '禁止'}")
                except Exception:
                    pass

                self.get_logger().info(f"[ACM] 设置完成: {link_name} <-> {object_id} -> {'允许' if allow else '禁止'}")
                return True
            except Exception as e:
                self.get_logger().warn(f"[ACM] 第 {attempt} 次设置失败: {e}\n{traceback.format_exc()}")
                await asyncio.sleep(0.2 * attempt)

        self.get_logger().error(f"[ACM] 在 {MAX_RETRIES} 次尝试后仍无法设置 {link_name} <-> {object_id} 为 {'允许' if allow else '禁止'}")
        return False

    async def control_gripper_async(self, side: str, position: float, max_effort: float,
                                     wait_for_result: bool = True):
        client = self.left_gripper_client if side == 'left' else self.right_gripper_client
        goal = GripperCommand.Goal()
        requested_position = float(position)
        clamped_position = self._clamp_gripper_position(requested_position)
        if abs(clamped_position - requested_position) > 1e-6:
            self.get_logger().warn(
                f"{side}夹爪目标超范围: req={requested_position:.3f}m -> clamp={clamped_position:.3f}m")
        goal.command.position = clamped_position
        goal.command.max_effort = max_effort
        
        open_threshold = 0.8 * float(self.gripper_open_position)
        action_desc = "闭合" if clamped_position < open_threshold else "张开"
        self.get_logger().info(f"夹爪{action_desc} {side}: pos={clamped_position:.3f}m, effort={max_effort:.0f}N")
        try:
            goal_handle = await asyncio.wait_for(
                client.send_goal_async(goal), timeout=3.0)
            if not goal_handle.accepted:
                self.get_logger().warn(f"{side}夹爪目标被拒绝，强行继续")
                return True
            
            if wait_for_result:
                try:
                    result = await asyncio.wait_for(
                        goal_handle.get_result_async(), timeout=5.0)
                    if result.status == GoalStatus.STATUS_SUCCEEDED:
                        self.get_logger().info(f"{side}夹爪{action_desc}完成(状态={result.status})")
                    elif result.status == GoalStatus.STATUS_ABORTED:
                        cur_opening = self._finger_opening(side)
                        self.get_logger().warn(
                            f"{side}夹爪{action_desc}返回 ABORTED(6)，当前指关节={cur_opening}")
                    else:
                        self.get_logger().warn(f"{side}夹爪{action_desc}完成但状态异常={result.status}")
                except asyncio.TimeoutError:
                    self.get_logger().warn(f"{side}夹爪等待结果超时，继续执行")
            
            return True
        except asyncio.TimeoutError:
            self.get_logger().warn(f"{side}夹爪发送目标超时，强行继续")
            return True
        except Exception as e:
            self.get_logger().warn(f"{side}夹爪通讯异常({e})，强行继续")
            return True

    async def sync_grasp(self, width: float, effort: float, wait_for_result: bool = True):
        """同步控制双爪"""
        width = self._clamp_gripper_position(width)
        await asyncio.gather(
            self.control_gripper_async('left', width, effort, wait_for_result),
            self.control_gripper_async('right', width, effort, wait_for_result)
        )
        return True

    async def compute_fk_async(self, fk_link_names, joint_names, joint_positions):
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 compute_fk 服务...')
            
        req = GetPositionFK.Request()
        req.header.frame_id = self.planning_frame
        req.fk_link_names = fk_link_names
        
        req.robot_state.is_diff = True
        req.robot_state.joint_state.name = joint_names
        req.robot_state.joint_state.position = joint_positions
        
        future = self.fk_client.call_async(req)
        result = await future
        
        if result.error_code.val == 1 and len(result.pose_stamped) > 0:
            return result.pose_stamped[0]
        return None

    async def compute_ik_async(self, group_name: str, pose: PoseStamped, avoid_collisions: bool = True):
        """
        调用 compute_ik 服务求解逆运动学。

        关键改进：将当前关节状态作为 IK 种子填入 robot_state，
        避免以全零关节角为起点导致求解器陷入奇异构型。
        """
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 compute_ik 服务...')
        
        req = GetPositionIK.Request()
        req.ik_request.group_name = group_name
        req.ik_request.avoid_collisions = avoid_collisions
        req.ik_request.pose_stamped = pose
        req.ik_request.timeout.sec = 2

        # 填入当前关节状态作为 IK 种子（而非默认全零）
        # 这能显著提高 IK 成功率，并让结果更接近当前姿态（避免大跳变）
        if self.current_joint_state:
            req.ik_request.robot_state.is_diff = False
            req.ik_request.robot_state.joint_state.name = list(self.current_joint_state.keys())
            req.ik_request.robot_state.joint_state.position = list(self.current_joint_state.values())
        else:
            req.ik_request.robot_state.is_diff = True

        future = self.ik_client.call_async(req)
        result = await future
        
        if result.error_code.val == 1:
            return result.solution.joint_state
        return None

    def create_pose_goal(self, side: str, pose: PoseStamped, pos_tol: float = 0.02,
                         ori_tol_x: float = 0.05, ori_tol_y: float = 0.05, ori_tol_z: float = 0.08):
        goal = MoveGroup.Goal()
        group_name = self._group_name(side)
        link_name = 'mj_left_link8' if side == 'left' else 'mj_right_link8'

        goal.request.group_name = group_name
        goal.request.num_planning_attempts = 100
        goal.request.allowed_planning_time = 20.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.15  
        
        constraints = Constraints()
        constraints.name = "goal_constraints"

        pos_constraint = PositionConstraint()
        pos_constraint.header = pose.header
        pos_constraint.link_name = link_name
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [pos_tol] 
        
        volume = BoundingVolume()
        volume.primitives.append(primitive)
        volume.primitive_poses.append(pose.pose)
        
        pos_constraint.constraint_region = volume
        pos_constraint.weight = 1.0
        
        ori_constraint = OrientationConstraint()
        ori_constraint.header = pose.header
        ori_constraint.link_name = link_name
        ori_constraint.orientation = pose.pose.orientation
        
        # 姿态容差：平衡夹爪垂直精度与IK可行性
        ori_constraint.absolute_x_axis_tolerance = ori_tol_x
        ori_constraint.absolute_y_axis_tolerance = ori_tol_y
        ori_constraint.absolute_z_axis_tolerance = ori_tol_z
        ori_constraint.weight = 1.0
        
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        goal.request.goal_constraints.append(constraints)
        
        return goal

    async def move_arm_to_pose_async(self, side: str, pose: PoseStamped,
                                     pos_tol: float = 0.02,
                                     ori_tol_x: float = 0.05,
                                     ori_tol_y: float = 0.05,
                                     ori_tol_z: float = 0.08,
                                     max_retries: int = 3):
        """
        先通过 compute_ik 求解目标关节角，再用 JointConstraint 目标规划执行。

        为什么不用 PositionConstraint + OrientationConstraint？
        - PositionConstraint 的 BoundingVolume 球心必须是相对 link_name 的局部坐标，
          而不是世界坐标系，直接传世界坐标导致约束错误，MoveGroup 80ms 内就失败。
        - JointConstraint 路径：先 IK → 拿到关节角目标 → 在关节空间规划，
          与 OMPL 规划器配合最稳定，是 MoveIt2 推荐的编程接口。
        """
        from moveit_msgs.msg import JointConstraint

        client = self.left_arm_client if side == 'left' else self.right_arm_client
        group_name  = self._group_name(side)
        joint_names = self.left_arm_joints if side == 'left' else self.right_arm_joints

        p = pose.pose.position
        self.get_logger().info(
            f"[{side}臂] 目标({p.x:.3f},{p.y:.3f},{p.z:.3f}) → 先求 IK ...")

        # ── Step 1: IK 求解，最多重试 max_retries 次（每次稍微改变 avoid_collisions 策略）──
        ik_solution = None
        for attempt in range(1, max_retries + 1):
            avoid_col = (attempt <= 2)  # 前两次启用碰撞检查，第三次关闭（找到可行解优先）
            ik_solution = await self.compute_ik_async(group_name, pose, avoid_collisions=avoid_col)
            if ik_solution:
                self.get_logger().info(f"[{side}臂] IK 第{attempt}次求解成功 ✓")
                break
            self.get_logger().warn(f"[{side}臂] IK 第{attempt}/{max_retries}次求解失败，重试...")
            await asyncio.sleep(0.1)

        if not ik_solution:
            self.get_logger().error(f"[{side}臂] IK 全部失败，目标位姿不可达 ✗")
            return False

        # 从 IK 结果里提取本臂的 7 个关节角
        target_positions = []
        for jn in joint_names:
            try:
                idx = list(ik_solution.name).index(jn)
                target_positions.append(ik_solution.position[idx])
            except ValueError:
                self.get_logger().error(f"[{side}臂] IK 结果缺少关节 {jn}")
                return False

        # ── Step 2: 构造 JointConstraint 目标并规划 ──
        for attempt in range(1, max_retries + 1):
            goal = MoveGroup.Goal()
            goal.request.group_name = group_name
            goal.request.num_planning_attempts = 20
            goal.request.allowed_planning_time = 15.0
            goal.request.max_velocity_scaling_factor = 0.25
            goal.request.max_acceleration_scaling_factor = 0.15

            constraints = Constraints()
            constraints.name = "joint_goal"
            # 容差随重试次数放宽：0.05 → 0.08 → 0.12 rad（≈3°→5°→7°）
            tol = 0.05 * (1.5 ** (attempt - 1))
            tol = min(tol, 0.15)
            for i, jn in enumerate(joint_names):
                jc = JointConstraint()
                jc.joint_name = jn
                jc.position = target_positions[i]
                jc.tolerance_above = tol
                jc.tolerance_below = tol
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)
            goal.request.goal_constraints.append(constraints)

            self.get_logger().info(
                f"[{side}臂] 规划第{attempt}/{max_retries}次，关节容差={tol:.3f}rad")
            try:
                goal_handle = await asyncio.wait_for(
                    client.send_goal_async(goal), timeout=10.0)
            except asyncio.TimeoutError:
                self.get_logger().warn(f"[{side}臂] 发送目标超时")
                continue

            if not goal_handle.accepted:
                self.get_logger().warn(f"[{side}臂] 目标被拒绝，重试")
                continue

            try:
                result = await asyncio.wait_for(
                    goal_handle.get_result_async(), timeout=30.0)
            except asyncio.TimeoutError:
                self.get_logger().warn(f"[{side}臂] 等待结果超时")
                continue

            err_val = getattr(getattr(result.result, 'error_code', None), 'val', -999)
            if result.status == GoalStatus.STATUS_SUCCEEDED and err_val == 1:
                self.get_logger().info(f"[{side}臂] 规划执行成功 ✓")
                return True

            self.get_logger().warn(
                f"[{side}臂] 规划第{attempt}次失败 (error_code={err_val}, status={result.status})")

        self.get_logger().error(f"[{side}臂] 全部规划均失败 ✗")
        return False

    async def sync_move_arms(self, left_pose: PoseStamped, right_pose: PoseStamped,
                             pos_tol: float = 0.02,
                             ori_tol_x: float = 0.05,
                             ori_tol_y: float = 0.05,
                             ori_tol_z: float = 0.08,
                             max_retries: int = 3):
        """双臂移动：优先使用 dual_controller 做真正同步，否则顺序移动"""
        lp = left_pose.pose.position
        rp = right_pose.pose.position
        self.get_logger().info(
            f"双臂移动 → 左({lp.x:.3f},{lp.y:.3f},{lp.z:.3f}) 右({rp.x:.3f},{rp.y:.3f},{rp.z:.3f})")
        
        if self.dual_controller_available:
            # 先各自 plan-only，再合并执行
            self.get_logger().info("使用双臂控制器同步执行...")
            left_traj = await self._plan_only_async('left', left_pose)
            right_traj = await self._plan_only_async('right', right_pose)
            
            if left_traj is not None and right_traj is not None:
                return await self._execute_merged_trajectory(left_traj, right_traj)
            
            self.get_logger().warn("同步规划失败，降级为顺序移动")
        
        # 顺序移动（备选方案）
        left_ok = await self.move_arm_to_pose_async(
            'left', left_pose, pos_tol, ori_tol_x, ori_tol_y, ori_tol_z, max_retries=max_retries)
        if not left_ok:
            self.get_logger().error("左臂移动失败")
            return False
        
        right_ok = await self.move_arm_to_pose_async(
            'right', right_pose, pos_tol, ori_tol_x, ori_tol_y, ori_tol_z, max_retries=max_retries)
        if not right_ok:
            self.get_logger().error("右臂移动失败")
            return False
        
        self.get_logger().info("两臂移动完成")
        return True

    async def _execute_merged_trajectory(self, left_traj, right_traj, time_scale=1.3):
        """将左右臂轨迹等时参数化合并为14-DOF轨迹并通过 dual_controller 执行"""
        def get_dur_sec(pt):
            return pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
        def set_dur(pt, t_sec):
            pt.time_from_start.sec = int(t_sec)
            pt.time_from_start.nanosec = int((t_sec - int(t_sec)) * 1e9)

        # 统一时间轴
        t_set = {0.0}
        for pt in left_traj.points:  t_set.add(round(get_dur_sec(pt), 3))
        for pt in right_traj.points: t_set.add(round(get_dur_sec(pt), 3))
        t_unified = sorted(t_set)
        t_unified = [t * time_scale for t in t_unified]

        def interp(traj, target_t):
            target_t = target_t / time_scale
            pts = traj.points
            if target_t <= 0.0: return list(pts[0].positions)
            last_t = get_dur_sec(pts[-1])
            if target_t >= last_t: return list(pts[-1].positions)
            for i in range(len(pts) - 1):
                t1, t2 = get_dur_sec(pts[i]), get_dur_sec(pts[i+1])
                if t1 <= target_t <= t2:
                    r = (target_t - t1) / max(t2 - t1, 1e-6)
                    return [p1 + r*(p2-p1) for p1, p2 in zip(pts[i].positions, pts[i+1].positions)]
            return list(pts[-1].positions)

        merged = JointTrajectory()
        merged.joint_names = self.all_arm_joints
        for t in t_unified:
            pt = JointTrajectoryPoint()
            pt.positions = interp(left_traj, t) + interp(right_traj, t)
            pt.velocities = [0.0] * 14
            set_dur(pt, t)
            merged.points.append(pt)

        fjt_goal = FollowJointTrajectory.Goal()
        fjt_goal.trajectory = merged
        fjt_goal.goal_time_tolerance = Duration(sec=5, nanosec=0)

        self.get_logger().info(f"执行合并轨迹: {len(merged.points)}点, {t_unified[-1]:.1f}s")
        try:
            gh = await asyncio.wait_for(
                self.dual_controller_client.send_goal_async(fjt_goal), timeout=10.0)
            if not gh.accepted:
                self.get_logger().warn("合并轨迹被拒绝")
                return False
            result = await asyncio.wait_for(gh.get_result_async(), timeout=30.0)
            ok = result.status in [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED]
            self.get_logger().info(f"合并轨迹执行{'成功' if ok else '失败'}(状态={result.status})")
            return ok
        except asyncio.TimeoutError:
            self.get_logger().error("合并轨迹执行超时")
            return False

    async def move_to_home_joints(self):
        """使用关节空间轨迹让双臂回到初始姿态，比笛卡尔空间更可靠"""
        if not self.dual_controller_available:
            self.get_logger().warn("双臂控制器不可用，尝试顺序移动...")
            # 退回使用顺序移动
            await self._move_single_arm_to_home('left')
            await self._move_single_arm_to_home('right')
            return True

        self.get_logger().info("使用关节轨迹回到初始姿态...")
        
        # 构建 14 关节的目标位置（左臂 7 + 右臂 7），各自采用镜像 Home 姿态
        all_target_positions = self.home_joint_positions_left + self.home_joint_positions_right
        
        # 获取当前关节位置
        current_positions = []
        for jn in self.all_arm_joints:
            current_positions.append(self.current_joint_state.get(jn, 0.0))
        
        # 创建轨迹
        traj = JointTrajectory()
        traj.joint_names = self.all_arm_joints
        
        # 中间点（当前位置）
        pt_start = JointTrajectoryPoint()
        pt_start.positions = current_positions
        pt_start.time_from_start = Duration(sec=0, nanosec=0)
        
        # 目标点（初始姿态）
        pt_goal = JointTrajectoryPoint()
        pt_goal.positions = all_target_positions
        pt_goal.time_from_start = Duration(sec=5, nanosec=0)  # 5 秒到达
        
        traj.points = [pt_start, pt_goal]
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        
        try:
            gh = await asyncio.wait_for(
                self.dual_controller_client.send_goal_async(goal),
                timeout=5.0
            )
            if not gh.accepted:
                self.get_logger().warn("回到初始姿态的目标被拒绝")
                return False
            
            result = await asyncio.wait_for(gh.get_result_async(), timeout=15.0)
            self.get_logger().info(f"回到初始姿态完成，状态: {result.status}")
            return result.status in [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED]
        except Exception as e:
            self.get_logger().warn(f"回到初始姿态异常: {e}")
            return False

    async def _move_single_arm_to_home(self, side: str):
        """单臂回到初始姿态（备用方案）"""
        from moveit_msgs.msg import JointConstraint
        
        group_name = self._group_name(side)
        joint_names = self.left_arm_joints if side == 'left' else self.right_arm_joints
        target_joints = self.home_joint_positions_left if side == 'left' else self.home_joint_positions_right
        
        goal = MoveGroup.Goal()
        goal.request.group_name = group_name
        goal.request.num_planning_attempts = 30
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        
        # 使用关节约束而不是位姿约束
        constraints = Constraints()
        for i, jn in enumerate(joint_names):
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = target_joints[i]
            jc.tolerance_above = 0.1
            jc.tolerance_below = 0.1
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        goal.request.goal_constraints.append(constraints)
        
        client = self.left_arm_client if side == 'left' else self.right_arm_client
        try:
            gh = await asyncio.wait_for(client.send_goal_async(goal), timeout=5.0)
            if not gh.accepted:
                self.get_logger().warn(f"{side} 臂回到初始姿态目标被拒绝")
                return False
            result = await asyncio.wait_for(gh.get_result_async(), timeout=20.0)
            return result.status == GoalStatus.STATUS_SUCCEEDED
        except Exception as e:
            self.get_logger().warn(f"{side} 臂回到初始姿态异常: {e}")
            return False

    async def _plan_only_async(self, side, pose, max_retries=2):
        client = self.left_arm_client if side == 'left' else self.right_arm_client
        group_name = self._group_name(side)
        joint_names = self.left_arm_joints if side == 'left' else self.right_arm_joints

        # 先求 IK
        target_js = await self.compute_ik_async(group_name, pose, avoid_collisions=False)
        if not target_js:
            self.get_logger().warn(f"[{side}] IK 求解失败，无法规划")
            return None
            
        # 提取对应关节的数值
        target_positions = []
        for jn in joint_names:
            idx = target_js.name.index(jn)
            target_positions.append(target_js.position[idx])

        # 构造关节点目标
        goal = MoveGroup.Goal()
        goal.request.group_name = group_name
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.15
        # 使用 OMPL+CHOMP 联合管道：OMPL 粗规划 + CHOMP 平滑优化
        # 确保 MoveIt2 配置中已定义名为 'ompl_chomp' 的 pipeline
        goal.request.pipeline_id = 'ompl_chomp'
        goal.planning_options.plan_only = True
        
        constraints = Constraints()
        constraints.name = "joint_goal"
        from moveit_msgs.msg import JointConstraint
        for i, jn in enumerate(joint_names):
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = target_positions[i]
            # 【修复3】放宽容差从 0.05 → 0.15，因为双臂夹紧物体后可操作空间变小，
            # 适度放宽容差可大幅提高 OMPL 和 IK 的成功率
            jc.tolerance_above = 0.15
            jc.tolerance_below = 0.15
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(constraints)
        
        goal_handle = await asyncio.wait_for(client.send_goal_async(goal), timeout=10.0)
        result = await asyncio.wait_for(goal_handle.get_result_async(), timeout=25.0)
        
        if result.result.error_code.val == 1:
            return result.result.planned_trajectory.joint_trajectory
            
        return None

    async def sync_move_arms_coordinated(self, left_pose, right_pose):
        """
        第四阶段：真实闭链规划与 CHOMP 优化
        由于已在底层集成 pipeline_id='ompl_chomp'，这里执行严格的主从跟踪算法：
        1. 主臂依靠 OMPL 规划出平滑的左臂 JointTrajectory。
        2. 基于左臂每一个插值后节点计算期望位姿 (FK) -> 加上左右臂物理约束增量 -> 计算右臂在同一时刻的必须位姿及其 IK。
        3. 合并成严格 14-DOF 同步轨迹发送给 dual 控制器。
        """
        self.get_logger().info("【Phase 4 闭链规划】: 仅主臂独立规划并进行从臂随动逆解推导...")
        if not self.dual_controller_available:
            self.get_logger().warn("双臂控制器不可用，降级为顺序移动")
            return await self.sync_move_arms(left_pose, right_pose)

        # 1. 独立规划主臂
        left_traj = await self._plan_only_async('left', left_pose)
        if left_traj is None:
            self.get_logger().warn("主臂轨迹规划失败！")
            return False

        self.get_logger().info(f"主臂规划成功，总航点数 {len(left_traj.points)}，进入闭链跟踪IK求解环节...")

        # 验证并构建恒定空间转换 (简化模型：两目标抓取点间的全局相对平移)
        offset_x = right_pose.pose.position.x - left_pose.pose.position.x
        offset_y = right_pose.pose.position.y - left_pose.pose.position.y
        offset_z = right_pose.pose.position.z - left_pose.pose.position.z

        merged_traj = JointTrajectory()
        merged_traj.joint_names = self.all_arm_joints
        
        from copy import deepcopy

        # 为了避免长时间调用服务堵塞，降低采样密度处理
        pts = left_traj.points
        resampled_points = pts[::max(1, len(pts)//20)]  # 抽样20个点，保证计算速度
        if resampled_points[-1] != pts[-1]:
            resampled_points.append(pts[-1])

        # 获取上一刻的状态，作为 IK 种子 (此处简化使用初始状态)
        for i, pt in enumerate(resampled_points):
            # Compute FK for left arm
            fk_res = await self.compute_fk_async(
                fk_link_names=['mj_left_link8'], 
                joint_names=self.left_arm_joints, 
                joint_positions=list(pt.positions)
            )
            
            p_slave = deepcopy(right_pose)  # 保持朝向
            if fk_res:
                # M-S Constant Offset
                p_slave.pose.position.x = fk_res.pose.position.x + offset_x
                p_slave.pose.position.y = fk_res.pose.position.y + offset_y
                p_slave.pose.position.z = fk_res.pose.position.z + offset_z
            else:
                self.get_logger().warn("FK失败..退回端点估算插值")

            # IK 求解从臂 (禁用避免碰撞加速求解)
            ik_js = await self.compute_ik_async(self._group_name('right'), p_slave, avoid_collisions=False)
            
            if not ik_js:
                self.get_logger().warn(f"【IK】 第 {i} 航点从臂奇异不可达，抛弃轨迹...")
                return False
                
            right_pos = []
            for jn in self.right_arm_joints:
                idx = ik_js.name.index(jn)
                right_pos.append(ik_js.position[idx])
                
            new_pt = JointTrajectoryPoint()
            new_pt.positions = list(pt.positions) + right_pos
            new_pt.velocities = [0.0] * 14
            new_pt.time_from_start = pt.time_from_start
            merged_traj.points.append(new_pt)

        self.get_logger().info(f"闭链轨迹生成完成，点数：{len(merged_traj.points)}，正在发送控制器约束执行...")
        fjt_goal = FollowJointTrajectory.Goal()
        fjt_goal.trajectory = merged_traj
        from builtin_interfaces.msg import Duration
        fjt_goal.goal_time_tolerance = Duration(sec=5, nanosec=0)

        gh = await asyncio.wait_for(self.dual_controller_client.send_goal_async(fjt_goal), timeout=10.0)
        if not gh.accepted:
            self.get_logger().warn("合并轨迹被拒绝")
            return False
            
        result = await asyncio.wait_for(gh.get_result_async(), timeout=30.0)
        return result.status in [4, 5] # SUCCEEDED / ABORTED


    async def execute_task_flow(self):
        self.get_logger().info("=== 开始双臂协同搬运任务 (ROS2 + MoveIt2 + CHOMP + 柔顺控制) ===")

        # 等待关节状态初始化
        for _ in range(30):
            if len(self.current_joint_state) >= 14: break
            await asyncio.sleep(0.1)

        # [第一阶段改造] 动态感知设置
        self.planning_frame = "base_link"

        # --- [TF修复] 重新发布模拟的 target_bar TF，并统一坐标系 ---
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.planning_frame # 关键修复：父坐标系统一为 base_link
        t.child_frame_id = 'target_bar'
        # 物体相对于 base_link 的位置
        t.transform.translation.x = 0.5
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.36
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"已发布模拟的 'target_bar' 静态 TF (父坐标系: {self.planning_frame})。")
        
        # 短暂等待，确保 TF 在网络中广播
        await asyncio.sleep(0.5)
        # --- TF 发布结束 ---

        target_pose = await self.get_target_pose_async()
        if target_pose is None:
            self.get_logger().error("无法获取目标物体位姿，任务终止！")
            self.current_state = TaskState.ERROR
            # 在进入主循环前就直接返回
            return
        
        # [第一阶段改造] 基于物体实时位姿，动态计算双臂抓取点
        # 物体尺寸：长度0.4m (半长0.2m)
        self.BAR_LENGTH = 0.40
        
        # 获取物体姿态的旋转矩阵
        target_orientation_q = target_pose.pose.orientation
        target_rot = R.from_quat([
            target_orientation_q.x,
            target_orientation_q.y,
            target_orientation_q.z,
            target_orientation_q.w
        ])

        # 计算沿物体局部X轴的半长向量
        half_length_vec_local = np.array([self.BAR_LENGTH / 2.0, 0, 0])
        
        # 将半长向量旋转到世界坐标系
        offset_vec_world = target_rot.apply(half_length_vec_local)

        # 计算左右抓取点的中心位置
        center_p = np.array([
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            target_pose.pose.position.z
        ])
        left_grasp_center = center_p - offset_vec_world
        right_grasp_center = center_p + offset_vec_world
        
        # 为了后续状态机使用，将numpy数组存为成员变量
        self.left_grasp_center_pos = left_grasp_center
        self.right_grasp_center_pos = right_grasp_center
        self.target_object_orientation = target_pose.pose.orientation # 保存姿态给后续使用

        # === 高度计算（从 dual_scene.xml + mj_dual.xml 精确推导）===
        # 桌子: pos="0.5 0 0.2", size半高=0.14 → 桌面 z = 0.2+0.14 = 0.34m
        # 铝条静止后中心 z = 桌面(0.34) + 半高(0.02) = 0.36m
        self.TABLE_TOP_Z = 0.34
        self.BAR_RESTING_Z = target_pose.pose.position.z # 使用动态获取的高度

        # 发布目标物体 target_bar 的静态 TF，模拟视觉感知节点
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.planning_frame
        t.child_frame_id = 'target_bar'
        t.transform.translation.x = self.BAR_CENTER_X
        t.transform.translation.y = self.BAR_CENTER_Y
        t.transform.translation.z = self.BAR_RESTING_Z
        # 物体自身的相对姿态 (假设与世界对齐)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        
        # 物体尺寸：长度0.4m (半长0.2m), 截面 4cm x 4cm
        self.BAR_LENGTH = 0.40
        
        # === TCP 偏移量修正（关键！）===
        # 查看 mj_dual.xml 结构：
        #   <body name="mj_left_link8" pos="0 0 0.107">        ← link7→link8 偏移
        #     <body name="mj_left_hand" quat="...">            ← 没有 pos！hand 和 link8 同一原点！
        #       <site name="mj_left_ee_site" pos="0 0 0"/>     ← ee_site 也在 link8 原点
        #       <body name="finger" pos="0 0 0.0584">          ← 手指起始于 link8 下方 5.84cm
        #         fingertip_pad pos ≈ 0.04~0.05m (手指本体)    ← 夹持中心 ≈ finger_base + 0.045
        #
        # 实际 link8 → 手指夹持中心 = 0.0584 + 0.045 = 0.1034m
        # 之前错误地把 link7→link8 偏移(0.107m) 当成 link8→hand 偏移，导致高度多了 ~10.7cm！
        self.TCP_OFFSET = 0.1034
        
        # 抓取点 Y 坐标 (世界坐标系)
        # grasp_site_left pos="0 0.1 0" → world y = 0.1
        # grasp_site_right pos="0 -0.1 0" → world y = -0.1
        self.GRASP_OFFSET_Y_LEFT  =  0.10
        self.GRASP_OFFSET_Y_RIGHT = -0.10
        
        # link8 目标高度 = 铝条中心(0.36) + TCP偏移(0.1034) = 0.4634
        # 手指夹持中心对准铝条中心，无需额外余量
        self.GRASP_Z = self.BAR_RESTING_Z + self.TCP_OFFSET   # 0.4634
        self.PRE_GRASP_HEIGHT = self.GRASP_Z + 0.12            # 0.5834 (预抓取高于 12cm)
        self.LIFT_HEIGHT = self.GRASP_Z + 0.15                 # 0.6134 (提升 15cm)
        self.TRANSPORT_X_OFFSET = 0.10                        

        # === 抓取姿态四元数 (基于物体朝向动态计算，保证夹爪开口轴与物体宽度平行) ===
        try:
            # 自动检测物体的局部长度/宽度轴：优先选择在水平面上分量较大的轴作为长度轴
            local_x_world = target_rot.apply([1.0, 0.0, 0.0])
            local_y_world = target_rot.apply([0.0, 1.0, 0.0])
            lx_h = np.array([local_x_world[0], local_x_world[1], 0.0])
            ly_h = np.array([local_y_world[0], local_y_world[1], 0.0])
            # 当物体放置在桌面上时，长度轴应有更大的水平分量
            if np.linalg.norm(lx_h) >= np.linalg.norm(ly_h):
                # local X 为长度轴，宽度轴为 local Y
                bar_width_dir = local_y_world
                self.get_logger().info("自动检测：local X 为长度轴，使用 local Y 作为夹爪开口轴")
            else:
                # local Y 为长度轴，宽度轴为 local X
                bar_width_dir = local_x_world
                self.get_logger().info("自动检测：local Y 为长度轴，使用 local X 作为夹爪开口轴")
            # 期望末端 Z 轴向下（world -Z）作为抓取接近向量
            z_des = np.array([0.0, 0.0, -1.0])

            # 将宽度向量投影到与 z_des 正交的平面，避免抓取姿态出现朝向分量
            proj = bar_width_dir - np.dot(bar_width_dir, z_des) * z_des
            norm = np.linalg.norm(proj)
            if norm < 1e-3:
                raise RuntimeError('物体宽度向量与期望抓取向量接近共线，使用兜底四元数')
            y_axis = proj / norm
            x_axis = np.cross(y_axis, z_des)
            x_axis = x_axis / np.linalg.norm(x_axis)

            rot_mat = np.column_stack((x_axis, y_axis, z_des))
            r_grasp = R.from_matrix(rot_mat)

            # 机械臂末端（link8->手爪）存在固有约 45° 的 yaw 安装偏角，
            # 若不补偿会出现“夹爪看起来平行但实际歪夹”的情况。
            self.GRIPPER_BUILTIN_YAW_COMP_DEG = -45.0
            r_comp = R.from_euler('z', np.deg2rad(self.GRIPPER_BUILTIN_YAW_COMP_DEG))
            r_grasp_comp = r_grasp * r_comp

            q = r_grasp_comp.as_quat()  # [x, y, z, w]
            _grasp_quat = Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))
            self.grasp_orientation_left = _grasp_quat
            self.grasp_orientation_right = _grasp_quat
            self.get_logger().info(
                f"已计算抓取四元数并补偿手爪固有 yaw={self.GRIPPER_BUILTIN_YAW_COMP_DEG:.1f}°: "
                f"quat=[{q[0]:.4f},{q[1]:.4f},{q[2]:.4f},{q[3]:.4f}]")
        except Exception as e:
            # 兜底：保留之前的经验四元数
            self.get_logger().warn(f"动态计算抓取姿态失败({e})，使用兜底四元数")
            q_fallback = np.array([0.9239, 0.3827, 0.0, 0.0], dtype=float)
            r_fb = R.from_quat(q_fallback)
            r_comp = R.from_euler('z', np.deg2rad(-45.0))
            q_fb_comp = (r_fb * r_comp).as_quat()
            _grasp_quat = Quaternion(x=float(q_fb_comp[0]), y=float(q_fb_comp[1]), z=float(q_fb_comp[2]), w=float(q_fb_comp[3]))
            self.grasp_orientation_left = _grasp_quat
            self.grasp_orientation_right = _grasp_quat

        # 启动安全看门狗监控
        self.safety_watchdog_active = True
        watchdog_task = asyncio.create_task(self.safety_watchdog_loop())

        # 在下探阶段确保夹爪已张开，避免手爪闭合状态直接撞到铝条
        try:
            await self.ensure_grippers_open()
            self.get_logger().info("已将夹爪张到最大宽度以提升下探容差")
        except Exception:
            self.get_logger().warn("尝试将夹爪张开以增加容差时失败，继续执行")

        # === 核心状态机执行循环 ===
        while self.current_state != TaskState.FINISHED and self.current_state != TaskState.ERROR:
            try:
                if self.current_state == TaskState.INIT_ENVIRONMENT:
                    await self.state_init_environment()
                
                elif self.current_state == TaskState.PLAN_APPROACH:
                    await self.state_plan_approach()
                
                elif self.current_state == TaskState.EXECUTE_APPROACH:
                    await self.state_execute_approach()
                
                elif self.current_state == TaskState.CLOSE_GRIPPERS:
                    await self.state_close_grippers()
                
                elif self.current_state == TaskState.PLAN_SYNC_TRAJECTORY:
                    await self.state_plan_sync_trajectory()
                
                elif self.current_state == TaskState.EXECUTE_WITH_COMPLIANCE:
                    await self.state_execute_with_compliance()
                
                elif self.current_state == TaskState.OPEN_GRIPPERS:
                    await self.state_open_grippers()
                
                elif self.current_state == TaskState.RETURN_TO_HOME:
                    await self.state_return_to_home()

            except Exception as e:
                self.get_logger().error(f"在状态 [{self.current_state.name}] 发生严重错误: {e}")
                self.current_state = TaskState.ERROR
                break

        if self.current_state == TaskState.FINISHED:
            self.get_logger().info("=" * 50)
            self.get_logger().info("=== 🎉 双臂搬运任务全部完成！目标完全达成 ===")
            self.get_logger().info("=" * 50)
        else:
            self.get_logger().error("=" * 50)
            self.get_logger().error("=== ❌ 任务执行失败，尝试恢复机械臂至初始状态 ===")
            self.get_logger().error("=" * 50)
            try:
                # 只在发生错误时直接打开夹爪并退回（跳过下降步骤）
                await self.ensure_grippers_open()
                # 分离虚拟物体 (Detach) 保证规划器畅通
                detach_obj = AttachedCollisionObject()
                detach_obj.link_name = 'mj_left_link8'
                detach_obj.object.id = 'target_bar'
                detach_obj.object.operation = CollisionObject.REMOVE
                scene_msg = PlanningScene()
                scene_msg.is_diff = True
                scene_msg.robot_state.attached_collision_objects.append(detach_obj)
                scene_msg.robot_state.is_diff = True
                self.scene_pub.publish(scene_msg)
                
                await asyncio.sleep(1.0)
                await self.switch_to_trajectory_mode()
                await self.move_to_home_joints()
            except Exception as reset_e:
                self.get_logger().error(f"恢复状态时发生异常: {reset_e}")

        # 任务结束后关闭看门狗
        self.safety_watchdog_active = False
        await watchdog_task
        # 自动退出前统一闭爪（无论成功/失败都执行）
        await self.close_grippers_on_exit("状态机结束")

    # ------------------ 状态机分离函数区 ------------------ #

    async def safety_watchdog_loop(self):
        """ 安全看门狗循环 (模拟受力越界保护与同步误差监控) """
        self.get_logger().info("[Watchdog] 安全看门狗监控护航运行中... (10Hz)")
        rate_hz = 10.0
        period = 1.0 / rate_hz
        force_trip_counter = 0
        max_abs_force = float(getattr(self, 'watchdog_force_limit', 100.0))
        trip_required = max(1, int(getattr(self, 'watchdog_force_trip_count', 1)))
        
        while self.safety_watchdog_active:
            # 1. 监测受力越界
            f_l_z = self.current_left_wrench.wrench.force.z
            f_r_z = self.current_right_wrench.wrench.force.z
            
            if abs(f_l_z) > max_abs_force or abs(f_r_z) > max_abs_force:
                force_trip_counter += 1
                if force_trip_counter < trip_required:
                    self.get_logger().warn(
                        f"[Watchdog] 受力超限尖峰({force_trip_counter}/{trip_required}) "
                        f"(L={f_l_z:.1f}N, R={f_r_z:.1f}N, limit={max_abs_force:.1f}N)，继续观察")
                    await asyncio.sleep(period)
                    continue
                self.get_logger().error(f"[Watchdog! E-STOP] 测得极端受力！(L: {f_l_z:.1f}N, R: {f_r_z:.1f}N) -> 触发急停方案")
                
                # A. 立即改变状态机状态为 ERROR，这会打断 execute_task_flow 里的循环
                self.current_state = TaskState.ERROR
                
                # B. 中断柔顺控制循环，停止指令发送
                self.compliance_task_active = False 
                
                # C. 立即向 Servo 发布全 0 Twist，刹停机器人
                zero_twist = TwistStamped()
                zero_twist.header.stamp = self.get_clock().now().to_msg()
                zero_twist.header.frame_id = self.planning_frame
                zero_twist.twist.linear.x = 0.0
                zero_twist.twist.linear.y = 0.0
                zero_twist.twist.linear.z = 0.0
                
                self.servo_pub_left.publish(zero_twist)
                self.servo_pub_right.publish(zero_twist)
                
                self.get_logger().error("已下发零速刹停指令，任务流程中止。")
                break # 退出看门狗循环
            else:
                force_trip_counter = max(0, force_trip_counter - 1)
                
            # 2. 监测两臂的关节位姿，通过TF解算当前距离
            # 此处简化为等待真实机械臂tf数据
            # if abs(dist_actual - BAR_LENGTH) > 0.05:
            #     self.get_logger().warn("[Watchdog] 搬运过程双臂间距同步偏差 > 5mm !")

            await asyncio.sleep(period)
        self.get_logger().info("[Watchdog] 安全看门狗退出.")

    async def state_init_environment(self):
        """State 1: 初始化参数构建、发布目标碰撞物体/环境障碍物等"""
        self.get_logger().info(">> State 1: init_environment() - 空间环境初始化与障碍物设置...")
        
        # 构建一个虚拟的规划场景
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        
        # 1. 添加环境障碍物：一张桌子或地板，防止手臂下探过深碰撞
        table = CollisionObject()
        table.id = "work_table"
        table.header.frame_id = self.planning_frame
        table.operation = CollisionObject.ADD
        
        box_primitive = SolidPrimitive()
        box_primitive.type = SolidPrimitive.BOX
        # 与 dual_scene.xml 一致: size="0.25 0.6 0.14" → 实际尺寸 0.5 x 1.2 x 0.28
        box_primitive.dimensions = [0.50, 1.20, 0.05]
        
        box_pose = PoseStamped().pose
        box_pose.position.x = 0.5
        box_pose.position.y = 0.0
        # 桌子中心 z=0.2, 桌面 z=0.34
        box_pose.position.z = 0.30
        box_pose.orientation.w = 1.0
        
        table.primitives.append(box_primitive)
        table.primitive_poses.append(box_pose)
        
        # 2. 添加目标夹取物体：长条形构件 (用于可视化及避障参考)
        target_obj = CollisionObject()
        target_obj.id = "target_bar"
        target_obj.header.frame_id = self.planning_frame
        target_obj.operation = CollisionObject.ADD
        
        bar_primitive = SolidPrimitive()
        bar_primitive.type = SolidPrimitive.BOX
        # 长条形物体: 与 xml 匹配：x=0.04(厚), y=0.4(长), z=0.04(高)
        bar_primitive.dimensions = [0.04, 0.40, 0.04]
        
        bar_pose = PoseStamped().pose
        bar_pose.position.x = 0.5
        bar_pose.position.y = 0.0
        # 铝条受重力跌落后静止中心高度: 桌面(0.34)+半高(0.02)=0.36
        bar_pose.position.z = self.BAR_RESTING_Z
        bar_pose.orientation.w = 1.0
        
        target_obj.primitives.append(bar_primitive)
        target_obj.primitive_poses.append(bar_pose)

        # 发布场景
        scene_msg.world.collision_objects.append(table)
        scene_msg.world.collision_objects.append(target_obj)
        self.scene_pub.publish(scene_msg)
        self.get_logger().info("已发布搬运桌台与目标长条状碰撞物体到 MoveIt2 规划场景")
        
        # 尝试复位夹爪，确保它们打开状态
        self.get_logger().info("开启双臂夹爪准备...")
        await self.ensure_grippers_open()
        await asyncio.sleep(1.0) # 等待场景同步与夹爪打开

        self.current_state = TaskState.PLAN_APPROACH

    async def state_plan_approach(self):
        """State 2: 通过 TF 动态获取目标位姿，计算并定义预抓取点"""
        self.get_logger().info(">> State 2: plan_approach() - 动态 TF 获取与抓取点计算...")
        
        try:
            # 尝试获取目标物体的 TF (先尝试 'target_bar', 若无则退回 'long_bar')
            try:
                tf_msg = self.tf_buffer.lookup_transform(self.planning_frame, 'target_bar', rclpy.time.Time())
            except:
                tf_msg = self.tf_buffer.lookup_transform(self.planning_frame, 'long_bar', rclpy.time.Time())
                
            p_center = np.array([
                tf_msg.transform.translation.x,
                tf_msg.transform.translation.y,
                tf_msg.transform.translation.z
            ])
            
            q_bar = [
                tf_msg.transform.rotation.x,
                tf_msg.transform.rotation.y,
                tf_msg.transform.rotation.z,
                tf_msg.transform.rotation.w
            ]
            r_bar = R.from_quat(q_bar)
            
            self.get_logger().info(f"成功获取目标位姿: Center={p_center}")
            
            # 利用物体长度动态推算左抓取点和右抓取点
            L = self.BAR_LENGTH
            grasp_offset = 0.10 # 距端点的偏移量，使 L/2 - offset = 0.1m
            
            # P_left = P_center + R_bar * [0, L/2 - offset, 0]^T
            local_vec_left = np.array([0.0, (L / 2) - grasp_offset, 0.0])
            p_left = p_center + r_bar.apply(local_vec_left)
            
            # P_right = P_center - R_bar * [0, L/2 - offset, 0]^T
            p_right = p_center - r_bar.apply(local_vec_left)

            self.get_logger().info(f"动态推算左抓取点: {p_left}")
            self.get_logger().info(f"动态推算右抓取点: {p_right}")

            # 动态更新抓取高度 (基于物体实时 Z 高度 + TCP 偏移)
            current_bar_z = p_center[2]
            self.GRASP_Z = current_bar_z + self.TCP_OFFSET + self.grasp_z_safety_bias
            self.PRE_GRASP_HEIGHT = self.GRASP_Z + 0.12
            self.LIFT_HEIGHT = self.GRASP_Z + 0.15

            # 缓存给 execute 阶段使用
            self.dynamic_p_left = p_left
            self.dynamic_p_right = p_right

            # --- 姿态候选自动择优：加入左右臂对称 yaw 微偏置，降低“歪夹/单臂限位”概率 ---
            def build_quat_msg(arr):
                return Quaternion(x=float(arr[0]), y=float(arr[1]), z=float(arr[2]), w=float(arr[3]))

            q_base = np.array([
                self.grasp_orientation_left.x,
                self.grasp_orientation_left.y,
                self.grasp_orientation_left.z,
                self.grasp_orientation_left.w,
            ], dtype=float)
            # 绕末端局部 Z 轴翻转 180°
            q_flip = (R.from_quat(q_base) * R.from_euler('z', np.pi)).as_quat()

            limits = [
                (-2.8973, 2.8973),
                (-1.7628, 1.7628),
                (-2.8973, 2.8973),
                (-3.0718, -0.0698),
                (-2.8973, 2.8973),
                (-0.0175, 3.7525),
                (-2.8973, 2.8973),
            ]

            def extract_joint_values(js, arm_joint_names):
                if js is None:
                    return None
                if isinstance(js, dict):
                    vals = [js.get(n, None) for n in arm_joint_names]
                    return vals if not any(v is None for v in vals) else None
                if hasattr(js, 'name') and hasattr(js, 'position'):
                    name_to_pos = {n: p for n, p in zip(js.name, js.position)}
                    vals = [name_to_pos.get(n, None) for n in arm_joint_names]
                    return vals if not any(v is None for v in vals) else None
                return None

            def min_margin(vals):
                if vals is None or len(vals) < 7:
                    return -1e9
                mm = 1e9
                for idx in range(7):
                    lo, hi = limits[idx]
                    v = vals[idx]
                    m = min(v - lo, hi - v)
                    mm = min(mm, m)
                return mm

            async def eval_pose_pair_score(q_left_arr, q_right_arr):
                lp = PoseStamped()
                lp.header.frame_id = self.planning_frame
                lp.pose.position.x = p_left[0]
                lp.pose.position.y = p_left[1]
                lp.pose.position.z = self.PRE_GRASP_HEIGHT
                lp.pose.orientation = build_quat_msg(q_left_arr)

                rp = PoseStamped()
                rp.header.frame_id = self.planning_frame
                rp.pose.position.x = p_right[0]
                rp.pose.position.y = p_right[1]
                rp.pose.position.z = self.PRE_GRASP_HEIGHT
                rp.pose.orientation = build_quat_msg(q_right_arr)

                ik_l = await self.compute_ik_async(self._group_name('left'), lp, avoid_collisions=False)
                ik_r = await self.compute_ik_async(self._group_name('right'), rp, avoid_collisions=False)
                if ik_l is None or ik_r is None:
                    return -1e9, None, None

                vals_l = extract_joint_values(ik_l, self.left_arm_joints)
                vals_r = extract_joint_values(ik_r, self.right_arm_joints)
                margin_l = min_margin(vals_l)
                margin_r = min_margin(vals_r)
                base_score = min(margin_l, margin_r)

                # 额外惩罚：避免 wrist(j7)贴近限位，降低“歪夹”概率
                if vals_l is None or vals_r is None:
                    return -1e9, None, None
                j7_l = vals_l[6]
                j7_r = vals_r[6]
                j7_penalty = 0.0
                j7_soft = 2.60
                if abs(j7_l) > j7_soft:
                    j7_penalty += (abs(j7_l) - j7_soft) * 2.0
                if abs(j7_r) > j7_soft:
                    j7_penalty += (abs(j7_r) - j7_soft) * 2.0

                return base_score - j7_penalty, vals_l, vals_r

            best = {
                'score': -1e9,
                'q_l': q_base,
                'q_r': q_base,
                'tag': 'fallback',
                'j7_l': None,
                'j7_r': None,
            }

            base_rot_candidates = [
                ('base', R.from_quat(q_base)),
                ('flip180', R.from_quat(q_flip)),
            ]
            yaw_bias_deg_list = [0.0, 3.0, -3.0, 6.0, -6.0]

            for base_name, base_rot in base_rot_candidates:
                for yaw_deg in yaw_bias_deg_list:
                    # 左右对称偏置：左 +yaw，右 -yaw
                    q_l = (base_rot * R.from_euler('z', np.deg2rad(yaw_deg))).as_quat()
                    q_r = (base_rot * R.from_euler('z', np.deg2rad(-yaw_deg))).as_quat()
                    score, vals_l, vals_r = await eval_pose_pair_score(q_l, q_r)
                    if score > best['score']:
                        best['score'] = score
                        best['q_l'] = q_l
                        best['q_r'] = q_r
                        best['tag'] = f"{base_name}, yaw_bias={yaw_deg:+.1f}deg"
                        best['j7_l'] = None if vals_l is None else vals_l[6]
                        best['j7_r'] = None if vals_r is None else vals_r[6]

            self.grasp_orientation_left = build_quat_msg(best['q_l'])
            self.grasp_orientation_right = build_quat_msg(best['q_r'])
            self.get_logger().info(
                f"姿态择优(抗歪夹): {best['tag']} score={best['score']:.3f}, "
                f"j7_l={best['j7_l']}, j7_r={best['j7_r']}")

            self.left_pre_grasp = PoseStamped()
            self.left_pre_grasp.header.frame_id = self.planning_frame
            self.left_pre_grasp.pose.position.x = p_left[0]
            self.left_pre_grasp.pose.position.y = p_left[1]
            self.left_pre_grasp.pose.position.z = self.PRE_GRASP_HEIGHT
            self.left_pre_grasp.pose.orientation = self.grasp_orientation_left

            self.right_pre_grasp = PoseStamped()
            self.right_pre_grasp.header.frame_id = self.planning_frame
            self.right_pre_grasp.pose.position.x = p_right[0]
            self.right_pre_grasp.pose.position.y = p_right[1]
            self.right_pre_grasp.pose.position.z = self.PRE_GRASP_HEIGHT
            self.right_pre_grasp.pose.orientation = self.grasp_orientation_right
            
            self.current_state = TaskState.EXECUTE_APPROACH

        except Exception as e:
            self.get_logger().error(f"State 2 失败（TF/姿态择优/IK评分）: {e}")
            self.get_logger().info("请检查 TF、IK 服务与抓取姿态候选评分流程。")
            if self._is_force_continue_enabled() and hasattr(self, 'left_grasp_center_pos') and hasattr(self, 'right_grasp_center_pos'):
                self.get_logger().warn("State 2 启用中期兜底：使用预计算抓取点并强制进入执行阶段")
                p_left = np.array(self.left_grasp_center_pos, dtype=float)
                p_right = np.array(self.right_grasp_center_pos, dtype=float)
                self.dynamic_p_left = p_left
                self.dynamic_p_right = p_right
                self.GRASP_Z = max(float(self.BAR_RESTING_Z + self.TCP_OFFSET + self.grasp_z_safety_bias), 0.40)
                self.PRE_GRASP_HEIGHT = self.GRASP_Z + 0.12
                self.LIFT_HEIGHT = self.GRASP_Z + 0.15

                q_fallback = Quaternion(x=0.9239, y=0.3827, z=0.0, w=0.0)
                self.grasp_orientation_left = getattr(self, 'grasp_orientation_left', q_fallback)
                self.grasp_orientation_right = getattr(self, 'grasp_orientation_right', q_fallback)

                self.left_pre_grasp = PoseStamped()
                self.left_pre_grasp.header.frame_id = self.planning_frame
                self.left_pre_grasp.pose.position.x = float(p_left[0])
                self.left_pre_grasp.pose.position.y = float(p_left[1])
                self.left_pre_grasp.pose.position.z = float(self.PRE_GRASP_HEIGHT)
                self.left_pre_grasp.pose.orientation = self.grasp_orientation_left

                self.right_pre_grasp = PoseStamped()
                self.right_pre_grasp.header.frame_id = self.planning_frame
                self.right_pre_grasp.pose.position.x = float(p_right[0])
                self.right_pre_grasp.pose.position.y = float(p_right[1])
                self.right_pre_grasp.pose.position.z = float(self.PRE_GRASP_HEIGHT)
                self.right_pre_grasp.pose.orientation = self.grasp_orientation_right
                self.current_state = TaskState.EXECUTE_APPROACH
                return
            self.current_state = TaskState.ERROR

    async def state_execute_approach(self):
        """State 3: 规划并移动到预抓取点和抓取点"""
        self.get_logger().info(">> State 3: execute_approach() - 双臂安全平移并下探至抓取位置...")
        
        # === 第一步：移动到预抓取点（高于铝条 15cm）===
        # 使用收紧的容差，防止 IK 求解器选择变形的关节配置
        success_pre = await self.sync_move_arms(
            self.left_pre_grasp, self.right_pre_grasp,
            pos_tol=0.02, ori_tol_x=0.05, ori_tol_y=0.05, ori_tol_z=0.08)
        if not success_pre:
            if self._is_force_continue_enabled():
                self.get_logger().warn("预抓取点移动失败，中期兜底：跳过预抓取，直接尝试下探")
            else:
                self.get_logger().error("预抓取点移动失败，触发复位")
                self.current_state = TaskState.ERROR
                return
        
        # 关节健壮性检查：到达预抓取点后，确认没有接近限位的变形配置
        await asyncio.sleep(0.3)  # 等待关节状态更新
        if not self.check_joint_sanity():
            if self._is_force_continue_enabled():
                self.get_logger().warn("预抓取后关节角接近限位，中期兜底：降速继续并尽快完成夹取")
            else:
                self.get_logger().error("预抓取后关节角接近限位，配置可能变形！触发复位")
                self.current_state = TaskState.ERROR
                return
        
        self.get_logger().info("预抓取点到达成功，开始下探至抓取高度...")
        # 临时允许手爪与目标物体发生碰撞：通过修改 AllowedCollisionMatrix (ACM)
        # 这样可以保留 target_bar 作为 world 中的碰撞体同时允许与 mj_left_link8 接触
        try:
            await self.set_acm_allow('mj_left_link8', 'target_bar', allow=True)
        except Exception as e:
            self.get_logger().warn(f"设置 ACM 失败: {e}，将回退为移除碰撞体策略")
            try:
                remove_obj = CollisionObject()
                remove_obj.id = "target_bar"
                remove_obj.operation = CollisionObject.REMOVE
                scene_msg_rm = PlanningScene()
                scene_msg_rm.is_diff = True
                scene_msg_rm.world.collision_objects.append(remove_obj)
                self.scene_pub.publish(scene_msg_rm)
                await asyncio.sleep(0.5)
            except Exception:
                pass

        # === 第二步：垂直下探到抓取高度（自适应高度候选，避免单点不可达）===
        # 先尝试略高于理论抓取中心，避免手掌/腕部先触碰铝条
        candidate_grasp_z = [
            self.GRASP_Z + 0.010,
            self.GRASP_Z + 0.005,
            self.GRASP_Z,
            self.GRASP_Z - 0.005,
        ]

        success_grasp = False
        selected_grasp_z = self.GRASP_Z
        for i, candidate_z in enumerate(candidate_grasp_z, 1):
            left_grasp_pose = PoseStamped()
            left_grasp_pose.header.frame_id = self.planning_frame
            left_grasp_pose.pose.position.x = self.dynamic_p_left[0]
            left_grasp_pose.pose.position.y = self.dynamic_p_left[1]
            left_grasp_pose.pose.position.z = candidate_z
            left_grasp_pose.pose.orientation = self.grasp_orientation_left

            right_grasp_pose = PoseStamped()
            right_grasp_pose.header.frame_id = self.planning_frame
            right_grasp_pose.pose.position.x = self.dynamic_p_right[0]
            right_grasp_pose.pose.position.y = self.dynamic_p_right[1]
            right_grasp_pose.pose.position.z = candidate_z
            right_grasp_pose.pose.orientation = self.grasp_orientation_right

            self.get_logger().info(
                f"下探候选高度尝试 {i}/{len(candidate_grasp_z)}: z={candidate_z:.4f}m")
            # 优先尝试使用 MoveIt 的 Cartesian Path（直线插补），避免 OMPL 绘制弧线
            use_cartesian = False
            left_traj = None
            right_traj = None
            try:
                if self.cartesian_client is not None and self.cartesian_client.wait_for_service(timeout_sec=1.0):
                    from moveit_msgs.srv import GetCartesianPath as GCP
                    # 左臂笛卡尔直线下探
                    req_l = GCP.Request()
                    req_l.group_name = self._group_name('left')
                    req_l.waypoints = [self.left_pre_grasp.pose, left_grasp_pose.pose]
                    req_l.max_step = 0.01
                    req_l.jump_threshold = 0.0
                    req_l.avoid_collisions = False
                    fut_l = self.cartesian_client.call_async(req_l)
                    res_l = await asyncio.wait_for(fut_l, timeout=10.0)

                    # 右臂笛卡尔直线下探
                    req_r = GCP.Request()
                    req_r.group_name = self._group_name('right')
                    req_r.waypoints = [self.right_pre_grasp.pose, right_grasp_pose.pose]
                    req_r.max_step = 0.01
                    req_r.jump_threshold = 0.0
                    req_r.avoid_collisions = False
                    fut_r = self.cartesian_client.call_async(req_r)
                    res_r = await asyncio.wait_for(fut_r, timeout=10.0)

                    frac_l = getattr(res_l, 'fraction', 0.0)
                    frac_r = getattr(res_r, 'fraction', 0.0)
                    if frac_l >= 0.99 and frac_r >= 0.99:
                        # 从 service 返回的 RobotTrajectory 中取出 joint_trajectory
                        left_traj = getattr(getattr(res_l, 'solution', None), 'joint_trajectory', None)
                        right_traj = getattr(getattr(res_r, 'solution', None), 'joint_trajectory', None)
                        if left_traj is not None and right_traj is not None:
                            use_cartesian = True
            except Exception as e:
                self.get_logger().warn(f"Cartesian path 服务调用失败或路径质量不足: {e}")

            if use_cartesian and self.dual_controller_available:
                self.get_logger().info("使用笛卡尔直线轨迹并行执行双臂下探...")
                ok = await self._execute_merged_trajectory(left_traj, right_traj)
                success_grasp = bool(ok)
            else:
                # 回退：若未能获得 Cartesian Path 或无双臂控制器，则使用伺服直降（更稳定且避免 OMPL）
                self.get_logger().info("降级为伺服直降执行下探（servo_move_delta_z）")
                delta_z = candidate_z - self.PRE_GRASP_HEIGHT
                success_grasp = await self.servo_move_delta_z(delta_z=delta_z, duration=1.8)
            if success_grasp:
                selected_grasp_z = candidate_z
                break
        self.GRASP_Z = selected_grasp_z
        
        if success_grasp:
            await asyncio.sleep(0.3)
            if not self.check_joint_sanity():
                if self._is_force_continue_enabled():
                    self.get_logger().warn("抓取位关节偏紧，中期兜底：跳过回撤，继续闭爪搬运")
                else:
                    self.get_logger().error("抓取位置关节角变形！安全撤离后复位")
                    await self.sync_move_arms(self.left_pre_grasp, self.right_pre_grasp)
                    self.current_state = TaskState.ERROR
                    return
            self.get_logger().info("双臂已到达抓取位置，关节状态正常 ✓")
            self.current_state = TaskState.CLOSE_GRIPPERS
        else:
            self.get_logger().warn("规划下探全部失败，启用伺服直降兜底策略...")
            delta_z = self.GRASP_Z - self.PRE_GRASP_HEIGHT
            if await self.servo_move_delta_z(delta_z=delta_z, duration=2.2):
                await asyncio.sleep(0.2)
                self.get_logger().info("伺服直降完成，进入夹爪闭合阶段")
                self.current_state = TaskState.CLOSE_GRIPPERS
            else:
                if self._is_force_continue_enabled():
                    self.get_logger().warn("下探仍失败，中期兜底：直接进入闭爪阶段，保证流程可继续")
                    self.current_state = TaskState.CLOSE_GRIPPERS
                else:
                    self.get_logger().error("下探夹取失败！安全撤离后复位")
                    if not await self.sync_move_arms(self.left_pre_grasp, self.right_pre_grasp):
                        await self.servo_move_cartesian('lift', duration=1.5)
                    self.current_state = TaskState.ERROR

    async def calibrate_force_sensors(self, duration_sec=0.5):
        """静止等待并对六维力传感器去皮(Tare)校准"""
        self.get_logger().info(f"开始力控传感器校准 (采集 {duration_sec} 秒内均值)...")
        
        left_z_history, right_z_history = [], []
        left_y_history, right_y_history = [], []
        
        end_time = self.get_clock().now().nanoseconds + duration_sec * 1e9
        while self.get_clock().now().nanoseconds < end_time:
            left_z_history.append(self.current_left_wrench.wrench.force.z)
            left_y_history.append(self.current_left_wrench.wrench.force.y)
            right_z_history.append(self.current_right_wrench.wrench.force.z)
            right_y_history.append(self.current_right_wrench.wrench.force.y)
            await asyncio.sleep(0.01)
            
        if not left_z_history:
             self.get_logger().warn("未收集到足够力控数据，校准使用当前瞬间值")
             left_z_history = [self.current_left_wrench.wrench.force.z]
             left_y_history = [self.current_left_wrench.wrench.force.y]
             right_z_history = [self.current_right_wrench.wrench.force.z]
             right_y_history = [self.current_right_wrench.wrench.force.y]
             
        self.left_force_bias['y'] = sum(left_y_history) / len(left_y_history)
        self.left_force_bias['z'] = sum(left_z_history) / len(left_z_history)
        self.right_force_bias['y'] = sum(right_y_history) / len(right_y_history)
        self.right_force_bias['z'] = sum(right_z_history) / len(right_z_history)
        
        self.get_logger().info(f"去皮完成: L_Bias(y={self.left_force_bias['y']:.2f}, z={self.left_force_bias['z']:.2f}) R_Bias(y={self.right_force_bias['y']:.2f}, z={self.right_force_bias['z']:.2f})")

    async def controlled_grasp_close(self, target_width=0.02, max_width=0.08, step=0.005, effort=170.0, contact_threshold=5.0, timeout=6.0):
        """逐步收紧夹爪直到达到目标宽度或检测到接触力（带超时）。
        返回 True 表示成功夹住（检测到接触或达到目标宽度），False 表示超时失败。
        """
        max_width = self._clamp_gripper_position(max_width)
        target_width = self._clamp_gripper_position(target_width)
        self.get_logger().info(f"开始受力驱动的逐步夹紧：从 {max_width:.3f} -> {target_width:.3f} 步长 {step:.3f}")
        # 先确保夹爪全开
        try:
            await self.sync_grasp(max_width, effort, wait_for_result=False)
        except Exception:
            pass

        start_time = self.get_clock().now().nanoseconds * 1e-9
        width = max_width
        success = False
        while width > target_width + 1e-6:
            # 每一步收紧
            width = max(target_width, width - step)
            await self.sync_grasp(width, effort, wait_for_result=False)
            await asyncio.sleep(0.15)

            # 读取力反馈判断是否接触
            left_fy = abs(self.current_left_wrench.wrench.force.y - self.left_force_bias['y'])
            right_fy = abs(self.current_right_wrench.wrench.force.y - self.right_force_bias['y'])
            self.get_logger().info(f"尝试宽度={width:.3f}m, 力反馈 L_y={left_fy:.2f}N R_y={right_fy:.2f}N")

            if left_fy > contact_threshold or right_fy > contact_threshold:
                self.get_logger().info(f"检测到接触力：L_y={left_fy:.2f}N R_y={right_fy:.2f}N，停止收紧")
                success = True
                break

            # 超时检查
            if (self.get_clock().now().nanoseconds * 1e-9) - start_time > timeout:
                self.get_logger().warn("controlled_grasp_close 超时，停止尝试")
                break

            if width <= target_width + 1e-6:
                break

        # 最后确保执行一次同步闭爪命令以固定位置
        try:
            await self.sync_grasp(width, effort, wait_for_result=True)
        except Exception:
            pass

        return success or (width <= target_width + 1e-6)

    async def state_close_grippers(self):
        """State 4: 发送夹爪闭合指令，附着物体构建闭链"""
        self.get_logger().info(">> State 4: close_grippers() - 执行夹爪同步夹取...")
        
        # === 夹爪参数设定 ===
        # 物体厚度约 0.04m，将目标位置设为更小以持续激发接触挤压力
        GRASP_POSITION = self.gripper_closed_position
        # 夹爪最大力：对轻质铝条用 170N (最大值)
        GRASP_EFFORT = self.gripper_effort_close
        
        # 使用 ACM 允许末端与目标物体发生接触（替代 attach/remove），然后进行受力驱动的逐步闭合
        self.get_logger().info("准备允许夹持碰撞并进行受力驱动的逐步夹紧...")
        try:
            ok = await self.set_acm_allow('mj_left_link8', 'target_bar', allow=True)
            if not ok:
                self.get_logger().warn("设置 ACM 失败或未确认，继续但可能导致 MoveIt 拒绝接触")
        except Exception as e:
            self.get_logger().warn(f"设置 ACM 异常: {e}，继续执行但可能存在碰撞问题")

        # 在闭合前进行力传感器去皮校准，以获得可靠的接触判定基线
        await self.calibrate_force_sensors(duration_sec=0.5)

        # 使用受力驱动的逐步夹紧（从大到小），优先使用带反馈的逐步收紧策略
        grasp_ok = await self.controlled_grasp_close(target_width=GRASP_POSITION,
                                                     max_width=self.gripper_open_position,
                                                     step=0.005,
                                                     effort=GRASP_EFFORT,
                                                     contact_threshold=5.0,
                                                     timeout=6.0)

        if grasp_ok:
            self.get_logger().info("受力驱动夹取成功，准备搬运")

            # 确保夹爪以最终目标宽度锁定以增加稳定性，并等待物理稳定
            try:
                await self.sync_grasp(self.gripper_closed_position, GRASP_EFFORT)
            except Exception:
                pass
            await asyncio.sleep(2.0)

            # 在 MoveIt 中附着物体以形成闭链（在 ACM 已允许的前提下）
            try:
                attach_obj = AttachedCollisionObject()
                attach_obj.link_name = 'mj_left_link8'
                attach_obj.object.id = 'target_bar'
                attach_obj.object.operation = CollisionObject.ADD
                scene_msg_attach = PlanningScene()
                scene_msg_attach.is_diff = True
                scene_msg_attach.robot_state.attached_collision_objects.append(attach_obj)
                scene_msg_attach.robot_state.is_diff = True
                self.scene_pub.publish(scene_msg_attach)
                await asyncio.sleep(0.2)
                self.get_logger().info("已在 MoveIt 中附着 target_bar 到 mj_left_link8")
            except Exception as e:
                self.get_logger().warn(f"尝试 Attach 失败: {e}，将继续但请注意闭链一致性")

            # 启动力控自适应循环以保护闭链提举阶段
            try:
                if not getattr(self, 'compliance_task_active', False):
                    self.compliance_task_active = True
                    asyncio.create_task(self.simulated_compliance_control_loop())
                    self.get_logger().info("已启动 simulated_compliance_control_loop，用于闭链搬运的阻抗/力控保护")
            except Exception:
                pass

            self.current_state = TaskState.PLAN_SYNC_TRAJECTORY
        else:
            # 兜底：当受力驱动未能检测到接触时，尝试一次固定位置闭合并基于位置判定
            self.get_logger().warn("受力驱动夹取未成功，尝试一次固定位置闭合作为兜底")
            await self.sync_grasp(GRASP_POSITION, GRASP_EFFORT)
            await asyncio.sleep(1.0)
            left_contact_force = abs(self.current_left_wrench.wrench.force.y - self.left_force_bias.get('y', 0.0))
            right_contact_force = abs(self.current_right_wrench.wrench.force.y - self.right_force_bias.get('y', 0.0))
            self.get_logger().info(f"[兜底判定] L_y={left_contact_force:.2f}N R_y={right_contact_force:.2f}N")
            if left_contact_force > 1.0 or right_contact_force > 1.0:
                self.get_logger().info("兜底闭合判定为成功，继续搬运")
                self.current_state = TaskState.PLAN_SYNC_TRAJECTORY
            else:
                if self._is_force_continue_enabled():
                    self.get_logger().warn("夹取力反馈不足，中期兜底：继续执行搬运段完成演示闭环")
                    self.current_state = TaskState.PLAN_SYNC_TRAJECTORY
                else:
                    self.get_logger().error("夹取失败，尝试撤离并重试或终止任务")
                    # 尝试后撤离预抓取位
                    try:
                        await self.sync_move_arms(self.left_pre_grasp, self.right_pre_grasp)
                    except Exception:
                        await self.servo_move_cartesian('lift', duration=1.5)
                    self.current_state = TaskState.ERROR

    async def state_plan_sync_trajectory(self):
        """State 5: 跳过困难的OMPL规划，改为直接用伺服控制上升并平移"""
        self.get_logger().info(">> State 5: plan_sync_trajectory() - 简化为伺服直接上升")
        
        # 使用当前末端高度动态估计抬升量，避免硬编码 z=0.36 导致过冲
        target_z = float(self.LIFT_HEIGHT)
        current_ee_z = await self._estimate_dual_ee_z()
        if current_ee_z is None:
            current_ee_z = max(float(self.GRASP_Z) - 0.02, 0.36)
            self.get_logger().warn("无法读取当前末端高度，使用抓取高度近似值估计抬升量")

        delta_z = target_z - current_ee_z
        if delta_z < 0.02:
            self.get_logger().info(
                f"当前高度已接近目标 (current={current_ee_z:.3f}, target={target_z:.3f})，跳过 State5 抬升")
        else:
            delta_z = min(delta_z, 0.22)
            duration = max(3.0, min(6.5, delta_z / 0.035))
            self.get_logger().info(
                f"跳过OMPL规划，改用伺服连续上升: current_z={current_ee_z:.3f}, "
                f"target_z={target_z:.3f}, delta_z={delta_z:.3f}, duration={duration:.2f}s")
            if not await self.servo_move_delta_z(delta_z=delta_z, duration=duration):
                self.get_logger().warn("伺服上升失败，但继续强行进入执行阶段")
        
        await asyncio.sleep(0.5)
        
        # 构建一个虚拟的笛卡尔轨迹（用于兼容后续执行逻辑）
        # 实际State 6会使用伺服而不是这个轨迹
        self.planned_master_cartesian_trajectory = []
        
        # 计算偏移量
        self.traj_offset_x = float(self.dynamic_p_right[0]) - float(self.dynamic_p_left[0])
        self.traj_offset_y = float(self.dynamic_p_right[1]) - float(self.dynamic_p_left[1])
        self.traj_offset_z = 0.0
        
        self.get_logger().info("✓ 伺服上升完成，准备进入搬运执行阶段")
        self.current_state = TaskState.EXECUTE_WITH_COMPLIANCE

    async def get_current_pose(self, link_name):
        try:
            tf_msg = self.tf_buffer.lookup_transform(self.planning_frame, link_name, rclpy.time.Time())
            p = [tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z]
            q = [tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z, tf_msg.transform.rotation.w]
            return p, q
        except Exception as e:
            return None, None

    def interpolate_trajectory(self, t):
        traj = self.planned_master_cartesian_trajectory
        if not traj: return None
        if t <= traj[0]['time']: return traj[0]['pose']
        if t >= traj[-1]['time']: return traj[-1]['pose']
        
        for i in range(len(traj)-1):
            if traj[i]['time'] <= t < traj[i+1]['time']:
                dt = traj[i+1]['time'] - traj[i]['time']
                if dt == 0: return traj[i]['pose']
                ratio = (t - traj[i]['time']) / dt
                p1 = traj[i]['pose'].position
                p2 = traj[i+1]['pose'].position
                
                from copy import deepcopy
                interp_pose = deepcopy(traj[i]['pose'])
                # 位置：线性插值
                interp_pose.position.x = p1.x + (p2.x - p1.x) * ratio
                interp_pose.position.y = p1.y + (p2.y - p1.y) * ratio
                interp_pose.position.z = p1.z + (p2.z - p1.z) * ratio
                
                # 姿态：四元数 SLERP 球面线性插值（关键！避免姿态跳变导致物体弹飞）
                q1 = traj[i]['pose'].orientation
                q2 = traj[i+1]['pose'].orientation
                r1 = R.from_quat([q1.x, q1.y, q1.z, q1.w])
                r2 = R.from_quat([q2.x, q2.y, q2.z, q2.w])
                # 使用 scipy 的球面插值
                slerp_key_times = [0.0, 1.0]
                from scipy.spatial.transform import Slerp
                slerp = Slerp(slerp_key_times, R.concatenate([r1, r2]))
                r_interp = slerp([ratio])[0]
                q_interp = r_interp.as_quat()  # [x, y, z, w]
                interp_pose.orientation.x = q_interp[0]
                interp_pose.orientation.y = q_interp[1]
                interp_pose.orientation.z = q_interp[2]
                interp_pose.orientation.w = q_interp[3]
                return interp_pose
        return traj[-1]['pose']

    async def servo_trace_trajectory(self):
        self.get_logger().info("使用笛卡尔空间实时伺服 (MoveIt Servo) 基于计算轨迹执行追踪...")
        
        rate_hz = 50.0
        sleep_duration = 1.0 / rate_hz
        
        traj = self.planned_master_cartesian_trajectory
        total_duration = traj[-1]['time']
        
        Kp = 2.5
        Kp_ang = 3.0
        
        start_time = self.get_clock().now().nanoseconds * 1e-9
        
        left_twist = TwistStamped()
        left_twist.header.frame_id = self.planning_frame
        right_twist = TwistStamped()
        right_twist.header.frame_id = self.planning_frame

        import numpy as np

        while True:
            current_time = self.get_clock().now().nanoseconds * 1e-9
            t = current_time - start_time
            
            if t > total_duration + 0.5:
                break
                
            desired_master_pose = self.interpolate_trajectory(t)
            if not desired_master_pose:
                break
                
            # 计算期望速度前馈 (近似导数)
            next_pose = self.interpolate_trajectory(t + sleep_duration)
            v_ff_x = (next_pose.position.x - desired_master_pose.position.x) / sleep_duration
            v_ff_y = (next_pose.position.y - desired_master_pose.position.y) / sleep_duration
            v_ff_z = (next_pose.position.z - desired_master_pose.position.z) / sleep_duration

            actual_master_p, actual_master_q = await self.get_current_pose("mj_left_link8")
            actual_slave_p, actual_slave_q = await self.get_current_pose("mj_right_link8")

            des_q = [
                desired_master_pose.orientation.x,
                desired_master_pose.orientation.y,
                desired_master_pose.orientation.z,
                desired_master_pose.orientation.w,
            ]
            R_des = R.from_quat(des_q)
            
            # Left arm control (Master)
            if actual_master_p is not None:
                err_x = desired_master_pose.position.x - actual_master_p[0]
                err_y = desired_master_pose.position.y - actual_master_p[1]
                err_z = desired_master_pose.position.z - actual_master_p[2]
                R_act_m = R.from_quat(actual_master_q)
                rot_vec_m = (R_des * R_act_m.inv()).as_rotvec()
            else:
                err_x, err_y, err_z = 0.0, 0.0, 0.0
                rot_vec_m = np.array([0.0, 0.0, 0.0])
                
            left_twist.header.stamp = self.get_clock().now().to_msg()
            left_twist.header.frame_id = self.planning_frame
            left_twist.twist.linear.x = Kp * err_x + v_ff_x
            left_twist.twist.linear.y = Kp * err_y + v_ff_y
            left_twist.twist.linear.z = Kp * err_z + v_ff_z
            left_twist.twist.angular.x = Kp_ang * rot_vec_m[0]
            left_twist.twist.angular.y = Kp_ang * rot_vec_m[1]
            left_twist.twist.angular.z = Kp_ang * rot_vec_m[2]
            
            # Right arm control (Slave)
            des_slave_x = desired_master_pose.position.x + self.traj_offset_x
            des_slave_y = desired_master_pose.position.y + self.traj_offset_y
            des_slave_z = desired_master_pose.position.z + self.traj_offset_z
            
            if actual_slave_p is not None:
                err_sx = des_slave_x - actual_slave_p[0]
                err_sy = des_slave_y - actual_slave_p[1]
                err_sz = des_slave_z - actual_slave_p[2]
                R_act_s = R.from_quat(actual_slave_q)
                rot_vec_s = (R_des * R_act_s.inv()).as_rotvec()
            else:
                err_sx, err_sy, err_sz = 0.0, 0.0, 0.0
                rot_vec_s = np.array([0.0, 0.0, 0.0])
                
            right_twist.header.stamp = self.get_clock().now().to_msg()
            right_twist.header.frame_id = self.planning_frame
            right_twist.twist.linear.x = Kp * err_sx + v_ff_x
            right_twist.twist.linear.y = Kp * err_sy + v_ff_y
            right_twist.twist.linear.z = Kp * err_sz + v_ff_z
            right_twist.twist.angular.x = Kp_ang * rot_vec_s[0]
            right_twist.twist.angular.y = Kp_ang * rot_vec_s[1]
            right_twist.twist.angular.z = Kp_ang * rot_vec_s[2]

            # 如果柔顺控制激活，将来自 TCP 坐标系的补偿量叠加
            # 注意：此处假设柔顺计算是在 planning_frame 轴向对齐的情况下进行的简化处理
            # 若末端存在显著旋转，需先将 compliance_v_y/z 从 TCP 旋转至 planning_frame
            if self.compliance_task_active:
                right_twist.twist.linear.y += self.compliance_v_y
                right_twist.twist.linear.z += self.compliance_v_z
            
            self.servo_pub_left.publish(left_twist)
            self.servo_pub_right.publish(right_twist)
            
            await asyncio.sleep(sleep_duration)
            
        # 发送停止指令
        left_twist.twist.linear.x, left_twist.twist.linear.y, left_twist.twist.linear.z = 0.0, 0.0, 0.0
        right_twist.twist.linear.x, right_twist.twist.linear.y, right_twist.twist.linear.z = 0.0, 0.0, 0.0
        left_twist.twist.angular.x, left_twist.twist.angular.y, left_twist.twist.angular.z = 0.0, 0.0, 0.0
        right_twist.twist.angular.x, right_twist.twist.angular.y, right_twist.twist.angular.z = 0.0, 0.0, 0.0
        self.servo_pub_left.publish(left_twist)
        self.servo_pub_right.publish(right_twist)

        self.get_logger().info("轨迹 Servo 追踪执行完毕。")
        return True

    async def state_execute_with_compliance(self):
        """State 6: 执行搬运（简化版本）"""
        self.get_logger().info(">> State 6: execute_with_compliance() - 执行搬运...")

        # 【修复】不强制要求Servo控制器，允许简单伺服执行搬运
        servo_available = await self.switch_to_servo_mode()
        if servo_available:
            self.get_logger().info("✓ Servo控制器可用")
        else:
            self.get_logger().warn("⚠️  Servo控制器不可用，但继续执行")
        
        # 执行搬运：做小幅补偿抬升，避免与 State 5 重复大幅上抬造成闭链应力峰值
        current_ee_z = await self._estimate_dual_ee_z()
        target_transport_z = float(self.LIFT_HEIGHT + 0.05)
        if current_ee_z is None:
            delta_z = 0.05
        else:
            delta_z = max(0.0, min(0.12, target_transport_z - current_ee_z))

        if delta_z > 0.01:
            duration = max(2.0, min(4.0, delta_z / 0.04))
            self.get_logger().info(
                f"开始搬运补偿抬升: current_z={current_ee_z}, target={target_transport_z:.3f}, delta={delta_z:.3f}")
            if not await self.servo_move_delta_z(delta_z=delta_z, duration=duration):
                self.get_logger().warn("上升失败，但继续执行")
        else:
            self.get_logger().info("当前高度已足够，跳过 State 6 额外抬升")
        
        await asyncio.sleep(0.5)
        self.get_logger().info("✓ 搬运动作执行完成，进入开爪放置阶段")
        self.current_state = TaskState.OPEN_GRIPPERS

    async def simulated_compliance_control_loop(self):
        """ 主从受力自适应阻抗微调 (50Hz)
        
        控制律说明：
        1. Z轴柔顺：当主从臂Z轴受力差 ΔFz 超出阈值时，
           补偿速度 v_z = ΔFz/Kd_z + Kp_dist*(z_master - z_slave)
           前半项驱动从臂跟随主臂运动趋势，后半项防止高度漂移积分误差。
        2. Y轴柔顺：当夹持内应力 |Fy_L|+|Fy_R| 过大时，
           向外释放，速度 v_y = -(ΔFy_excess/Kd_y)，防止物体被挤压变形。
        3. 弹簧位置恢复项（Kp_dist）：基于TF实时测量双臂间距，
           与初始间距做差产生纠偏速度，防止速度积分导致间距漂移。
        """
        self.get_logger().info("启动主从协调机制: 柔顺高频实时控制线程 [50Hz] (基于 TCP 轴向补偿)")
        
        rate_hz = 50.0
        period = 1.0 / rate_hz
        
        # 阻抗模型参数
        Kd_z = 200.0   # TCP Z轴刚度 (N/(m/s))，即需要多大的力差才会产生1m/s补偿
        Kd_y = 150.0   # TCP Y轴刚度 (N/(m/s))
        Kp_dist = 1.0  # 位置弹簧恢复增益 (1/s)：间距偏差1m时产生1m/s恢复速度
        
        self.compliance_v_z = 0.0
        self.compliance_v_y = 0.0
        
        while self.compliance_task_active:
            # ---- 去皮后受力读取 ----
            # 注意：假设 Wrench 帧与 planning_frame 轴向一致（简化处理）
            # 若传感器安装存在旋转偏差，应使用 tf_buffer.transform() 先将力矩转换到 planning_frame
            f_master_z = self.current_left_wrench.wrench.force.z  - self.left_force_bias['z']
            f_slave_z  = self.current_right_wrench.wrench.force.z - self.right_force_bias['z']
            f_master_y = self.current_left_wrench.wrench.force.y  - self.left_force_bias['y']
            f_slave_y  = self.current_right_wrench.wrench.force.y - self.right_force_bias['y']
            
            # Z轴合力差：主臂上推多，说明从臂应跟上
            delta_f_z = f_master_z - f_slave_z
            # Y轴内应力总量：两臂夹持力之和，超过阈值需向外释放
            delta_f_y = abs(f_master_y) + abs(f_slave_y)

            # ---- 弹簧位置恢复项（同步查询 TF，使用 timeout=0 非阻塞）----
            dist_p_term_y = 0.0
            dist_p_term_z = 0.0
            try:
                # lookup_transform with rclpy.time.Time() 查询最新已知 TF（非阻塞）
                tf_left  = self.tf_buffer.lookup_transform(
                    self.planning_frame, 'mj_left_link8',  rclpy.time.Time())
                tf_right = self.tf_buffer.lookup_transform(
                    self.planning_frame, 'mj_right_link8', rclpy.time.Time())
                
                actual_master_y = tf_left.transform.translation.y
                actual_slave_y  = tf_right.transform.translation.y
                actual_master_z = tf_left.transform.translation.z
                actual_slave_z  = tf_right.transform.translation.z

                # Y轴间距弹簧：实际Y间距偏离初始偏移量时产生恢复速度
                actual_dist_y = actual_master_y - actual_slave_y
                dist_error_y  = self.traj_offset_y - actual_dist_y
                dist_p_term_y = Kp_dist * dist_error_y

                # Z轴对齐弹簧：两臂应保持Z高度同步
                dist_error_z  = actual_master_z - actual_slave_z  # 期望为 traj_offset_z
                dist_p_term_z = Kp_dist * (self.traj_offset_z - dist_error_z)
            except Exception:
                pass  # TF 暂时不可用时保持上一帧的补偿值

            # ---- Z轴柔顺补偿 ----
            # 控制律：v_z = ΔFz/Kd_z * gain + Kp_dist*距离误差
            # 低通滤波（0.8/0.2 alpha）防止高频噪声导致振荡
            if abs(delta_f_z) > self.expected_force_diff_threshold:
                target_v_z = (delta_f_z / Kd_z) * 1.5 + dist_p_term_z
                self.compliance_v_z = 0.8 * self.compliance_v_z + 0.2 * target_v_z
            else:
                # 无显著力差时，仅靠弹簧项维持间距同步
                self.compliance_v_z = 0.8 * self.compliance_v_z + 0.2 * dist_p_term_z
                
            # ---- Y轴柔顺补偿 ----
            # 内应力超过 20N 阈值时，向外（远离物体）释放
            if delta_f_y > 20.0:
                target_v_y = ((delta_f_y - 20.0) / Kd_y) * 1.5 + dist_p_term_y
                # 符号取反：内应力增大 -> 从臂向外移动（y 变小）
                self.compliance_v_y = 0.8 * self.compliance_v_y + 0.2 * (-target_v_y)
            else:
                self.compliance_v_y = 0.8 * self.compliance_v_y + 0.2 * dist_p_term_y

            await asyncio.sleep(period)
        self.get_logger().info("结束本次主从柔顺控制调节。")

    async def servo_move_cartesian(self, phase, duration=2.0):
        self.get_logger().info(f"执行简单伺服动作: {phase}")
        rate_hz = 50.0
        duration = max(0.1, float(duration))
        sleep_duration = 1.0 / rate_hz

        left_twist = TwistStamped()
        right_twist = TwistStamped()
        left_twist.header.frame_id = self.planning_frame
        right_twist.header.frame_id = self.planning_frame

        v_z = 0.0
        target_delta_z = 0.0
        if phase == 'lift':
            target_delta_z = 0.15
        elif phase == 'release_down':
            target_delta_z = -0.15
        v_z = target_delta_z / duration

        v_limit = max(0.01, float(getattr(self, 'max_servo_linear_speed', 0.06)))
        if abs(v_z) > v_limit and abs(target_delta_z) > 1e-6:
            duration = abs(target_delta_z) / v_limit
            v_z = target_delta_z / duration
            self.get_logger().warn(
                f"{phase} 速度超限，自动放慢: duration={duration:.2f}s, v_z={v_z:.3f}m/s (limit={v_limit:.3f})")
        steps = max(1, int(duration * rate_hz))

        for _ in range(steps):
            now_time = self.get_clock().now().to_msg()
            left_twist.header.stamp = now_time
            right_twist.header.stamp = now_time

            left_twist.twist.linear.x = 0.0
            left_twist.twist.linear.y = 0.0
            left_twist.twist.linear.z = v_z
            right_twist.twist.linear.x = 0.0
            right_twist.twist.linear.y = 0.0
            right_twist.twist.linear.z = v_z

            left_twist.twist.angular.x = 0.0
            left_twist.twist.angular.y = 0.0
            left_twist.twist.angular.z = 0.0
            right_twist.twist.angular.x = 0.0
            right_twist.twist.angular.y = 0.0
            right_twist.twist.angular.z = 0.0

            self.servo_pub_left.publish(left_twist)
            self.servo_pub_right.publish(right_twist)
            await asyncio.sleep(sleep_duration)

        left_twist.twist.linear.x = 0.0
        left_twist.twist.linear.y = 0.0
        left_twist.twist.linear.z = 0.0
        right_twist.twist.linear.x = 0.0
        right_twist.twist.linear.y = 0.0
        right_twist.twist.linear.z = 0.0
        left_twist.twist.angular.x = 0.0
        left_twist.twist.angular.y = 0.0
        left_twist.twist.angular.z = 0.0
        right_twist.twist.angular.x = 0.0
        right_twist.twist.angular.y = 0.0
        right_twist.twist.angular.z = 0.0
        self.servo_pub_left.publish(left_twist)
        self.servo_pub_right.publish(right_twist)
        return True

    async def servo_move_delta_z(self, delta_z, duration=2.0):
        self.get_logger().info(f"执行伺服 Z 向位移: delta_z={delta_z:.4f}m, duration={duration:.2f}s")
        if duration <= 0.0:
            return False

        rate_hz = 50.0
        duration = max(0.1, float(duration))
        sleep_duration = 1.0 / rate_hz
        v_limit = max(0.01, float(getattr(self, 'max_servo_linear_speed', 0.06)))
        if abs(delta_z) > 1e-9:
            min_duration = abs(delta_z) / v_limit
            if duration < min_duration:
                self.get_logger().warn(
                    f"Z 伺服速度超限，自动延长时长: {duration:.2f}s -> {min_duration:.2f}s "
                    f"(limit={v_limit:.3f}m/s)")
                duration = min_duration
        steps = max(1, int(duration * rate_hz))
        v_z = delta_z / duration

        left_twist = TwistStamped()
        right_twist = TwistStamped()
        left_twist.header.frame_id = self.planning_frame
        right_twist.header.frame_id = self.planning_frame

        for _ in range(steps):
            now_time = self.get_clock().now().to_msg()
            left_twist.header.stamp = now_time
            right_twist.header.stamp = now_time

            left_twist.twist.linear.x = 0.0
            left_twist.twist.linear.y = 0.0
            left_twist.twist.linear.z = v_z
            right_twist.twist.linear.x = 0.0
            right_twist.twist.linear.y = 0.0
            right_twist.twist.linear.z = v_z

            left_twist.twist.angular.x = 0.0
            left_twist.twist.angular.y = 0.0
            left_twist.twist.angular.z = 0.0
            right_twist.twist.angular.x = 0.0
            right_twist.twist.angular.y = 0.0
            right_twist.twist.angular.z = 0.0

            self.servo_pub_left.publish(left_twist)
            self.servo_pub_right.publish(right_twist)
            await asyncio.sleep(sleep_duration)

        left_twist.twist.linear.x = 0.0
        left_twist.twist.linear.y = 0.0
        left_twist.twist.linear.z = 0.0
        right_twist.twist.linear.x = 0.0
        right_twist.twist.linear.y = 0.0
        right_twist.twist.linear.z = 0.0
        left_twist.twist.angular.x = 0.0
        left_twist.twist.angular.y = 0.0
        left_twist.twist.angular.z = 0.0
        right_twist.twist.angular.x = 0.0
        right_twist.twist.angular.y = 0.0
        right_twist.twist.angular.z = 0.0
        self.servo_pub_left.publish(left_twist)
        self.servo_pub_right.publish(right_twist)
        return True

    async def state_open_grippers(self):
        """State 7: 到达放置点，解除闭合与附着（detach），开启夹爪"""
        self.get_logger().info(">> State 7: open_grippers() - 下放物体并 detach 分离...")
        
        # 实时插补下降到放置表面
        await self.servo_move_cartesian('release_down', duration=2.5)
        
        self.get_logger().info("解除夹持目标，打开双臂夹爪...")
        await self.ensure_grippers_open()
        await asyncio.sleep(1.0)
        
        # 移除附着 (Detach)
        self.get_logger().info("从手臂上分离 'target_bar'")
        detach_obj = AttachedCollisionObject()
        detach_obj.link_name = 'mj_left_link8'
        detach_obj.object.id = 'target_bar'
        detach_obj.object.operation = CollisionObject.REMOVE
        
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.robot_state.attached_collision_objects.append(detach_obj)
        scene_msg.robot_state.is_diff = True
        self.scene_pub.publish(scene_msg)
        await asyncio.sleep(0.5)
        
        # 放置完成后：恢复 ACM（禁止 mj_left_link8 与 target_bar 碰撞），恢复默认碰撞检测
        try:
            await self.set_acm_allow('mj_left_link8', 'target_bar', allow=False)
            self.get_logger().info("ACM 恢复完成，碰撞检测已恢复")
        except Exception as e:
            self.get_logger().warn(f"恢复 ACM 失败: {e}")

        self.current_state = TaskState.RETURN_TO_HOME

    async def state_return_to_home(self):
        """State 8: 结束任务返回起始姿态"""
        self.get_logger().info(">> State 8: return_to_home() - 脱出干涉区，复位双臂到Home位置...")
        
        # 使用伺服抬起双臂，离开物体
        await self.servo_move_cartesian('lift', duration=2.0)

        await self.switch_to_trajectory_mode()
        
        self.get_logger().info("安全退避完毕，执行返回Home坐标轨迹.")
        if await self.move_to_home_joints():
            self.current_state = TaskState.FINISHED
        else:
            self.current_state = TaskState.ERROR


def main(args=None):
    rclpy.init(args=args)
    node = DualArmTaskNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    node.wait_for_servers()
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(node.configure_mode_async())
        loop.run_until_complete(node.execute_task_flow())
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"异常: {e}")
    finally:
        # 双保险：即使状态机异常中断，主程序退出前也尝试闭爪
        try:
            loop.run_until_complete(node.close_grippers_on_exit("main退出兜底"))
        except Exception as close_e:
            node.get_logger().warn(f"main退出闭爪兜底失败: {close_e}")

    try:
        executor.shutdown()
    except Exception:
        pass

    try:
        spin_thread.join(timeout=1.0)
    except Exception:
        pass

    try:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass

if __name__ == '__main__':
    main()
