#!/usr/bin/env python3
import rclpy
import asyncio
import argparse
import sys
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
from task_types import TaskState
from gripper_ops import GripperOpsMixin
from servo_ops import ServoOpsMixin
from state_ops import StateOpsMixin


class DualArmTaskNode(Node, GripperOpsMixin, ServoOpsMixin, StateOpsMixin):
    def __init__(self, mode: str = 'auto'):
        super().__init__('dual_arm_task_node')
        self.get_logger().info("=== 初始化基于 ROS2+MoveIt2(OMPL+TOTG) 的双臂协作控制节点 ===")
        self.requested_mode = mode
        self.active_mode = 'auto'
        self.allow_continue_without_gripper = False
        self.gripper_wait_result = True
        self.accept_aborted_as_success = False
        self.current_state = TaskState.INIT_ENVIRONMENT
        
        # --- 超时与控制参数 (Control Parameters) ---
        self.gripper_goal_response_timeout_sec = 3.0
        self.gripper_result_timeout_sec = 6.0
        self.use_orientation_constraint = True
        self.max_effort_limit = 170.0  # 夹爪最大努力
        self.expected_force_diff_threshold = 7.5 # 主从臂测力同步最大允许偏差 (N)
        
        # --- 回调组设置 (保证 Action / Subscription 不堵塞) ---
        self.cb_group_action = ReentrantCallbackGroup()
        self.cb_group_sub = MutuallyExclusiveCallbackGroup()

        # --- 设置动作客户端 (Action Clients) ---
        self.left_arm_action_name = '/move_action'
        self.right_arm_action_name = '/move_action'
        self.left_gripper_action_name = '/mj_left_gripper_sim_node/gripper_action'
        self.right_gripper_action_name = '/mj_right_gripper_sim_node/gripper_action'
        self.dual_controller_action_name = '/dual_panda_arm_controller/follow_joint_trajectory'

        self.left_arm_client = ActionClient(self, MoveGroup, self.left_arm_action_name, callback_group=self.cb_group_action)
        self.right_arm_client = ActionClient(self, MoveGroup, self.right_arm_action_name, callback_group=self.cb_group_action)
        self.left_gripper_client = ActionClient(self, GripperCommand, self.left_gripper_action_name, callback_group=self.cb_group_action)
        self.right_gripper_client = ActionClient(self, GripperCommand, self.right_gripper_action_name, callback_group=self.cb_group_action)
        self.dual_controller_client = ActionClient(self, FollowJointTrajectory, self.dual_controller_action_name, callback_group=self.cb_group_action)

        # --- 发布者与订阅者 (Publishers & Subscribers) ---
        self.scene_pub = self.create_publisher(PlanningScene, 'planning_scene', 10)
        
        # 针对双臂分别建立 Servo 控制接口
        self.servo_pub_left = self.create_publisher(TwistStamped, '/servo_node_left/delta_twist_cmds', 10)
        self.servo_pub_right = self.create_publisher(TwistStamped, '/servo_node_right/delta_twist_cmds', 10)
        
        self.current_joint_state = {}
        # 赋予初始全0状态防报错
        self.current_left_wrench = WrenchStamped()
        self.current_right_wrench = WrenchStamped()
        # 目标位姿来源标记（用于 demo 模式下的预抓取门控策略）
        self.target_pose_from_tf = False
        self.target_pose_source_frame = 'unknown'
        self.latest_target_pose = None
        self.state2_used_fallback_pregrasp = False
        
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

        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        # Cartesian path service 用于笛卡尔直线插补（避免 OMPL 在近距离下探时产生弧线）
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
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
        self.max_servo_linear_speed = 0.045  # m/s
        self.servo_ramp_ratio = 0.36
        self.servo_phase_lift_delta = 0.10
        self.servo_phase_release_down_delta = -0.08
        self.watchdog_force_limit = 130.0   # N
        self.watchdog_force_trip_count = 3  # 连续超限计数，抑制瞬时尖峰误触发
        # 双臂笛卡尔搬运平滑参数（抑制抬升/平移阶段晃动）
        self.cooperative_lift_speed = 0.015        # m/s
        self.cooperative_transport_speed = 0.010   # m/s
        self.cooperative_cartesian_speed = 0.015   # m/s (默认)
        self.cartesian_segment_settle_sec = 0.02   # 段间短暂稳定等待
        self.lift_cartesian_settle_sec = 0.0       # 抬升阶段关闭段间停顿，避免“卡顿上升”
        self.lift_cartesian_max_segment = 0.25     # 抬升优先单段执行
        self.lift_cartesian_retry_segment = 0.08   # 单段失败时的分段重试长度
        self.grasp_descent_speed = 0.010           # m/s, 下探速度目标（过快易突降）
        self.grasp_cartesian_max_step = 0.005      # m, 下探笛卡尔插值步长
        self.grasp_cartesian_split_height = 0.030  # m, 大位移下探时插入中间航点
        self.grasp_cartesian_align_xy_first = True
        self.grasp_cartesian_waypoint_xy_tol = 0.003
        self.pregrasp_align_xy_tol = 0.003
        self.pregrasp_align_z_tol = 0.003
        self.pregrasp_align_z_gap_tol = 0.002
        self.pregrasp_refine_pos_tol = 0.004
        self.pregrasp_z_offset = 0.0                # m, 双臂共用预抓取高度偏置
        self.enforce_level_pregrasp = True          # 预抓取阶段双臂强制同高
        self.strict_pregrasp_gate = True            # 预抓取验收不允许 demo 模式强行继续
        self.pregrasp_left_z_offset = 0.0           # m, 仅在禁用同高策略时生效
        self.pregrasp_right_z_offset = 0.0          # m, 仅在禁用同高策略时生效
        self.enable_pregrasp_force_align = True   # 起始规划稳态优先，必要时再开启强制对齐
        self.pregrasp_force_align_xy_tol = 0.003
        self.pregrasp_force_align_z_gap_tol = 0.002
        self.pregrasp_force_align_pos_tol = 0.003
        self.allow_flip_orientation_candidate = False  # 默认禁用180°翻转姿态候选
        self.grasp_yaw_bias_max_deg = 2.0              # 姿态微调范围（度）
        self.grasp_site_half_span = 0.10               # m, 对齐 dual_scene.xml grasp_site ±0.10m
        self.grasp_axis_tie_eps = 1e-4                 # 抓取轴判定平票阈值（平票时强制走 local Y）
        self.pregrasp_min_lateral_spacing = 0.16       # m, 预抓取最小横向(Y)间距，防止双臂落在同一中线
        self.publish_mock_target_tf = False            # 默认不发布假 target_bar TF，避免覆盖真实 long_bar
        # IK 构型自然化：多种子求解并偏向“接近 home + 远离限位”的解
        self.enable_natural_ik_bias = True
        self.natural_ik_soft_limit_margin = 0.25       # rad, 接近限位开始惩罚
        self.natural_ik_hard_limit_margin = 0.12       # rad, 过近限位重惩罚
        self.natural_ik_j7_soft_abs = 2.00             # rad, 手腕过大扭转惩罚阈值
        self.natural_ik_current_delta_weight = 0.06    # 与当前姿态差异惩罚系数
        self.natural_ik_home_weights = [0.7, 1.0, 0.6, 1.3, 0.4, 0.5, 0.9]
        self.natural_ik_pregrasp_penalty_scale = 0.18
        self.transport_cartesian_max_step = 0.006
        self.transport_cartesian_retry_segment = 0.08
        self.transport_cartesian_one_shot_margin = 0.02
        self.transport_cartesian_settle_sec = 0.0
        # 轨迹执行平滑参数（抑制“走走停停”引发的全程晃动）
        self.dual_traj_max_joint_vel = 0.22        # rad/s
        self.dual_traj_max_joint_acc = 0.35        # rad/s^2
        self.dual_traj_vel_lpf = 0.78              # 速度一阶低通系数
        self.dual_traj_min_dt = 0.06               # s, 相邻轨迹点最小时间间隔
        self.dual_traj_start_err_tol = 0.0025      # rad, 首点偏差触发平滑引入
        self.dual_traj_start_blend_vel = 0.08      # rad/s, 首点引入段速度参考
        self.dual_traj_lead_in_min_sec = 0.25      # s
        self.dual_traj_lead_in_max_sec = 0.90      # s
        self.dual_traj_mid_blend_ratio = 0.45      # 两段式引入中点比例
        self.moveit_joint_vel_scale = 0.12
        self.moveit_joint_acc_scale = 0.07
        self.sync_plan_vel_scale = 0.12
        self.sync_plan_acc_scale = 0.07
        self.sync_plan_joint_tolerance = 0.04         # rad, 同步 plan-only 关节目标容差
        self.sync_plan_max_end_joint_error = 0.05     # rad, 轨迹末点相对 IK 目标的最大误差
        self.sync_plan_allow_collision_ik_fallback = False
        self.sync_merge_time_scale = 1.50
        # 柔顺控制阻抗参数（低晃动优先）
        self.compliance_kd_z = 340.0
        self.compliance_kd_y = 280.0
        self.compliance_kp_dist = 0.45
        self.compliance_force_y_threshold = 28.0
        self.compliance_force_gain = 0.85
        self.compliance_lpf_alpha = 0.94
        self.compliance_max_v_z = 0.020
        self.compliance_max_v_y = 0.018
        # 放置阶段参数（避免直接“丢下”）
        self.place_release_clearance = 0.012       # m, 开爪时离桌面的安全余量
        self.place_descent_speed = 0.010           # m/s, 缓降速度
        self.place_retreat_speed = 0.014           # m/s, 开爪后回抬速度
        self.release_down_default_delta = -0.08    # m, 无TF高度时保守缓降
        self.release_down_max_delta = -0.10        # m, 单次最大下放位移
        self.place_release_extra_down = 0.010      # m, 在理论放置高度上再下压一点，避免悬空开爪
        self.place_release_target_tol = 0.006      # m, 放置高度到位容差
        self.place_release_max_iters = 4           # 放置阶段最多步进次数
        self.place_release_min_total_down = 0.08   # m, 最小累计下放距离
        self.place_descent_verify_min_abs = 0.008  # m, 放置阶段位移验收最小绝对值
        self.place_descent_verify_ratio = 0.35     # 放置阶段位移验收比例阈值
        self.place_descent_retry_max = 0.06        # m, 放置阶段二次兜底最大位移
        self.post_release_lift_delta = 0.020       # m, 开爪后轻微回抬
        # 退出阶段参数（必须先上抬再回 Home）
        self.return_lift_delta = 0.10
        self.return_lift_speed = 0.020
        self.return_lift_verify_min_abs = 0.015
        self.return_lift_verify_ratio = 0.35
        self.return_lift_retry_max = 0.06
        # MuJoCo 夹爪关节是单指 0~0.04m；给 GripperCommand 传 0.08 会被 ABORTED
        self.gripper_open_position = 0.04
        self.gripper_closed_position = 0.005
        self.gripper_effort_open = 120.0
        self.gripper_effort_close = 170.0
        self.gripper_hold_interval_sec = 0.15
        # 夹取策略：默认采用受力驱动逐步夹紧；直夹仅作为显式启用的兜底模式
        self.use_direct_grasp_close = False
        self.direct_grasp_total_width = 0.036  # m, 两指总开度（直夹兜底时使用）
        self.direct_grasp_block_margin = 0.0015
        # 夹取阶段参数（中期演示优先：能夹起优先于物理真实性）
        self.grasp_contact_threshold = 0.8
        # 预压过大会把目标压入桌面，先用小位移建立接触
        self.grasp_preload_down = 0.003
        self.grasp_preload_down_min = 0.0015
        self.grasp_preload_down_max = 0.0070
        self.grasp_preload_open_boost = 0.0015
        self.grasp_preload_open_reduce = 0.0010
        self.grasp_retry_down = 0.003
        self.grasp_retry_down_min = 0.0020
        self.grasp_retry_down_max = 0.0140
        self.grasp_retry_empty_boost = 0.0040
        self.grasp_retry_contact_reduce = 0.0015
        self.grasp_preload_lift = 0.015
        self.grasp_hold_width_min = 0.008
        self.grasp_hold_width_max = 0.014
        self.grasp_object_opening_min = 0.012
        self.grasp_block_confirm_steps = 2
        self.grasp_block_margin = 0.003
        # 下探安全偏置，减少“手掌先碰铝条”
        self.grasp_z_safety_bias = 0.008
        # 在当前抓取基线上额外下探 2cm（根据实测“下降不够”问题）
        self.grasp_extra_descent = 0.020
        # 当前默认使用 MoveIt2 默认规划管线（OMPL）+ 请求适配器 TOTG。
        # 如后续确实接入 CHOMP/STOMP，请在 move_group 侧显式配置 planning_pipelines 后再追加候选。
        self.sync_plan_pipeline_candidates = ['']
        self.action_wait_poll_sec = 1.0
        self.action_wait_total_sec = 20.0
        self.service_wait_total_sec = 8.0
        self.target_tf_retry_count = 2
        self.target_tf_retry_interval_sec = 0.2


    def _joint_state_cb(self, msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_state[name] = msg.position[i]

    def _left_force_cb(self, msg: WrenchStamped):
        self.current_left_wrench = msg

    def _right_force_cb(self, msg: WrenchStamped):
        self.current_right_wrench = msg

    def _compute_safe_grasp_centers(self, center_p: np.ndarray, target_rot: R):
        """
        统一抓取中心计算：
        1) 抓取轴判定平票时强制选择 local Y；
        2) 左臂固定在 +Y 侧；
        3) 若横向间距不足，强制拉开最小 Y 间距，避免预抓取同中线碰撞。
        """
        span = float(getattr(self, 'grasp_site_half_span', 0.10))
        tie_eps = max(1e-6, float(getattr(self, 'grasp_axis_tie_eps', 1e-4)))
        min_lateral_spacing = max(0.02, float(getattr(self, 'pregrasp_min_lateral_spacing', 0.16)))

        local_x_world = target_rot.apply([1.0, 0.0, 0.0])
        local_y_world = target_rot.apply([0.0, 1.0, 0.0])
        lx_h = np.array([local_x_world[0], local_x_world[1], 0.0], dtype=float)
        ly_h = np.array([local_y_world[0], local_y_world[1], 0.0], dtype=float)
        lx_norm = float(np.linalg.norm(lx_h))
        ly_norm = float(np.linalg.norm(ly_h))

        if abs(lx_norm - ly_norm) <= tie_eps:
            half_length_vec_local = np.array([0.0, span, 0.0], dtype=float)
            axis_name = 'local Y(tie-break)'
        elif lx_norm > ly_norm:
            half_length_vec_local = np.array([span, 0.0, 0.0], dtype=float)
            axis_name = 'local X'
        else:
            half_length_vec_local = np.array([0.0, span, 0.0], dtype=float)
            axis_name = 'local Y'

        offset_vec_world = target_rot.apply(half_length_vec_local)
        p_left = np.array(center_p - offset_vec_world, dtype=float)
        p_right = np.array(center_p + offset_vec_world, dtype=float)

        # 统一左右语义：left 固定在 +Y 侧，right 固定在 -Y 侧
        if float(p_left[1]) < float(p_right[1]):
            p_left, p_right = p_right, p_left

        y_spacing_before = abs(float(p_left[1]) - float(p_right[1]))
        spacing_forced = False
        if y_spacing_before < min_lateral_spacing:
            center_y = float(center_p[1])
            half_gap = 0.5 * min_lateral_spacing
            p_left[1] = center_y + half_gap
            p_right[1] = center_y - half_gap
            spacing_forced = True

        y_spacing_after = abs(float(p_left[1]) - float(p_right[1]))
        return p_left, p_right, {
            'axis_name': axis_name,
            'span': span,
            'lx_norm': lx_norm,
            'ly_norm': ly_norm,
            'tie_eps': tie_eps,
            'spacing_forced': spacing_forced,
            'y_spacing_before': y_spacing_before,
            'y_spacing_after': y_spacing_after,
            'min_lateral_spacing': min_lateral_spacing,
        }

    def _action_status_is_success(self, status: int) -> bool:
        """统一动作状态判定：评估模式只认 SUCCEEDED，演示模式可容忍 ABORTED。"""
        try:
            status_val = int(status)
        except Exception:
            return False
        if status_val == GoalStatus.STATUS_SUCCEEDED:
            return True
        if status_val == GoalStatus.STATUS_ABORTED and bool(getattr(self, 'accept_aborted_as_success', False)):
            return True
        return False

    def _is_force_continue_enabled(self) -> bool:
        return bool(getattr(self, 'allow_continue_without_gripper', False))

    def _joint_limits(self):
        return [
            (-2.8973, 2.8973),   # J1
            (-1.7628, 1.7628),   # J2
            (-2.8973, 2.8973),   # J3
            (-3.0718, -0.0698),  # J4
            (-2.8973, 2.8973),   # J5
            (-0.0175, 3.7525),   # J6
            (-2.8973, 2.8973),   # J7
        ]

    def _extract_joint_values(self, js, joint_names):
        if js is None:
            return None
        if isinstance(js, dict):
            vals = [js.get(n, None) for n in joint_names]
            return vals if not any(v is None for v in vals) else None
        if hasattr(js, 'name') and hasattr(js, 'position'):
            name_to_pos = {n: p for n, p in zip(js.name, js.position)}
            vals = [name_to_pos.get(n, None) for n in joint_names]
            return vals if not any(v is None for v in vals) else None
        return None

    def _compute_joint_naturalness_cost(self, side: str, joint_values):
        if joint_values is None or len(joint_values) < 7:
            return 1e9

        if side == 'left':
            home = list(getattr(self, 'home_joint_positions_left', [0.0] * 7))
            arm_joint_names = list(getattr(self, 'left_arm_joints', []))
        else:
            home = list(getattr(self, 'home_joint_positions_right', [0.0] * 7))
            arm_joint_names = list(getattr(self, 'right_arm_joints', []))

        weights = list(getattr(self, 'natural_ik_home_weights', [0.7, 1.0, 0.6, 1.3, 0.4, 0.5, 0.9]))
        if len(weights) < 7:
            weights = (weights + [1.0] * 7)[:7]

        cost = 0.0
        for i in range(7):
            dv = float(joint_values[i]) - float(home[i])
            cost += float(weights[i]) * dv * dv

        limits = self._joint_limits()
        soft_margin = max(0.02, float(getattr(self, 'natural_ik_soft_limit_margin', 0.25)))
        hard_margin = max(0.01, min(soft_margin, float(getattr(self, 'natural_ik_hard_limit_margin', 0.12))))
        for i in range(7):
            lo, hi = limits[i]
            v = float(joint_values[i])
            margin = min(v - lo, hi - v)
            if margin < soft_margin:
                dm = soft_margin - margin
                cost += dm * dm * 12.0
            if margin < hard_margin:
                dm2 = hard_margin - margin
                cost += 2.5 + dm2 * dm2 * 80.0

        j7_soft = max(0.1, float(getattr(self, 'natural_ik_j7_soft_abs', 2.0)))
        j7_abs = abs(float(joint_values[6]))
        if j7_abs > j7_soft:
            dj = j7_abs - j7_soft
            cost += dj * dj * 10.0

        current_delta_w = max(0.0, float(getattr(self, 'natural_ik_current_delta_weight', 0.06)))
        if current_delta_w > 0.0 and isinstance(getattr(self, 'current_joint_state', None), dict):
            for i, jn in enumerate(arm_joint_names):
                if jn in self.current_joint_state:
                    cost += current_delta_w * abs(float(joint_values[i]) - float(self.current_joint_state[jn]))

        return float(cost)

    def _build_ik_seed_candidates(self, side: str):
        candidates = []

        if isinstance(getattr(self, 'current_joint_state', None), dict) and self.current_joint_state:
            names = list(self.current_joint_state.keys())
            positions = [float(self.current_joint_state[n]) for n in names]
            candidates.append(('current', names, positions))

        all_arm_joints = list(getattr(self, 'all_arm_joints', []))
        if all_arm_joints:
            home_map = {}
            for idx, jn in enumerate(getattr(self, 'left_arm_joints', [])):
                if idx < len(getattr(self, 'home_joint_positions_left', [])):
                    home_map[jn] = float(self.home_joint_positions_left[idx])
            for idx, jn in enumerate(getattr(self, 'right_arm_joints', [])):
                if idx < len(getattr(self, 'home_joint_positions_right', [])):
                    home_map[jn] = float(self.home_joint_positions_right[idx])

            home_positions = [float(home_map.get(jn, 0.0)) for jn in all_arm_joints]
            candidates.append(('home', all_arm_joints, home_positions))

            if isinstance(getattr(self, 'current_joint_state', None), dict) and self.current_joint_state:
                side_set = set(getattr(self, 'left_arm_joints', []) if side == 'left' else getattr(self, 'right_arm_joints', []))
                blended = []
                for jn in all_arm_joints:
                    cur = float(self.current_joint_state.get(jn, home_map.get(jn, 0.0)))
                    hm = float(home_map.get(jn, cur))
                    if jn in side_set:
                        blended.append(0.6 * cur + 0.4 * hm)
                    else:
                        blended.append(cur)
                candidates.append(('blend_side_home', all_arm_joints, blended))

        if not candidates:
            candidates.append(('default', None, None))
        return candidates

    async def compute_ik_natural_async(self, side: str, pose: PoseStamped,
                                       avoid_collisions: bool = True,
                                       allow_collision_fallback: bool = False,
                                       verbose: bool = True):
        group_name = self._group_name(side)
        joint_names = self.left_arm_joints if side == 'left' else self.right_arm_joints
        use_bias = bool(getattr(self, 'enable_natural_ik_bias', True))
        seed_candidates = self._build_ik_seed_candidates(side) if use_bias else [('default', None, None)]

        best = None
        for seed_label, seed_names, seed_positions in seed_candidates:
            ik_js = await self.compute_ik_async(
                group_name,
                pose,
                avoid_collisions=avoid_collisions,
                seed_joint_names=seed_names,
                seed_joint_positions=seed_positions,
            )
            if ik_js is None:
                continue

            vals = self._extract_joint_values(ik_js, joint_names)
            if vals is None:
                continue

            cost = self._compute_joint_naturalness_cost(side, vals) if use_bias else 0.0
            if best is None or cost < best['cost']:
                best = {
                    'js': ik_js,
                    'vals': vals,
                    'cost': float(cost),
                    'seed': seed_label,
                }

        if best is None and avoid_collisions and allow_collision_fallback:
            return await self.compute_ik_natural_async(
                side,
                pose,
                avoid_collisions=False,
                allow_collision_fallback=False,
                verbose=verbose,
            )

        if best is None:
            return None, None

        if verbose:
            self.get_logger().info(
                f"[{side}臂] IK自然择优: seed={best['seed']}, "
                f"avoid_collisions={avoid_collisions}, cost={best['cost']:.3f}")
        return best['js'], best['vals']

    async def get_target_pose_async(self, target_frame='long_bar', source_frame='base_link', fallback_frame='target_bar'):
        """
        [动态感知修复] 使用异步监听获取目标物体的精确位姿。
        取代硬编码的 BAR_CENTER_X/Y/Z。
        """
        pose = PoseStamped()
        pose.header.frame_id = source_frame
        self.target_pose_from_tf = False
        self.target_pose_source_frame = 'unknown'
        frames = [target_frame]
        if fallback_frame and fallback_frame != target_frame:
            frames.append(fallback_frame)

        retry_count = max(1, int(getattr(self, 'target_tf_retry_count', 2)))
        retry_interval = max(0.01, float(getattr(self, 'target_tf_retry_interval_sec', 0.2)))

        # 尝试获取 TF（优先 long_bar，失败再退回 target_bar）
        for frame in frames:
            self.get_logger().info(f"正在从 TF 获取 '{frame}' 位姿...")
            for _ in range(retry_count):
                try:
                    now = rclpy.time.Time()
                    trans = self.tf_buffer.lookup_transform(
                        source_frame, frame, now,
                        timeout=rclpy.duration.Duration(seconds=1.0)
                    )

                    pose.pose.position.x = trans.transform.translation.x
                    pose.pose.position.y = trans.transform.translation.y
                    pose.pose.position.z = trans.transform.translation.z
                    pose.pose.orientation = trans.transform.rotation

                    self.get_logger().info(
                        f"✓ 成功获取 '{frame}' 位姿: "
                        f"[{pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, {pose.pose.position.z:.3f}]"
                    )
                    self.target_pose_from_tf = True
                    self.target_pose_source_frame = str(frame)
                    self.latest_target_pose = pose
                    return pose
                except Exception as e:
                    self.get_logger().warn(f"TF 获取失败 ({e}), 正在重试...")
                    await asyncio.sleep(retry_interval)

        # 兜底：如果 TF 彻底不可用，返回一个合理的默认位姿
        self.get_logger().error(f"无法获取 {frames} 的 TF，使用备选硬编码位姿！")
        pose.pose.position.x = 0.5
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.36
        pose.pose.orientation.w = 1.0
        self.target_pose_from_tf = False
        self.target_pose_source_frame = 'hardcoded_fallback'
        self.latest_target_pose = pose
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
        requested = str(getattr(self, 'requested_mode', 'demo')).strip().lower()
        if requested not in ('auto', 'demo', 'eval'):
            self.get_logger().warn(f"未知模式 '{requested}'，回退到 demo 模式")
            requested = 'demo'
        if requested == 'auto':
            requested = 'demo'

        if requested == 'demo':
            self.active_mode = 'demo'
            self.allow_continue_without_gripper = True
            self.gripper_wait_result = False
            self.accept_aborted_as_success = True
            self.midterm_demo_mode = True
            self.max_servo_linear_speed = min(float(getattr(self, 'max_servo_linear_speed', 0.045)), 0.045)
            self.watchdog_force_limit = 130.0
            self.watchdog_force_trip_count = 3
            # Demo 模式：预抓取门控采用软判定，避免毫米级误差触发整条任务失败
            self.strict_pregrasp_gate = False
            self.enable_pregrasp_force_align = True
            self.pregrasp_align_xy_tol = max(float(getattr(self, 'pregrasp_align_xy_tol', 0.003)), 0.008)
            self.pregrasp_align_z_tol = max(float(getattr(self, 'pregrasp_align_z_tol', 0.003)), 0.006)
            self.pregrasp_align_z_gap_tol = max(float(getattr(self, 'pregrasp_align_z_gap_tol', 0.002)), 0.010)
            self.pregrasp_refine_pos_tol = max(float(getattr(self, 'pregrasp_refine_pos_tol', 0.004)), 0.006)
            self.pregrasp_force_align_xy_tol = max(float(getattr(self, 'pregrasp_force_align_xy_tol', 0.003)), 0.008)
            self.pregrasp_force_align_z_gap_tol = max(float(getattr(self, 'pregrasp_force_align_z_gap_tol', 0.002)), 0.010)
            self.pregrasp_force_align_pos_tol = max(float(getattr(self, 'pregrasp_force_align_pos_tol', 0.003)), 0.006)
            self.get_logger().info(
                "运行模式=demo：允许流程兜底继续；动作 ABORTED 视为软成功。")
            return

        self.active_mode = 'eval'
        self.allow_continue_without_gripper = False
        self.gripper_wait_result = True
        self.accept_aborted_as_success = False
        self.midterm_demo_mode = False
        self.max_servo_linear_speed = min(float(getattr(self, 'max_servo_linear_speed', 0.045)), 0.035)
        self.watchdog_force_limit = min(float(getattr(self, 'watchdog_force_limit', 130.0)), 100.0)
        self.watchdog_force_trip_count = min(max(int(getattr(self, 'watchdog_force_trip_count', 3)), 1), 2)
        self.strict_pregrasp_gate = True
        self.pregrasp_align_xy_tol = min(float(getattr(self, 'pregrasp_align_xy_tol', 0.008)), 0.004)
        self.pregrasp_align_z_tol = min(float(getattr(self, 'pregrasp_align_z_tol', 0.006)), 0.004)
        self.pregrasp_align_z_gap_tol = min(float(getattr(self, 'pregrasp_align_z_gap_tol', 0.010)), 0.004)
        self.pregrasp_force_align_xy_tol = min(float(getattr(self, 'pregrasp_force_align_xy_tol', 0.008)), 0.004)
        self.pregrasp_force_align_z_gap_tol = min(float(getattr(self, 'pregrasp_force_align_z_gap_tol', 0.010)), 0.004)
        self.get_logger().info(
            "运行模式=eval：严格判定流程；动作仅 SUCCEEDED 才算成功。")

    def wait_for_servers(self):
        self.get_logger().info('等待动作服务器 (Action Servers)...')
        poll = max(0.2, float(getattr(self, 'action_wait_poll_sec', 1.0)))
        total = max(poll, float(getattr(self, 'action_wait_total_sec', 20.0)))
        max_rounds = max(1, int(total / poll))

        mandatory_clients = [
            ('Left Arm', self.left_arm_client, self.left_arm_action_name),
            ('Right Arm', self.right_arm_client, self.right_arm_action_name),
            ('Left Gripper', self.left_gripper_client, self.left_gripper_action_name),
            ('Right Gripper', self.right_gripper_client, self.right_gripper_action_name),
        ]
        optional_clients = [
            ('Dual Controller', self.dual_controller_client, self.dual_controller_action_name),
        ]

        status = {}
        for round_idx in range(1, max_rounds + 1):
            pending = []
            for name, client, action_name in mandatory_clients + optional_clients:
                if status.get(name, False):
                    continue
                if client.wait_for_server(timeout_sec=poll):
                    status[name] = True
                    self.get_logger().info(f"[连接成功] {name}: {action_name}")
                else:
                    status[name] = False
                    pending.append((name, action_name))

            if not pending:
                break
            if round_idx % 5 == 0 or round_idx == max_rounds:
                pending_str = ", ".join([f"{n}({a})" for n, a in pending])
                self.get_logger().warn(
                    f"动作服务器等待中 {round_idx}/{max_rounds}: 仍未就绪 -> {pending_str}")

        for name, _, action_name in mandatory_clients:
            if not status.get(name, False):
                self.get_logger().warn(
                    f"{name} 动作服务器不可用: {action_name}，但为了中期演示将强行忽略！")

        self.dual_controller_available = bool(status.get('Dual Controller', False))
        if not self.dual_controller_available:
            self.get_logger().warn(
                f"Dual Controller 不可用: {self.dual_controller_action_name}，将使用顺序/伺服兜底")

        # 额外检查关键服务，便于定位“服务器连不上”的根因
        svc_total = max(1.0, float(getattr(self, 'service_wait_total_sec', 8.0)))
        critical_services = [
            ('IK', self.ik_client, '/compute_ik'),
            ('FK', self.fk_client, '/compute_fk'),
            ('CartesianPath', self.cartesian_client, '/compute_cartesian_path'),
        ]
        for name, cli, srv_name in critical_services:
            if not cli.wait_for_service(timeout_sec=svc_total):
                self.get_logger().warn(f"{name} 服务不可用: {srv_name}")

        # 自动探测实际规划组名，修正 group_name 与 SRDF 不匹配的问题
        self._probe_planning_groups()
        self.get_logger().info(
            "若持续连接失败，请先检查: `ros2 action list -t` 与 `ros2 service list | rg compute_`")
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

    async def compute_ik_async(self, group_name: str, pose: PoseStamped, avoid_collisions: bool = True,
                               seed_joint_names=None, seed_joint_positions=None):
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
        req.ik_request.timeout.sec = 3

        # 优先使用显式传入的 IK 种子（例如 home/current blend），
        # 否则退回当前关节状态作为默认种子。
        if (
            seed_joint_names is not None and seed_joint_positions is not None and
            len(seed_joint_names) == len(seed_joint_positions) and len(seed_joint_names) > 0
        ):
            req.ik_request.robot_state.is_diff = False
            req.ik_request.robot_state.joint_state.name = [str(n) for n in seed_joint_names]
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed_joint_positions]
        elif self.current_joint_state:
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
        goal.request.max_velocity_scaling_factor = float(getattr(self, 'moveit_joint_vel_scale', 0.12))
        goal.request.max_acceleration_scaling_factor = float(getattr(self, 'moveit_joint_acc_scale', 0.07))
        
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

        # ── Step 1: IK 求解（多种子 + 自然构型择优）──
        allow_collision_fallback = bool(max_retries > 2)
        ik_solution, extracted_vals = await self.compute_ik_natural_async(
            side,
            pose,
            avoid_collisions=True,
            allow_collision_fallback=allow_collision_fallback,
        )
        if not ik_solution or not extracted_vals:
            self.get_logger().error(f"[{side}臂] IK 全部失败，目标位姿不可达 ✗")
            return False

        # 从 IK 结果里提取本臂的 7 个关节角
        target_positions = [float(v) for v in extracted_vals]

        # ── Step 2: 构造 JointConstraint 目标并规划 ──
        for attempt in range(1, max_retries + 1):
            goal = MoveGroup.Goal()
            goal.request.group_name = group_name
            goal.request.num_planning_attempts = 20
            goal.request.allowed_planning_time = 15.0
            goal.request.max_velocity_scaling_factor = float(getattr(self, 'moveit_joint_vel_scale', 0.12))
            goal.request.max_acceleration_scaling_factor = float(getattr(self, 'moveit_joint_acc_scale', 0.07))

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
                merge_time_scale = max(1.0, float(getattr(self, 'sync_merge_time_scale', 1.50)))
                return await self._execute_merged_trajectory(left_traj, right_traj, time_scale=merge_time_scale)

            missing = []
            if left_traj is None:
                missing.append('left')
            if right_traj is None:
                missing.append('right')
            self.get_logger().warn(
                f"同步规划失败（失败侧={','.join(missing) if missing else 'unknown'}），降级为顺序移动")
        
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
        min_dt = max(0.01, float(getattr(self, 'dual_traj_min_dt', 0.06)))
        if t_unified:
            t_retimed = [0.0]
            for i in range(1, len(t_unified)):
                t_retimed.append(max(float(t_unified[i]), t_retimed[-1] + min_dt))
            t_unified = t_retimed

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
            set_dur(pt, t)
            merged.points.append(pt)

        # 起始状态对齐：若轨迹首点与当前关节状态差异较大，插入“两段式”平滑引入
        try:
            current_positions = [self.current_joint_state.get(jn, None) for jn in self.all_arm_joints]
            if merged.points and all(v is not None for v in current_positions):
                first_pos = merged.points[0].positions
                max_start_err = max(
                    abs(float(first_pos[idx]) - float(current_positions[idx]))
                    for idx in range(len(self.all_arm_joints))
                )
                start_err_tol = max(0.001, float(getattr(self, 'dual_traj_start_err_tol', 0.0025)))
                if max_start_err > start_err_tol:
                    blend_vel = max(0.03, float(getattr(self, 'dual_traj_start_blend_vel', 0.08)))
                    lead_in_min = max(min_dt * 2.0, float(getattr(self, 'dual_traj_lead_in_min_sec', 0.25)))
                    lead_in_max = max(lead_in_min + min_dt, float(getattr(self, 'dual_traj_lead_in_max_sec', 0.90)))
                    lead_in = max(lead_in_min, min(lead_in_max, max_start_err / blend_vel))
                    mid_ratio = max(0.20, min(0.80, float(getattr(self, 'dual_traj_mid_blend_ratio', 0.45))))
                    mid_t = max(min_dt, lead_in * mid_ratio)

                    for point in merged.points:
                        set_dur(point, get_dur_sec(point) + lead_in)

                    mid_pt = JointTrajectoryPoint()
                    mid_pt.positions = [
                        float(cur + mid_ratio * (tar - cur))
                        for cur, tar in zip(current_positions, first_pos)
                    ]
                    set_dur(mid_pt, mid_t)

                    start_pt = JointTrajectoryPoint()
                    start_pt.positions = [float(v) for v in current_positions]
                    set_dur(start_pt, 0.0)
                    merged.points.insert(0, mid_pt)
                    merged.points.insert(0, start_pt)

                    self.get_logger().info(
                        f"合并轨迹首点偏差 {max_start_err:.4f}rad，插入两段式引入 lead_in={lead_in:.2f}s")
        except Exception:
            pass

        # 引入段插入后再次强制时间单调且满足最小间隔，避免时间过密导致“瞬时加速”
        if merged.points:
            t_retimed = [0.0]
            for i in range(1, len(merged.points)):
                t_raw = get_dur_sec(merged.points[i])
                t_retimed.append(max(float(t_raw), t_retimed[-1] + min_dt))
            for i, point in enumerate(merged.points):
                set_dur(point, t_retimed[i])

        t_unified = [get_dur_sec(pt) for pt in merged.points]

        # 关键修复：不要给每个航点都塞 0 速度（会导致控制器“频繁刹停-再启动”）
        # 改为根据相邻点差分估计速度并限幅，再做低通平滑，显著降低搬运抖动。
        n_pts = len(merged.points)
        n_joints = len(self.all_arm_joints)
        max_joint_vel = max(0.05, float(getattr(self, 'dual_traj_max_joint_vel', 0.22)))
        max_joint_acc = max(0.05, float(getattr(self, 'dual_traj_max_joint_acc', 0.35)))
        vel_alpha = max(0.0, min(0.98, float(getattr(self, 'dual_traj_vel_lpf', 0.78))))
        prev_vel = [0.0] * n_joints
        for i in range(n_pts):
            if n_pts <= 1:
                raw = [0.0] * n_joints
            elif i == 0:
                raw = [0.0] * n_joints
            elif i == n_pts - 1:
                raw = [0.0] * n_joints
            else:
                dt = max(1e-3, t_unified[i + 1] - t_unified[i - 1])
                raw = [
                    (merged.points[i + 1].positions[j] - merged.points[i - 1].positions[j]) / dt
                    for j in range(n_joints)
                ]

            vel = [vel_alpha * prev_vel[j] + (1.0 - vel_alpha) * raw[j] for j in range(n_joints)]
            vel = [max(-max_joint_vel, min(max_joint_vel, float(v))) for v in vel]

            if i > 0:
                dt_prev = max(1e-3, t_unified[i] - t_unified[i - 1])
                max_dv = max_joint_acc * dt_prev
                vel = [
                    prev_vel[j] + max(-max_dv, min(max_dv, vel[j] - prev_vel[j]))
                    for j in range(n_joints)
                ]

            merged.points[i].velocities = vel
            prev_vel = vel

        if merged.points:
            merged.points[0].velocities = [0.0] * n_joints
            merged.points[-1].velocities = [0.0] * n_joints

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
            exec_timeout = max(30.0, float(t_unified[-1]) + 20.0)
            result = await asyncio.wait_for(gh.get_result_async(), timeout=exec_timeout)
            ok = self._action_status_is_success(result.status)
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
            return self._action_status_is_success(result.status)
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
        goal.request.max_velocity_scaling_factor = float(getattr(self, 'moveit_joint_vel_scale', 0.16))
        goal.request.max_acceleration_scaling_factor = float(getattr(self, 'moveit_joint_acc_scale', 0.10))
        
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

        # 先求 IK（多种子 + 自然构型择优）
        allow_collision_fallback = bool(getattr(self, 'sync_plan_allow_collision_ik_fallback', False))
        target_js, extracted_vals = await self.compute_ik_natural_async(
            side,
            pose,
            avoid_collisions=True,
            allow_collision_fallback=allow_collision_fallback,
        )
        if not target_js or not extracted_vals:
            self.get_logger().warn(f"[{side}] IK 求解失败，无法规划")
            return None

        # 提取对应关节的数值
        target_positions = [float(v) for v in extracted_vals]

        from moveit_msgs.msg import JointConstraint
        pipeline_candidates = getattr(
            self, 'sync_plan_pipeline_candidates', [''])

        error_code_map = {
            1: 'SUCCESS',
            -1: 'FAILURE',
            -2: 'PLANNING_FAILED',
            -3: 'INVALID_MOTION_PLAN',
            -4: 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
            -5: 'CONTROL_FAILED',
            -6: 'UNABLE_TO_AQUIRE_SENSOR_DATA',
            -7: 'TIMED_OUT',
            -8: 'PREEMPTED',
            -9: 'START_STATE_IN_COLLISION',
            -10: 'START_STATE_VIOLATES_PATH_CONSTRAINTS',
            -11: 'GOAL_IN_COLLISION',
            -12: 'GOAL_VIOLATES_PATH_CONSTRAINTS',
            -13: 'GOAL_CONSTRAINTS_VIOLATED',
            -14: 'INVALID_GROUP_NAME',
            -15: 'INVALID_GOAL_CONSTRAINTS',
            -16: 'INVALID_ROBOT_STATE',
            -17: 'INVALID_LINK_NAME',
            -18: 'INVALID_OBJECT_NAME',
            -19: 'FRAME_TRANSFORM_FAILURE',
            -21: 'COLLISION_CHECKING_UNAVAILABLE',
            -22: 'ROBOT_STATE_STALE',
            -23: 'SENSOR_INFO_STALE',
            -24: 'NO_IK_SOLUTION',
        }

        for pipeline_id in pipeline_candidates:
            pipe_name = pipeline_id if pipeline_id else '<default>'
            goal = MoveGroup.Goal()
            goal.request.group_name = group_name
            goal.request.num_planning_attempts = 10
            goal.request.allowed_planning_time = 10.0
            goal.request.max_velocity_scaling_factor = float(getattr(self, 'sync_plan_vel_scale', getattr(self, 'moveit_joint_vel_scale', 0.12)))
            goal.request.max_acceleration_scaling_factor = float(getattr(self, 'sync_plan_acc_scale', getattr(self, 'moveit_joint_acc_scale', 0.07)))
            if pipeline_id:
                goal.request.pipeline_id = pipeline_id
            goal.planning_options.plan_only = True

            constraints = Constraints()
            constraints.name = "joint_goal"
            joint_tol = max(0.005, float(getattr(self, 'sync_plan_joint_tolerance', 0.04)))
            for i, jn in enumerate(joint_names):
                jc = JointConstraint()
                jc.joint_name = jn
                jc.position = target_positions[i]
                jc.tolerance_above = joint_tol
                jc.tolerance_below = joint_tol
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)
            goal.request.goal_constraints.append(constraints)

            self.get_logger().info(
                f"[{side}] 同步规划尝试: pipeline={pipe_name}, group={group_name}")
            try:
                goal_handle = await asyncio.wait_for(client.send_goal_async(goal), timeout=10.0)
            except asyncio.TimeoutError:
                self.get_logger().warn(f"[{side}] pipeline={pipe_name} 发送 plan_only 目标超时")
                continue
            except Exception as e:
                self.get_logger().warn(f"[{side}] pipeline={pipe_name} 发送 plan_only 异常: {e}")
                continue

            if not goal_handle.accepted:
                self.get_logger().warn(f"[{side}] pipeline={pipe_name} 目标被拒绝")
                continue

            try:
                result = await asyncio.wait_for(goal_handle.get_result_async(), timeout=25.0)
            except asyncio.TimeoutError:
                self.get_logger().warn(f"[{side}] pipeline={pipe_name} 等待结果超时")
                continue
            except Exception as e:
                self.get_logger().warn(f"[{side}] pipeline={pipe_name} 获取结果异常: {e}")
                continue

            status = getattr(result, 'status', -999)
            err_val = getattr(getattr(result.result, 'error_code', None), 'val', -999)
            err_name = error_code_map.get(err_val, f'UNKNOWN({err_val})')
            traj = getattr(getattr(result.result, 'planned_trajectory', None), 'joint_trajectory', None)
            point_count = 0 if traj is None else len(getattr(traj, 'points', []))

            if err_val == 1 and point_count > 0:
                try:
                    end_positions = list(traj.points[-1].positions)
                    if len(end_positions) == len(target_positions):
                        max_end_err = max(
                            abs(float(a) - float(b))
                            for a, b in zip(end_positions, target_positions)
                        )
                        max_allowed = max(
                            float(getattr(self, 'sync_plan_joint_tolerance', 0.04)),
                            float(getattr(self, 'sync_plan_max_end_joint_error', 0.05)),
                        )
                        if max_end_err > max_allowed:
                            self.get_logger().warn(
                                f"[{side}] pipeline={pipe_name} 末点偏差过大: "
                                f"max_end_err={max_end_err:.4f}rad > {max_allowed:.4f}rad，丢弃该轨迹")
                            continue
                except Exception:
                    pass
                self.get_logger().info(
                    f"[{side}] pipeline={pipe_name} 同步规划成功: status={status}, points={point_count}")
                return traj

            self.get_logger().warn(
                f"[{side}] pipeline={pipe_name} 同步规划失败: status={status}, "
                f"error_code={err_val}({err_name}), points={point_count}")

        self.get_logger().warn(f"[{side}] 所有同步规划 pipeline 均失败，返回顺序规划降级路径")
        return None

    async def sync_move_arms_coordinated(self, left_pose, right_pose):
        """
        第四阶段：闭链近似主从规划（当前基于 OMPL 规划输出）
        这里执行主从跟踪算法：
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
        return self._action_status_is_success(result.status)


    async def execute_task_flow(self):
        self.get_logger().info(
            f"=== 开始双臂协同搬运任务 (mode={self.active_mode}, ROS2 + MoveIt2(OMPL+TOTG) + 柔顺控制) ===")

        # 等待关节状态初始化
        for _ in range(30):
            if len(self.current_joint_state) >= 14: break
            await asyncio.sleep(0.1)

        # [第一阶段改造] 动态感知设置
        self.planning_frame = "base_link"

        # 若未启用视觉节点，可临时发布 mock TF；默认关闭以避免覆盖真实 long_bar 姿态
        if bool(getattr(self, 'publish_mock_target_tf', False)):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.planning_frame
            t.child_frame_id = 'target_bar'
            t.transform.translation.x = 0.5
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.36
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)
            self.get_logger().warn(
                f"已发布 mock 'target_bar' 静态 TF (父坐标系: {self.planning_frame})。")
            await asyncio.sleep(0.5)

        target_pose = await self.get_target_pose_async()
        if target_pose is None:
            self.get_logger().error("无法获取目标物体位姿，任务终止！")
            self.current_state = TaskState.ERROR
            # 在进入主循环前就直接返回
            return
        self.latest_target_pose = target_pose
        
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

        # 与 dual_scene.xml 抓取 site 对齐，并加入防撞保护（平票走Y + 最小横向间距）
        center_p = np.array([
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            target_pose.pose.position.z
        ])
        left_grasp_center, right_grasp_center, grasp_meta = self._compute_safe_grasp_centers(center_p, target_rot)
        self.get_logger().info(
            f"抓取轴判定={grasp_meta['axis_name']} "
            f"(||lx_h||={grasp_meta['lx_norm']:.4f}, ||ly_h||={grasp_meta['ly_norm']:.4f}, eps={grasp_meta['tie_eps']:.1e}), "
            f"Y间距={grasp_meta['y_spacing_after']:.4f}m")
        if grasp_meta['spacing_forced']:
            self.get_logger().warn(
                f"预抓取防撞保护触发: Y间距从 {grasp_meta['y_spacing_before']:.4f}m "
                f"提升至 {grasp_meta['y_spacing_after']:.4f}m (最小要求 {grasp_meta['min_lateral_spacing']:.4f}m)")
        
        # 为了后续状态机使用，将numpy数组存为成员变量
        self.left_grasp_center_pos = left_grasp_center
        self.right_grasp_center_pos = right_grasp_center
        self.target_object_orientation = target_pose.pose.orientation # 保存姿态给后续使用

        # === 高度计算（从 dual_scene.xml + mj_dual.xml 精确推导）===
        # 桌子: pos="0.5 0 0.2", size半高=0.14 → 桌面 z = 0.2+0.14 = 0.34m
        # 铝条静止后中心 z = 桌面(0.34) + 半高(0.02) = 0.36m
        self.TABLE_TOP_Z = 0.34
        self.BAR_RESTING_Z = target_pose.pose.position.z # 使用动态获取的高度

        if bool(getattr(self, 'publish_mock_target_tf', False)):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.planning_frame
            t.child_frame_id = 'target_bar'
            t.transform.translation.x = float(target_pose.pose.position.x)
            t.transform.translation.y = float(target_pose.pose.position.y)
            t.transform.translation.z = float(self.BAR_RESTING_Z)
            t.transform.rotation = target_pose.pose.orientation
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
        self.TRANSPORT_Y_OFFSET = 0.00
        self.TRANSPORT_Z_OFFSET = 0.00

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


def main(args=None):
    cli_args = list(args) if args is not None else sys.argv[1:]
    parser = argparse.ArgumentParser(description='Dual arm cooperative transport demo/eval runner.')
    parser.add_argument(
        '--mode',
        choices=['auto', 'demo', 'eval'],
        default='demo',
        help='demo: 中期演示容错模式; eval: 严格评估模式'
    )
    parsed, ros_args = parser.parse_known_args(cli_args)

    rclpy.init(args=ros_args)
    node = DualArmTaskNode(mode=parsed.mode)
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
