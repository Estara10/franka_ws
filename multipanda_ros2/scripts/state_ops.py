#!/usr/bin/env python3

#状态机状态操作函数实现
import asyncio

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from scipy.spatial.transform import Rotation as R
from shape_msgs.msg import SolidPrimitive

from task_types import TaskState


class StateOpsMixin:
    async def _move_dual_joints_direct(self, target_positions, duration: float = 3.0,
                                       description: str = "双臂关节轨迹兜底") -> bool:
        """
        直接通过 dual_panda_arm_controller 下发 14 关节目标，绕过 IK/MoveIt 规划。
        """
        if not bool(getattr(self, 'dual_controller_available', False)):
            self.get_logger().warn(f"{description}: dual_controller 不可用")
            return False

        client = getattr(self, 'dual_controller_client', None)
        if client is None:
            self.get_logger().warn(f"{description}: 未找到 dual_controller_client")
            return False
        if not client.server_is_ready() and not client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn(f"{description}: dual_controller action server 未就绪")
            return False

        n_joints = len(getattr(self, 'all_arm_joints', []))
        if n_joints <= 0 or target_positions is None or len(target_positions) != n_joints:
            self.get_logger().warn(
                f"{description}: 目标关节长度非法 len={0 if target_positions is None else len(target_positions)}, expected={n_joints}")
            return False

        try:
            from builtin_interfaces.msg import Duration
            from control_msgs.action import FollowJointTrajectory
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        except Exception as e:
            self.get_logger().warn(f"{description}: 导入轨迹消息失败: {e}")
            return False

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(self.all_arm_joints)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in target_positions]
        pt.velocities = [0.0] * n_joints
        duration = max(1.5, float(duration))
        pt.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration - int(duration)) * 1e9),
        )
        traj.points = [pt]
        goal.trajectory = traj

        self.get_logger().info(f"{description}: 发送14关节直接轨迹, duration={duration:.2f}s")
        try:
            goal_handle = await asyncio.wait_for(client.send_goal_async(goal), timeout=5.0)
            if not goal_handle.accepted:
                self.get_logger().warn(f"{description}: 目标被拒绝")
                return False
            result = await asyncio.wait_for(goal_handle.get_result_async(), timeout=duration + 12.0)
            status = int(getattr(result, 'status', -1))
            checker = getattr(self, '_action_status_is_success', None)
            if callable(checker):
                ok = bool(checker(status))
            else:
                ok = (status == 4)
            self.get_logger().info(f"{description}: 执行{'成功' if ok else '失败'}(status={result.status})")
            return ok
        except Exception as e:
            self.get_logger().warn(f"{description}: 执行异常: {e}")
            return False

    async def _dual_delta_z_fallback(self, delta_z: float, description: str = "双臂Z向位移兜底") -> bool:
        """
        当 Servo 控制器不可用时，使用“当前位姿 + IK + 轨迹执行”完成双臂同步升降。
        """
        # 优先回放“预抓取成功”时的关节快照，避免 IK 在持物状态下失败
        if delta_z > 0.0:
            snapshot = getattr(self, 'pre_grasp_joint_snapshot', None)
            if snapshot is not None and len(snapshot) == len(getattr(self, 'all_arm_joints', [])):
                ok = await self._move_dual_joints_direct(
                    snapshot,
                    duration=max(2.5, min(4.5, abs(delta_z) / 0.03)),
                    description=f"{description}-预抓取快照回放",
                )
                if ok:
                    return True

        left_p, left_q = await self.get_current_pose('mj_left_link8')
        right_p, right_q = await self.get_current_pose('mj_right_link8')
        if left_p is None or right_p is None or left_q is None or right_q is None:
            self.get_logger().warn(f"{description}: 无法读取当前末端位姿，兜底失败")
            return False

        left_target = PoseStamped()
        left_target.header.frame_id = self.planning_frame
        left_target.pose.position.x = float(left_p[0])
        left_target.pose.position.y = float(left_p[1])
        left_target_z = float(left_p[2] + delta_z)
        if delta_z > 0.0:
            pre_z = float(getattr(self, 'PRE_GRASP_HEIGHT', left_target_z)) + float(
                getattr(self, 'pregrasp_left_z_offset', 0.0)
            )
            left_target_z = min(left_target_z, pre_z + 0.01)
        left_target.pose.position.z = left_target_z
        left_target.pose.orientation = Quaternion(
            x=float(left_q[0]), y=float(left_q[1]), z=float(left_q[2]), w=float(left_q[3]))

        right_target = PoseStamped()
        right_target.header.frame_id = self.planning_frame
        right_target.pose.position.x = float(right_p[0])
        right_target.pose.position.y = float(right_p[1])
        right_target_z = float(right_p[2] + delta_z)
        if delta_z > 0.0:
            pre_z = float(getattr(self, 'PRE_GRASP_HEIGHT', right_target_z)) + float(
                getattr(self, 'pregrasp_right_z_offset', 0.0)
            )
            right_target_z = min(right_target_z, pre_z + 0.01)
        right_target.pose.position.z = right_target_z
        right_target.pose.orientation = Quaternion(
            x=float(right_q[0]), y=float(right_q[1]), z=float(right_q[2]), w=float(right_q[3]))

        self.get_logger().info(
            f"{description}: 使用 IK+轨迹同步执行 delta_z={delta_z:.3f}m")
        return await self.sync_move_arms(
            left_target, right_target,
            pos_tol=0.02, ori_tol_x=0.10, ori_tol_y=0.10, ori_tol_z=0.15, max_retries=2)

    async def _move_dual_cartesian_delta(self, delta_x: float = 0.0, delta_y: float = 0.0, delta_z: float = 0.0,
                                         description: str = "双臂笛卡尔位移", max_step: float = 0.008,
                                         avoid_collisions: bool = False, max_segment: float = 0.06,
                                         target_speed: float = None, settle_sec: float = None) -> bool:
        """
        根治方案：使用 MoveIt CartesianPath 服务生成双臂直线轨迹，再交给 dual_controller 执行。
        支持 dx/dy/dz 任意组合，并自动分段避免一次位移过长导致轨迹失败。
        """
        if not bool(getattr(self, 'dual_controller_available', False)):
            self.get_logger().warn(f"{description}: dual_controller 不可用")
            return False
        if self.cartesian_client is None or not self.cartesian_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"{description}: /compute_cartesian_path 服务不可用")
            return False

        total = np.array([float(delta_x), float(delta_y), float(delta_z)], dtype=float)
        if np.linalg.norm(total) < 1e-6:
            self.get_logger().info(f"{description}: 位移近似为 0，直接返回成功")
            return True

        from moveit_msgs.srv import GetCartesianPath as GCP

        max_axis_delta = max(abs(float(delta_x)), abs(float(delta_y)), abs(float(delta_z)))
        max_segment = max(0.01, float(max_segment))
        seg_count = max(1, int(np.ceil(max_axis_delta / max_segment)))
        step = total / float(seg_count)
        if target_speed is None:
            target_speed = float(getattr(self, 'cooperative_cartesian_speed', 0.025))
        target_speed = max(0.005, float(target_speed))
        if settle_sec is None:
            settle_sec = float(getattr(self, 'cartesian_segment_settle_sec', 0.04))
        settle_sec = max(0.0, float(settle_sec))
        self.get_logger().info(
            f"{description}: 分段执行 {seg_count} 段, 单段位移="
            f"({step[0]:.4f},{step[1]:.4f},{step[2]:.4f})m")

        for seg_idx in range(seg_count):
            left_p, left_q = await self.get_current_pose('mj_left_link8')
            right_p, right_q = await self.get_current_pose('mj_right_link8')
            if left_p is None or right_p is None or left_q is None or right_q is None:
                self.get_logger().warn(f"{description}: 无法读取当前末端位姿 (segment={seg_idx + 1}/{seg_count})")
                return False

            left_start = PoseStamped()
            left_start.header.frame_id = self.planning_frame
            left_start.pose.position.x = float(left_p[0])
            left_start.pose.position.y = float(left_p[1])
            left_start.pose.position.z = float(left_p[2])
            left_start.pose.orientation = Quaternion(
                x=float(left_q[0]), y=float(left_q[1]), z=float(left_q[2]), w=float(left_q[3]))

            left_goal = PoseStamped()
            left_goal.header.frame_id = self.planning_frame
            left_goal.pose.position.x = float(left_p[0] + step[0])
            left_goal.pose.position.y = float(left_p[1] + step[1])
            left_goal.pose.position.z = float(left_p[2] + step[2])
            left_goal.pose.orientation = left_start.pose.orientation

            right_start = PoseStamped()
            right_start.header.frame_id = self.planning_frame
            right_start.pose.position.x = float(right_p[0])
            right_start.pose.position.y = float(right_p[1])
            right_start.pose.position.z = float(right_p[2])
            right_start.pose.orientation = Quaternion(
                x=float(right_q[0]), y=float(right_q[1]), z=float(right_q[2]), w=float(right_q[3]))

            right_goal = PoseStamped()
            right_goal.header.frame_id = self.planning_frame
            right_goal.pose.position.x = float(right_p[0] + step[0])
            right_goal.pose.position.y = float(right_p[1] + step[1])
            right_goal.pose.position.z = float(right_p[2] + step[2])
            right_goal.pose.orientation = right_start.pose.orientation

            try:
                req_l = GCP.Request()
                req_l.group_name = self._group_name('left')
                req_l.waypoints = [left_start.pose, left_goal.pose]
                req_l.max_step = float(max_step)
                req_l.jump_threshold = 0.0
                req_l.avoid_collisions = bool(avoid_collisions)
                res_l = await asyncio.wait_for(self.cartesian_client.call_async(req_l), timeout=10.0)

                req_r = GCP.Request()
                req_r.group_name = self._group_name('right')
                req_r.waypoints = [right_start.pose, right_goal.pose]
                req_r.max_step = float(max_step)
                req_r.jump_threshold = 0.0
                req_r.avoid_collisions = bool(avoid_collisions)
                res_r = await asyncio.wait_for(self.cartesian_client.call_async(req_r), timeout=10.0)
            except Exception as e:
                self.get_logger().warn(f"{description}: CartesianPath 服务调用失败(segment={seg_idx + 1}/{seg_count}): {e}")
                return False

            frac_l = float(getattr(res_l, 'fraction', 0.0))
            frac_r = float(getattr(res_r, 'fraction', 0.0))
            left_traj = getattr(getattr(res_l, 'solution', None), 'joint_trajectory', None)
            right_traj = getattr(getattr(res_r, 'solution', None), 'joint_trajectory', None)
            if frac_l < 0.98 or frac_r < 0.98 or left_traj is None or right_traj is None:
                self.get_logger().warn(
                    f"{description}: 轨迹质量不足(segment={seg_idx + 1}/{seg_count}) "
                    f"frac_l={frac_l:.3f}, frac_r={frac_r:.3f}")
                return False

            left_last_t = 0.0
            right_last_t = 0.0
            if left_traj.points:
                lp = left_traj.points[-1].time_from_start
                left_last_t = float(lp.sec) + float(lp.nanosec) * 1e-9
            if right_traj.points:
                rp = right_traj.points[-1].time_from_start
                right_last_t = float(rp.sec) + float(rp.nanosec) * 1e-9
            base_duration = max(0.1, left_last_t, right_last_t)
            segment_dist = float(np.linalg.norm(step))
            desired_duration = max(0.8, segment_dist / target_speed)
            time_scale = max(1.0, min(8.0, desired_duration / base_duration))

            ok = await self._execute_merged_trajectory(left_traj, right_traj, time_scale=time_scale)
            if not ok:
                self.get_logger().warn(f"{description}: 合并轨迹执行失败(segment={seg_idx + 1}/{seg_count})")
                return False
            if settle_sec > 1e-4:
                await asyncio.sleep(settle_sec)

        self.get_logger().info(
            f"{description}: 执行完成, 总位移=({total[0]:.4f},{total[1]:.4f},{total[2]:.4f})m")
        return True

    async def _move_dual_cartesian_delta_z(self, delta_z: float, description: str = "双臂笛卡尔升降",
                                           max_step: float = 0.008, max_segment: float = None,
                                           target_speed: float = None, settle_sec: float = None) -> bool:
        """Z向笛卡尔位移：优先单段连续执行，失败后回退分段执行。"""
        delta_z = float(delta_z)
        if abs(delta_z) < 1e-6:
            return True

        if target_speed is None:
            target_speed = float(getattr(self, 'cooperative_lift_speed', 0.020))
        if settle_sec is None:
            settle_sec = float(getattr(self, 'lift_cartesian_settle_sec', 0.0))
        if max_segment is None:
            # 连续抬升优先：尽量单段执行，避免分段停顿导致“卡顿上升”
            max_segment = max(abs(delta_z) + 0.002, float(getattr(self, 'lift_cartesian_max_segment', 0.25)))

        ok = await self._move_dual_cartesian_delta(
            delta_x=0.0,
            delta_y=0.0,
            delta_z=delta_z,
            description=description,
            max_step=max_step,
            avoid_collisions=False,
            max_segment=max_segment,
            target_speed=target_speed,
            settle_sec=settle_sec,
        )
        if ok:
            return True

        # 规划质量不足时，回退为小段连续（无段间停顿）执行
        retry_segment = max(0.02, float(getattr(self, 'lift_cartesian_retry_segment', 0.08)))
        if retry_segment + 1e-6 < abs(delta_z):
            self.get_logger().warn(
                f"{description}: 单段连续执行失败，回退分段连续执行 retry_segment={retry_segment:.3f}m")
            return await self._move_dual_cartesian_delta(
                delta_x=0.0,
                delta_y=0.0,
                delta_z=delta_z,
                description=f"{description}-分段重试",
                max_step=max_step,
                avoid_collisions=False,
                max_segment=retry_segment,
                target_speed=target_speed,
                settle_sec=0.0,
            )
        return False

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
            # 统一入口：优先复用 execute_task_flow 已缓存的目标位姿；若为空再走 get_target_pose_async
            target_pose = getattr(self, 'latest_target_pose', None)
            if target_pose is None:
                target_pose = await self.get_target_pose_async()
            if target_pose is None:
                raise RuntimeError("目标位姿为空")

            self.latest_target_pose = target_pose
            p_center = np.array([
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z
            ], dtype=float)
            
            q_bar = [
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z,
                target_pose.pose.orientation.w
            ]
            r_bar = R.from_quat(q_bar)
            
            pose_src = str(getattr(self, 'target_pose_source_frame', 'unknown'))
            self.get_logger().info(f"成功获取目标位姿: Center={p_center}, source={pose_src}")
            
            # 利用物体姿态动态推算抓取点：与 dual_scene.xml 的 grasp_site ±0.10m 对齐
            # 关键防撞：平票走Y + 最小横向间距约束（统一走 test2.py 的安全计算函数）
            p_left, p_right, grasp_meta = self._compute_safe_grasp_centers(p_center, r_bar)
            self.get_logger().info(
                f"抓取轴自动判定: {grasp_meta['axis_name']}, site_half_span={grasp_meta['span']:.3f}m, "
                f"Y间距={grasp_meta['y_spacing_after']:.4f}m")
            if grasp_meta['spacing_forced']:
                self.get_logger().warn(
                    f"预抓取防撞保护触发: Y间距从 {grasp_meta['y_spacing_before']:.4f}m "
                    f"提升至 {grasp_meta['y_spacing_after']:.4f}m (最小要求 {grasp_meta['min_lateral_spacing']:.4f}m)")

            self.get_logger().info(f"动态推算左抓取点: {p_left}")
            self.get_logger().info(f"动态推算右抓取点: {p_right}")

            # 动态更新抓取高度 (基于物体实时 Z 高度 + TCP 偏移)
            current_bar_z = p_center[2]
            self.GRASP_Z = (
                current_bar_z
                + self.TCP_OFFSET
                + self.grasp_z_safety_bias
                - getattr(self, 'grasp_extra_descent', 0.02)
            )
            self.PRE_GRASP_HEIGHT = self.GRASP_Z + 0.10
            self.LIFT_HEIGHT = self.GRASP_Z + 0.12
            base_pregrasp_z = float(self.PRE_GRASP_HEIGHT + float(getattr(self, 'pregrasp_z_offset', 0.0)))
            if bool(getattr(self, 'enforce_level_pregrasp', True)):
                left_pregrasp_z = base_pregrasp_z
                right_pregrasp_z = base_pregrasp_z
            else:
                left_pregrasp_z = float(base_pregrasp_z + float(getattr(self, 'pregrasp_left_z_offset', 0.0)))
                right_pregrasp_z = float(base_pregrasp_z + float(getattr(self, 'pregrasp_right_z_offset', 0.0)))

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
                lp.pose.position.z = left_pregrasp_z
                lp.pose.orientation = build_quat_msg(q_left_arr)

                rp = PoseStamped()
                rp.header.frame_id = self.planning_frame
                rp.pose.position.x = p_right[0]
                rp.pose.position.y = p_right[1]
                rp.pose.position.z = right_pregrasp_z
                rp.pose.orientation = build_quat_msg(q_right_arr)

                ik_l, vals_l = await self.compute_ik_natural_async(
                    'left', lp, avoid_collisions=True, allow_collision_fallback=False, verbose=False)
                ik_r, vals_r = await self.compute_ik_natural_async(
                    'right', rp, avoid_collisions=True, allow_collision_fallback=False, verbose=False)
                if ik_l is None or ik_r is None:
                    return -1e9, None, None

                if vals_l is None:
                    vals_l = extract_joint_values(ik_l, self.left_arm_joints)
                if vals_r is None:
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
                j7_soft = 2.30
                if abs(j7_l) > j7_soft:
                    j7_penalty += (abs(j7_l) - j7_soft) * 3.0
                if abs(j7_r) > j7_soft:
                    j7_penalty += (abs(j7_r) - j7_soft) * 3.0

                # 额外惩罚：偏离当前关节状态太大，会造成“预抓取姿态扭曲/突变”
                current = getattr(self, 'current_joint_state', {})
                delta_penalty = 0.0
                if isinstance(current, dict) and current:
                    for idx, jn in enumerate(self.left_arm_joints):
                        if jn in current and idx < len(vals_l):
                            delta_penalty += abs(float(vals_l[idx]) - float(current[jn]))
                    for idx, jn in enumerate(self.right_arm_joints):
                        if jn in current and idx < len(vals_r):
                            delta_penalty += abs(float(vals_r[idx]) - float(current[jn]))
                    delta_penalty *= 0.08

                natural_penalty = 0.0
                natural_scale = max(0.0, float(getattr(self, 'natural_ik_pregrasp_penalty_scale', 0.18)))
                if natural_scale > 0.0:
                    natural_penalty = natural_scale * (
                        float(self._compute_joint_naturalness_cost('left', vals_l)) +
                        float(self._compute_joint_naturalness_cost('right', vals_r))
                    )

                return base_score - j7_penalty - delta_penalty - natural_penalty, vals_l, vals_r

            best = {
                'score': -1e9,
                'q_l': q_base,
                'q_r': q_base,
                'tag': 'fallback',
                'j7_l': None,
                'j7_r': None,
            }

            allow_flip_candidate = bool(getattr(self, 'allow_flip_orientation_candidate', False))
            yaw_bias_max_deg = max(0.0, float(getattr(self, 'grasp_yaw_bias_max_deg', 2.0)))
            if yaw_bias_max_deg < 0.1:
                yaw_bias_deg_list = [0.0]
            else:
                yaw_bias_deg_list = [0.0, yaw_bias_max_deg, -yaw_bias_max_deg]

            base_rot_candidates = [
                ('base', R.from_quat(q_base)),
            ]
            if allow_flip_candidate:
                base_rot_candidates.append(('flip180', R.from_quat(q_flip)))

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
            self.left_pre_grasp.pose.position.z = left_pregrasp_z
            self.left_pre_grasp.pose.orientation = self.grasp_orientation_left

            self.right_pre_grasp = PoseStamped()
            self.right_pre_grasp.header.frame_id = self.planning_frame
            self.right_pre_grasp.pose.position.x = p_right[0]
            self.right_pre_grasp.pose.position.y = p_right[1]
            self.right_pre_grasp.pose.position.z = right_pregrasp_z
            self.right_pre_grasp.pose.orientation = self.grasp_orientation_right
            self.state2_used_fallback_pregrasp = False
            
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
                self.GRASP_Z = max(float(
                    self.BAR_RESTING_Z
                    + self.TCP_OFFSET
                    + self.grasp_z_safety_bias
                    - getattr(self, 'grasp_extra_descent', 0.02)
                ), 0.38)
                self.PRE_GRASP_HEIGHT = self.GRASP_Z + 0.12
                self.LIFT_HEIGHT = self.GRASP_Z + 0.15
                base_pregrasp_z = float(self.PRE_GRASP_HEIGHT + float(getattr(self, 'pregrasp_z_offset', 0.0)))
                if bool(getattr(self, 'enforce_level_pregrasp', True)):
                    left_pregrasp_z = base_pregrasp_z
                    right_pregrasp_z = base_pregrasp_z
                else:
                    left_pregrasp_z = float(base_pregrasp_z + float(getattr(self, 'pregrasp_left_z_offset', 0.0)))
                    right_pregrasp_z = float(base_pregrasp_z + float(getattr(self, 'pregrasp_right_z_offset', 0.0)))

                q_fallback = Quaternion(x=0.9239, y=0.3827, z=0.0, w=0.0)
                self.grasp_orientation_left = getattr(self, 'grasp_orientation_left', q_fallback)
                self.grasp_orientation_right = getattr(self, 'grasp_orientation_right', q_fallback)

                self.left_pre_grasp = PoseStamped()
                self.left_pre_grasp.header.frame_id = self.planning_frame
                self.left_pre_grasp.pose.position.x = float(p_left[0])
                self.left_pre_grasp.pose.position.y = float(p_left[1])
                self.left_pre_grasp.pose.position.z = left_pregrasp_z
                self.left_pre_grasp.pose.orientation = self.grasp_orientation_left

                self.right_pre_grasp = PoseStamped()
                self.right_pre_grasp.header.frame_id = self.planning_frame
                self.right_pre_grasp.pose.position.x = float(p_right[0])
                self.right_pre_grasp.pose.position.y = float(p_right[1])
                self.right_pre_grasp.pose.position.z = right_pregrasp_z
                self.right_pre_grasp.pose.orientation = self.grasp_orientation_right
                self.state2_used_fallback_pregrasp = True
                self.current_state = TaskState.EXECUTE_APPROACH
                return
            self.current_state = TaskState.ERROR

    async def state_execute_approach(self):
        """State 3: 规划并移动到预抓取点和抓取点"""
        self.get_logger().info(">> State 3: execute_approach() - 双臂安全平移并下探至抓取位置...")
        strict_pregrasp = bool(getattr(self, 'strict_pregrasp_gate', True))
        force_continue = bool(self._is_force_continue_enabled())
        target_pose_from_tf = bool(getattr(self, 'target_pose_from_tf', False))
        state2_used_fallback = bool(getattr(self, 'state2_used_fallback_pregrasp', False))
        demo_soft_pregrasp_gate = (
            force_continue and (
                (not strict_pregrasp) or
                (not target_pose_from_tf) or
                state2_used_fallback
            )
        )
        if demo_soft_pregrasp_gate and strict_pregrasp:
            reason = []
            if not target_pose_from_tf:
                reason.append('目标TF缺失')
            if state2_used_fallback:
                reason.append('State2兜底抓点')
            if not reason:
                reason.append('demo容错模式')
            self.get_logger().warn(
                f"预抓取门控降级为软判定({'+'.join(reason)})：偏差告警但不中断流程")
        
        # === 第一步：移动到预抓取点（高于铝条 15cm）===
        # 使用收紧的容差，防止 IK 求解器选择变形的关节配置
        success_pre = await self.sync_move_arms(
            self.left_pre_grasp, self.right_pre_grasp,
            pos_tol=0.02, ori_tol_x=0.05, ori_tol_y=0.05, ori_tol_z=0.08)
        if not success_pre:
            if demo_soft_pregrasp_gate:
                self.get_logger().warn("预抓取点移动失败，中期兜底：跳过预抓取，直接尝试下探")
            else:
                self.get_logger().error("预抓取点移动失败，触发复位")
                self.current_state = TaskState.ERROR
                return
        
        # 关节健壮性检查：到达预抓取点后，确认没有接近限位的变形配置
        await asyncio.sleep(0.3)  # 等待关节状态更新
        if not self.check_joint_sanity():
            if demo_soft_pregrasp_gate:
                self.get_logger().warn("预抓取后关节角接近限位，中期兜底：降速继续并尽快完成夹取")
            else:
                self.get_logger().error("预抓取后关节角接近限位，配置可能变形！触发复位")
                self.current_state = TaskState.ERROR
                return

        # 预夹取位姿精对齐：若当前末端与预抓取目标存在明显偏差，先做一次收紧容差对齐
        try:
            left_now, _ = await self.get_current_pose('mj_left_link8')
            right_now, _ = await self.get_current_pose('mj_right_link8')
            if left_now is not None and right_now is not None:
                left_pre_target_z = float(getattr(self.left_pre_grasp.pose.position, 'z', self.PRE_GRASP_HEIGHT))
                right_pre_target_z = float(getattr(self.right_pre_grasp.pose.position, 'z', self.PRE_GRASP_HEIGHT))
                left_xy_err = float(np.linalg.norm([
                    float(left_now[0]) - float(self.dynamic_p_left[0]),
                    float(left_now[1]) - float(self.dynamic_p_left[1]),
                ]))
                right_xy_err = float(np.linalg.norm([
                    float(right_now[0]) - float(self.dynamic_p_right[0]),
                    float(right_now[1]) - float(self.dynamic_p_right[1]),
                ]))
                left_z_err = abs(float(left_now[2]) - left_pre_target_z)
                right_z_err = abs(float(right_now[2]) - right_pre_target_z)
                z_gap = abs(float(left_now[2]) - float(right_now[2]))

                xy_tol = max(0.001, float(getattr(self, 'pregrasp_align_xy_tol', 0.006)))
                z_tol = max(0.001, float(getattr(self, 'pregrasp_align_z_tol', 0.006)))
                z_gap_tol = max(0.001, float(getattr(self, 'pregrasp_align_z_gap_tol', 0.006)))
                need_refine = (
                    left_xy_err > xy_tol or right_xy_err > xy_tol or
                    left_z_err > z_tol or right_z_err > z_tol or z_gap > z_gap_tol
                )
                if need_refine:
                    self.get_logger().warn(
                        f"预抓取位姿存在偏差，执行精对齐: "
                        f"L_xy={left_xy_err:.4f}, R_xy={right_xy_err:.4f}, "
                        f"L_z={left_z_err:.4f}, R_z={right_z_err:.4f}, z_gap={z_gap:.4f}")
                    refine_ok = await self.sync_move_arms(
                        self.left_pre_grasp,
                        self.right_pre_grasp,
                        pos_tol=float(getattr(self, 'pregrasp_refine_pos_tol', 0.008)),
                        ori_tol_x=0.04,
                        ori_tol_y=0.04,
                        ori_tol_z=0.06,
                        max_retries=2,
                    )
                    if not refine_ok and (not demo_soft_pregrasp_gate):
                        self.get_logger().error("预抓取精对齐失败，终止下探")
                        self.current_state = TaskState.ERROR
                        return
                    await asyncio.sleep(0.2)
        except Exception as e:
            self.get_logger().warn(f"预抓取精对齐检查异常: {e}")

        # 预抓取强制几何对齐（可选，默认关闭）：在极端偏差场景下可启用
        run_force_align = bool(getattr(self, 'enable_pregrasp_force_align', False))
        if run_force_align and demo_soft_pregrasp_gate and (not target_pose_from_tf or state2_used_fallback):
            self.get_logger().warn("检测到 TF 缺失/兜底抓点：跳过预抓取强制几何对齐，避免误判触发复位")
            run_force_align = False
        if run_force_align:
            try:
                left_chk, _ = await self.get_current_pose('mj_left_link8')
                right_chk, _ = await self.get_current_pose('mj_right_link8')
                if left_chk is not None and right_chk is not None:
                    force_xy_tol = max(0.001, float(getattr(self, 'pregrasp_force_align_xy_tol', 0.003)))
                    force_z_gap_tol = max(0.001, float(getattr(self, 'pregrasp_force_align_z_gap_tol', 0.003)))

                    left_xy_err = float(np.linalg.norm([
                        float(left_chk[0]) - float(self.dynamic_p_left[0]),
                        float(left_chk[1]) - float(self.dynamic_p_left[1]),
                    ]))
                    right_xy_err = float(np.linalg.norm([
                        float(right_chk[0]) - float(self.dynamic_p_right[0]),
                        float(right_chk[1]) - float(self.dynamic_p_right[1]),
                    ]))
                    z_gap = abs(float(left_chk[2]) - float(right_chk[2]))

                    force_align_needed = (
                        left_xy_err > force_xy_tol or
                        right_xy_err > force_xy_tol or
                        z_gap > force_z_gap_tol
                    )
                    if force_align_needed:
                        align_z = max(float(self.PRE_GRASP_HEIGHT), 0.5 * (float(left_chk[2]) + float(right_chk[2])))
                        self.get_logger().warn(
                            f"预抓取强制对齐触发: L_xy={left_xy_err:.4f}, R_xy={right_xy_err:.4f}, "
                            f"z_gap={z_gap:.4f}, align_z={align_z:.4f}")

                        left_force_align = PoseStamped()
                        left_force_align.header.frame_id = self.planning_frame
                        left_force_align.pose.position.x = float(self.dynamic_p_left[0])
                        left_force_align.pose.position.y = float(self.dynamic_p_left[1])
                        left_force_align.pose.position.z = float(align_z)
                        left_force_align.pose.orientation = self.grasp_orientation_left

                        right_force_align = PoseStamped()
                        right_force_align.header.frame_id = self.planning_frame
                        right_force_align.pose.position.x = float(self.dynamic_p_right[0])
                        right_force_align.pose.position.y = float(self.dynamic_p_right[1])
                        right_force_align.pose.position.z = float(align_z)
                        right_force_align.pose.orientation = self.grasp_orientation_right

                        force_align_ok = await self.sync_move_arms(
                            left_force_align,
                            right_force_align,
                            pos_tol=float(getattr(self, 'pregrasp_force_align_pos_tol', 0.004)),
                            ori_tol_x=0.035,
                            ori_tol_y=0.035,
                            ori_tol_z=0.05,
                            max_retries=2,
                        )
                        await asyncio.sleep(0.2)

                        left_chk2, _ = await self.get_current_pose('mj_left_link8')
                        right_chk2, _ = await self.get_current_pose('mj_right_link8')
                        if left_chk2 is not None and right_chk2 is not None:
                            z_gap2 = abs(float(left_chk2[2]) - float(right_chk2[2]))
                            left_xy_err2 = float(np.linalg.norm([
                                float(left_chk2[0]) - float(self.dynamic_p_left[0]),
                                float(left_chk2[1]) - float(self.dynamic_p_left[1]),
                            ]))
                            right_xy_err2 = float(np.linalg.norm([
                                float(right_chk2[0]) - float(self.dynamic_p_right[0]),
                                float(right_chk2[1]) - float(self.dynamic_p_right[1]),
                            ]))
                            if (
                                (left_xy_err2 > force_xy_tol or right_xy_err2 > force_xy_tol or z_gap2 > force_z_gap_tol)
                                and (not demo_soft_pregrasp_gate)
                            ):
                                self.get_logger().error(
                                    f"预抓取强制对齐后仍偏差过大: L_xy={left_xy_err2:.4f}, "
                                    f"R_xy={right_xy_err2:.4f}, z_gap={z_gap2:.4f}")
                                self.current_state = TaskState.ERROR
                                return
                        if not force_align_ok and (not demo_soft_pregrasp_gate):
                            self.get_logger().error("预抓取强制对齐执行失败")
                            self.current_state = TaskState.ERROR
                            return
            except Exception as e:
                self.get_logger().warn(f"预抓取强制几何对齐异常: {e}")

        # 缓存预抓取位的14关节快照，用于“持物抬升”时绕过 IK 直接回放
        try:
            snapshot = [self.current_joint_state.get(jn, None) for jn in self.all_arm_joints]
            if all(v is not None for v in snapshot):
                self.pre_grasp_joint_snapshot = [float(v) for v in snapshot]
                self.get_logger().info("已缓存预抓取关节快照，可用于后续无IK抬升兜底")
        except Exception:
            pass
        
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
        # 目标整体下调约 2cm 后，仍保留分级候选避免一次性过冲
        candidate_grasp_z = [
            self.GRASP_Z + 0.010,
            self.GRASP_Z + 0.005,
            self.GRASP_Z,
            self.GRASP_Z - 0.005,
            self.GRASP_Z - 0.010,
            self.GRASP_Z - 0.015,
            self.GRASP_Z - 0.020,
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
            left_cur = None
            right_cur = None
            try:
                if self.cartesian_client is not None and self.cartesian_client.wait_for_service(timeout_sec=1.0):
                    from moveit_msgs.srv import GetCartesianPath as GCP
                    left_cur, left_cur_q = await self.get_current_pose('mj_left_link8')
                    right_cur, right_cur_q = await self.get_current_pose('mj_right_link8')

                    align_xy_first = bool(getattr(self, 'grasp_cartesian_align_xy_first', True))
                    align_xy_tol = max(0.001, float(getattr(self, 'grasp_cartesian_waypoint_xy_tol', 0.003)))
                    split_height = max(0.01, float(getattr(self, 'grasp_cartesian_split_height', 0.035)))
                    max_step = max(0.002, float(getattr(self, 'grasp_cartesian_max_step', 0.006)))

                    left_waypoints = []
                    right_waypoints = []

                    if left_cur is not None and left_cur_q is not None:
                        left_xy_off = max(
                            abs(float(left_cur[0]) - float(self.dynamic_p_left[0])),
                            abs(float(left_cur[1]) - float(self.dynamic_p_left[1]))
                        )
                        if align_xy_first and left_xy_off > align_xy_tol:
                            wp_align_l = PoseStamped().pose
                            wp_align_l.position.x = float(self.dynamic_p_left[0])
                            wp_align_l.position.y = float(self.dynamic_p_left[1])
                            wp_align_l.position.z = float(left_cur[2])
                            wp_align_l.orientation = self.grasp_orientation_left
                            left_waypoints.append(wp_align_l)
                        delta_l_z = float(candidate_z) - float(left_cur[2])
                        if abs(delta_l_z) > split_height:
                            wp_mid_l = PoseStamped().pose
                            wp_mid_l.position.x = float(self.dynamic_p_left[0])
                            wp_mid_l.position.y = float(self.dynamic_p_left[1])
                            wp_mid_l.position.z = float(left_cur[2] + 0.5 * delta_l_z)
                            wp_mid_l.orientation = self.grasp_orientation_left
                            left_waypoints.append(wp_mid_l)
                    left_waypoints.append(left_grasp_pose.pose)

                    if right_cur is not None and right_cur_q is not None:
                        right_xy_off = max(
                            abs(float(right_cur[0]) - float(self.dynamic_p_right[0])),
                            abs(float(right_cur[1]) - float(self.dynamic_p_right[1]))
                        )
                        if align_xy_first and right_xy_off > align_xy_tol:
                            wp_align_r = PoseStamped().pose
                            wp_align_r.position.x = float(self.dynamic_p_right[0])
                            wp_align_r.position.y = float(self.dynamic_p_right[1])
                            wp_align_r.position.z = float(right_cur[2])
                            wp_align_r.orientation = self.grasp_orientation_right
                            right_waypoints.append(wp_align_r)
                        delta_r_z = float(candidate_z) - float(right_cur[2])
                        if abs(delta_r_z) > split_height:
                            wp_mid_r = PoseStamped().pose
                            wp_mid_r.position.x = float(self.dynamic_p_right[0])
                            wp_mid_r.position.y = float(self.dynamic_p_right[1])
                            wp_mid_r.position.z = float(right_cur[2] + 0.5 * delta_r_z)
                            wp_mid_r.orientation = self.grasp_orientation_right
                            right_waypoints.append(wp_mid_r)
                    right_waypoints.append(right_grasp_pose.pose)

                    # 左臂笛卡尔直线下探
                    req_l = GCP.Request()
                    req_l.group_name = self._group_name('left')
                    req_l.waypoints = left_waypoints
                    req_l.max_step = float(max_step)
                    req_l.jump_threshold = 0.0
                    req_l.avoid_collisions = False
                    fut_l = self.cartesian_client.call_async(req_l)
                    res_l = await asyncio.wait_for(fut_l, timeout=10.0)

                    # 右臂笛卡尔直线下探
                    req_r = GCP.Request()
                    req_r.group_name = self._group_name('right')
                    req_r.waypoints = right_waypoints
                    req_r.max_step = float(max_step)
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
                descent_speed = max(0.006, float(getattr(self, 'grasp_descent_speed', 0.012)))
                descent_dist = abs(float(candidate_z) - float(self.PRE_GRASP_HEIGHT))
                if left_cur is not None and right_cur is not None:
                    descent_dist = max(
                        abs(float(candidate_z) - float(left_cur[2])),
                        abs(float(candidate_z) - float(right_cur[2])),
                    )
                def _traj_end_sec(traj):
                    if traj is None or not getattr(traj, 'points', None):
                        return 0.1
                    last = traj.points[-1].time_from_start
                    return float(last.sec) + float(last.nanosec) * 1e-9
                base_duration = max(0.1, _traj_end_sec(left_traj), _traj_end_sec(right_traj))
                desired_duration = max(1.2, float(descent_dist) / descent_speed)
                time_scale = max(1.0, min(8.0, desired_duration / base_duration))

                self.get_logger().info(
                    f"使用笛卡尔直线轨迹并行执行双臂下探... "
                    f"dist={descent_dist:.4f}m, base={base_duration:.2f}s, desired={desired_duration:.2f}s, "
                    f"time_scale={time_scale:.2f}")
                ok = await self._execute_merged_trajectory(left_traj, right_traj, time_scale=time_scale)
                success_grasp = bool(ok)
            else:
                # 回退：若未能获得 Cartesian Path 或无双臂控制器，则使用伺服直降（更稳定且避免 OMPL）
                self.get_logger().info("降级为伺服直降执行下探（servo_move_delta_z）")
                current_ee_z = await self._estimate_dual_ee_z()
                if current_ee_z is None:
                    current_ee_z = self.PRE_GRASP_HEIGHT
                delta_z = float(candidate_z) - float(current_ee_z)
                descent_speed = max(0.006, float(getattr(self, 'grasp_descent_speed', 0.012)))
                descent_duration = max(1.2, abs(delta_z) / descent_speed)
                success_grasp = await self.servo_move_delta_z(delta_z=delta_z, duration=descent_duration)
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

    async def controlled_grasp_close(self, target_width=0.02, max_width=0.08, step=0.005,
                                     effort=170.0, contact_threshold=1.0, timeout=6.0,
                                     block_margin=0.002):
        """逐步收紧夹爪直到达到目标宽度或检测到接触力（带超时）。
        返回 True 表示成功夹住（检测到接触力或检测到“夹爪被物体撑住”），
        仅“到达目标宽度”不再视为成功，避免误判。
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
        block_confirm_steps = max(1, int(getattr(self, 'grasp_block_confirm_steps', 2)))
        block_counter = 0
        while width > target_width + 1e-6:
            # 每一步收紧
            width = max(target_width, width - step)
            await self.sync_grasp(width, effort, wait_for_result=True)
            await asyncio.sleep(0.10)

            # 读取力反馈判断是否接触
            left_fy = abs(self.current_left_wrench.wrench.force.y - self.left_force_bias['y'])
            right_fy = abs(self.current_right_wrench.wrench.force.y - self.right_force_bias['y'])
            left_opening = self.get_gripper_opening_estimate('left')
            right_opening = self.get_gripper_opening_estimate('right')
            self.get_logger().info(
                f"尝试宽度={width:.3f}m, 力反馈 L_y={left_fy:.2f}N R_y={right_fy:.2f}N, "
                f"opening L={left_opening} R={right_opening}")

            if left_fy > contact_threshold or right_fy > contact_threshold:
                self.get_logger().info(f"检测到接触力：L_y={left_fy:.2f}N R_y={right_fy:.2f}N，停止收紧")
                success = True
                break

            # 回退判据：若命令继续收紧，但开度无法同步变小，说明大概率已夹到物体
            if left_opening is not None and right_opening is not None:
                left_blocked = left_opening > width + block_margin
                right_blocked = right_opening > width + block_margin
                if left_blocked and right_blocked:
                    block_counter += 1
                else:
                    block_counter = 0
                if block_counter >= block_confirm_steps:
                    self.get_logger().info(
                        f"检测到夹爪持续受阻({block_counter}步)，判定为接触成功")
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

        # 末端兜底判据：若最终开度明显大于空抓闭合值，说明夹爪中间有物体
        left_final = self.get_gripper_opening_estimate('left')
        right_final = self.get_gripper_opening_estimate('right')
        hold_min = float(getattr(self, 'grasp_object_opening_min', 0.012))
        opening_evidence = (
            left_final is not None and right_final is not None and
            (left_final > hold_min or right_final > hold_min)
        )
        if not success and opening_evidence:
            self.get_logger().info(
                f"检测到开度夹持证据: left={left_final:.4f}, right={right_final:.4f}, hold_min={hold_min:.4f}")
            success = True

        return success

    def _adaptive_grasp_downward(self, base_down: float, phase: str,
                                 grasp_target: float, contact_threshold: float,
                                 block_margin: float) -> float:
        """根据实时开度与受力证据自适应调整下压位移。"""
        base = max(0.0, float(base_down))
        if base <= 1e-6:
            return 0.0

        phase_key = str(phase).lower()
        min_default = 0.0015 if phase_key == 'preload' else 0.0020
        max_default = 0.0080 if phase_key == 'preload' else 0.0140
        min_down = max(0.0, float(getattr(self, f'grasp_{phase_key}_down_min', min_default)))
        max_down = max(min_down, float(getattr(self, f'grasp_{phase_key}_down_max', max_default)))

        left_open = self.get_gripper_opening_estimate('left')
        right_open = self.get_gripper_opening_estimate('right')
        opening_samples = [v for v in (left_open, right_open) if v is not None]
        avg_open = (sum(opening_samples) / len(opening_samples)) if opening_samples else None

        left_fy = abs(self.current_left_wrench.wrench.force.y - self.left_force_bias.get('y', 0.0))
        right_fy = abs(self.current_right_wrench.wrench.force.y - self.right_force_bias.get('y', 0.0))
        max_fy = max(left_fy, right_fy)

        hold_min = float(getattr(self, 'grasp_object_opening_min', 0.012))
        open_pos = float(getattr(self, 'gripper_open_position', 0.04))
        adaptive = base

        if phase_key == 'preload':
            if avg_open is not None:
                boost = float(getattr(self, 'grasp_preload_open_boost', 0.0015))
                reduce = float(getattr(self, 'grasp_preload_open_reduce', 0.0010))
                near_full_open = avg_open >= (open_pos * 0.90)
                likely_contacted = (
                    avg_open <= max(hold_min, grasp_target + block_margin) or
                    max_fy >= contact_threshold * 0.80
                )
                if near_full_open and max_fy < contact_threshold * 0.60:
                    adaptive = base + boost
                elif likely_contacted:
                    adaptive = max(min_down, base - reduce)
        else:
            boost_empty = float(getattr(self, 'grasp_retry_empty_boost', 0.0040))
            reduce_contact = float(getattr(self, 'grasp_retry_contact_reduce', 0.0015))
            both_open_known = left_open is not None and right_open is not None
            both_tight = (
                both_open_known and
                left_open < hold_min * 0.75 and
                right_open < hold_min * 0.75
            )
            both_blocked = (
                both_open_known and
                left_open > grasp_target + block_margin and
                right_open > grasp_target + block_margin
            )
            if both_tight and max_fy < contact_threshold * 0.60:
                adaptive = base + boost_empty
            elif both_blocked or max_fy >= contact_threshold * 0.90:
                adaptive = max(min_down, base - reduce_contact)

        adaptive = max(min_down, min(max_down, adaptive))
        self.get_logger().info(
            f"[自适应下压:{phase}] base={base:.4f} -> use={adaptive:.4f}, "
            f"open_l={left_open}, open_r={right_open}, fy_l={left_fy:.2f}, fy_r={right_fy:.2f}, "
            f"range=[{min_down:.4f},{max_down:.4f}]")
        return adaptive

    async def state_close_grippers(self):
        """State 4: 发送夹爪闭合指令，附着物体构建闭链"""
        self.get_logger().info(">> State 4: close_grippers() - 执行夹爪同步夹取...")
        # 防止上一次任务残留保压线程
        try:
            await self.stop_gripper_hold("进入新的夹取阶段")
        except Exception:
            pass
        
        # === 夹爪参数设定 ===
        # 针对“夹起失败”场景，收紧目标改得更激进，确保产生有效挤压
        # 过小目标宽度会在高夹持力下导致“过挤压滑脱/弹飞”，采用更贴近工件厚度的保压宽度
        GRASP_POSITION = max(0.012, min(float(self.gripper_closed_position), 0.020))
        # 夹爪最大力：对轻质铝条用 170N (最大值)
        GRASP_EFFORT = self.gripper_effort_close
        CONTACT_THRESHOLD = float(getattr(self, 'grasp_contact_threshold', 0.8))
        BLOCK_MARGIN = float(getattr(self, 'grasp_block_margin', 0.003))
        
        # 使用 ACM 允许末端与目标物体发生接触（替代 attach/remove），然后进行受力驱动的逐步闭合
        self.get_logger().info("准备允许夹持碰撞并进行受力驱动的逐步夹紧...")
        try:
            ok = await self.set_acm_allow('mj_left_link8', 'target_bar', allow=True)
            if not ok:
                self.get_logger().warn("设置 ACM 失败或未确认，继续但可能导致 MoveIt 拒绝接触")
        except Exception as e:
            self.get_logger().warn(f"设置 ACM 异常: {e}，继续执行但可能存在碰撞问题")

        # 直夹模式：先执行固定开度闭合，但必须通过夹持证据校验；否则自动回退到受力驱动闭合
        if bool(getattr(self, 'use_direct_grasp_close', False)):
            GRASP_EFFORT = float(self.gripper_effort_close)
            total_width = max(0.0, float(getattr(self, 'direct_grasp_total_width', 0.036)))
            target_single = self._clamp_gripper_position(total_width * 0.5)
            direct_block_margin = float(getattr(self, 'direct_grasp_block_margin', 0.0015))

            self.get_logger().info(
                f"直夹模式启用：目标总开度={total_width:.3f}m（单指={target_single:.3f}m），执行后将校验夹持证据")

            await self.sync_grasp(target_single, GRASP_EFFORT, wait_for_result=True)
            await asyncio.sleep(0.4)

            left_opening = self.get_gripper_opening_estimate('left')
            right_opening = self.get_gripper_opening_estimate('right')
            blocked_evidence = (
                left_opening is not None and right_opening is not None and
                left_opening > target_single + direct_block_margin and
                right_opening > target_single + direct_block_margin
            )

            self.get_logger().info(
                f"[直夹校验] target={target_single:.4f}, opening_left={left_opening}, "
                f"opening_right={right_opening}, blocked={blocked_evidence}")

            if not blocked_evidence:
                self.get_logger().warn("直夹未检测到夹持证据，回退到受力驱动逐步夹紧")
            else:
                # 进入搬运前启动持续保压，降低提举阶段松脱概率
                try:
                    hold_min_w = max(0.0, float(getattr(self, 'grasp_hold_width_min', 0.008)))
                    hold_max_w = max(hold_min_w, float(getattr(self, 'grasp_hold_width_max', 0.014)))
                    measured_min = None
                    if left_opening is not None and right_opening is not None:
                        measured_min = max(0.0, min(left_opening, right_opening) - 0.002)
                    hold_seed = target_single if measured_min is None else measured_min
                    hold_width = max(hold_min_w, min(hold_seed, hold_max_w))
                    self.get_logger().info(
                        f"[保压宽度-直夹] measured_min={measured_min}, use={hold_width:.4f}, "
                        f"range=[{hold_min_w:.4f},{hold_max_w:.4f}]")
                    await self.start_gripper_hold(width=hold_width, effort=GRASP_EFFORT)
                except Exception as e:
                    self.get_logger().warn(f"启动夹爪保压循环失败: {e}")

                # 在 MoveIt 中附着物体以形成闭链
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

                try:
                    if not getattr(self, 'compliance_task_active', False):
                        self.compliance_task_active = True
                        asyncio.create_task(self.simulated_compliance_control_loop())
                        self.get_logger().info("已启动 simulated_compliance_control_loop，用于闭链搬运的阻抗/力控保护")
                except Exception:
                    pass

                self.current_state = TaskState.PLAN_SYNC_TRAJECTORY
                return

        # 在闭合前进行力传感器去皮校准，以获得可靠的接触判定基线
        await self.calibrate_force_sensors(duration_sec=0.5)

        # 夹取前微下压：给指尖建立接触预载，减少“只碰不夹”概率
        preload_down = self._adaptive_grasp_downward(
            base_down=float(getattr(self, 'grasp_preload_down', 0.008)),
            phase='preload',
            grasp_target=GRASP_POSITION,
            contact_threshold=CONTACT_THRESHOLD,
            block_margin=BLOCK_MARGIN,
        )
        if preload_down > 1e-6:
            self.get_logger().info(f"夹取预载：先微下压 {preload_down:.3f}m")
            ok_preload = await self._move_dual_cartesian_delta_z(
                delta_z=-preload_down, description="夹取预载下压")
            if not ok_preload:
                await self.servo_move_delta_z(delta_z=-preload_down, duration=max(0.6, preload_down / 0.02))
            await asyncio.sleep(0.2)

        # 使用受力驱动的逐步夹紧（从大到小），优先使用带反馈的逐步收紧策略
        grasp_ok = await self.controlled_grasp_close(target_width=GRASP_POSITION,
                                                     max_width=self.gripper_open_position,
                                                     step=0.0025,
                                                     effort=GRASP_EFFORT,
                                                     contact_threshold=CONTACT_THRESHOLD,
                                                     timeout=8.0,
                                                     block_margin=BLOCK_MARGIN)

        # 二次兜底：继续下压并强夹到 0，给出“中期演示优先”的硬手段
        if not grasp_ok:
            retry_down = self._adaptive_grasp_downward(
                base_down=float(getattr(self, 'grasp_retry_down', 0.008)),
                phase='retry',
                grasp_target=GRASP_POSITION,
                contact_threshold=CONTACT_THRESHOLD,
                block_margin=BLOCK_MARGIN,
            )
            self.get_logger().warn("首次夹紧未检出接触，执行二次兜底：再下压并强夹")
            if retry_down > 1e-6:
                ok_retry_preload = await self._move_dual_cartesian_delta_z(
                    delta_z=-retry_down, description="二次夹取预载下压")
                if not ok_retry_preload:
                    await self.servo_move_delta_z(delta_z=-retry_down, duration=max(0.6, retry_down / 0.02))
                await asyncio.sleep(0.2)
            grasp_ok = await self.controlled_grasp_close(target_width=0.0,
                                                         max_width=self.gripper_open_position,
                                                         step=0.002,
                                                         effort=GRASP_EFFORT,
                                                         contact_threshold=max(0.3, CONTACT_THRESHOLD * 0.5),
                                                         timeout=6.0,
                                                         block_margin=BLOCK_MARGIN)

        if grasp_ok:
            self.get_logger().info("受力驱动夹取成功，准备搬运")

            # 确保夹爪以最终目标宽度锁定以增加稳定性，并等待物理稳定
            try:
                await self.sync_grasp(GRASP_POSITION, GRASP_EFFORT)
            except Exception:
                pass
            await asyncio.sleep(0.6)

            # 夹持证据复核：若最终几乎闭死，通常是空抓，触发一次额外下压重试
            left_hold = self.get_gripper_opening_estimate('left')
            right_hold = self.get_gripper_opening_estimate('right')
            hold_min = float(getattr(self, 'grasp_object_opening_min', 0.012))
            if (
                left_hold is not None and right_hold is not None and
                left_hold < hold_min and right_hold < hold_min
            ):
                self.get_logger().warn(
                    f"夹持复核疑似空抓: left={left_hold:.4f}, right={right_hold:.4f}, hold_min={hold_min:.4f}")
                retry_down = self._adaptive_grasp_downward(
                    base_down=float(getattr(self, 'grasp_retry_down', 0.008)),
                    phase='retry',
                    grasp_target=GRASP_POSITION,
                    contact_threshold=CONTACT_THRESHOLD,
                    block_margin=BLOCK_MARGIN,
                )
                if retry_down > 1e-6:
                    ok_regrasp_preload = await self._move_dual_cartesian_delta_z(
                        delta_z=-retry_down, description="空抓复核下压")
                    if not ok_regrasp_preload:
                        await self.servo_move_delta_z(delta_z=-retry_down, duration=max(0.6, retry_down / 0.02))
                    await asyncio.sleep(0.2)
                await self.sync_grasp(0.0, GRASP_EFFORT)
                await asyncio.sleep(0.4)

            preload_lift = max(0.0, float(getattr(self, 'grasp_preload_lift', 0.015)))
            if preload_lift > 1e-6:
                self.get_logger().info(f"夹后预载：小幅抬升 {preload_lift:.3f}m 以稳定持物")
                ok_post_lift = await self._move_dual_cartesian_delta_z(
                    delta_z=preload_lift, description="夹后预载抬升")
                if not ok_post_lift:
                    await self.servo_move_delta_z(delta_z=preload_lift, duration=max(0.8, preload_lift / 0.02))
                await asyncio.sleep(0.2)

            # 进入搬运前启动持续保压，降低提举阶段松脱概率
            try:
                hold_min_w = max(0.0, float(getattr(self, 'grasp_hold_width_min', 0.008)))
                hold_max_w = max(hold_min_w, float(getattr(self, 'grasp_hold_width_max', 0.014)))
                hold_left = self.get_gripper_opening_estimate('left')
                hold_right = self.get_gripper_opening_estimate('right')
                measured_min = None
                if hold_left is not None and hold_right is not None:
                    measured_min = max(0.0, min(hold_left, hold_right) - 0.002)
                hold_seed = GRASP_POSITION if measured_min is None else measured_min
                hold_width = max(hold_min_w, min(hold_seed, hold_max_w))
                self.get_logger().info(
                    f"[保压宽度-受力夹取] measured_min={measured_min}, use={hold_width:.4f}, "
                    f"range=[{hold_min_w:.4f},{hold_max_w:.4f}]")
                await self.start_gripper_hold(width=hold_width, effort=GRASP_EFFORT)
            except Exception as e:
                self.get_logger().warn(f"启动夹爪保压循环失败: {e}")

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
            await self.sync_grasp(0.0, GRASP_EFFORT)
            await asyncio.sleep(1.0)
            left_contact_force = abs(self.current_left_wrench.wrench.force.y - self.left_force_bias.get('y', 0.0))
            right_contact_force = abs(self.current_right_wrench.wrench.force.y - self.right_force_bias.get('y', 0.0))
            left_opening = self.get_gripper_opening_estimate('left')
            right_opening = self.get_gripper_opening_estimate('right')
            self.get_logger().info(
                f"[兜底判定] L_y={left_contact_force:.2f}N R_y={right_contact_force:.2f}N, "
                f"opening L={left_opening} R={right_opening}")
            blocked_by_object = (
                left_opening is not None and right_opening is not None and
                (left_opening > 0.002 or right_opening > 0.002)
            )
            if left_contact_force > 1.0 or right_contact_force > 1.0 or blocked_by_object:
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
        """State 5: 使用双臂笛卡尔直线抬升（根治 Servo 不可用问题）"""
        self.get_logger().info(">> State 5: plan_sync_trajectory() - 笛卡尔直线抬升")
        
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
                f"执行笛卡尔直线抬升: current_z={current_ee_z:.3f}, "
                f"target_z={target_z:.3f}, delta_z={delta_z:.3f}, duration={duration:.2f}s")
            ok = await self._move_dual_cartesian_delta_z(
                delta_z=delta_z, description="State5主抬升(双臂笛卡尔)")
            if not ok:
                self.get_logger().warn("State5 笛卡尔抬升失败，尝试预抓取关节快照回放")
                snapshot = getattr(self, 'pre_grasp_joint_snapshot', None)
                if snapshot is not None:
                    ok = await self._move_dual_joints_direct(
                        snapshot, duration=max(2.5, duration), description="State5关节快照回放")
                if not ok:
                    self.get_logger().warn("State5 快照回放失败，最后尝试 IK 抬升")
                    await self._dual_delta_z_fallback(delta_z=delta_z, description="State5 IK兜底")
        
        await asyncio.sleep(0.5)
        
        # 构建一个虚拟的笛卡尔轨迹（用于兼容后续执行逻辑）
        # 实际State 6会使用伺服而不是这个轨迹
        self.planned_master_cartesian_trajectory = []
        
        # 计算偏移量
        self.traj_offset_x = float(self.dynamic_p_right[0]) - float(self.dynamic_p_left[0])
        self.traj_offset_y = float(self.dynamic_p_right[1]) - float(self.dynamic_p_left[1])
        self.traj_offset_z = 0.0
        
        self.get_logger().info("✓ 上升阶段完成，准备进入搬运执行阶段")
        self.current_state = TaskState.EXECUTE_WITH_COMPLIANCE

    async def state_execute_with_compliance(self):
        """State 6: 执行搬运（先抬升再平移到放置位）"""
        self.get_logger().info(">> State 6: execute_with_compliance() - 执行搬运...")
        
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
            ok = await self._move_dual_cartesian_delta(
                delta_z=delta_z,
                description="State6补偿抬升(双臂笛卡尔)",
                max_step=0.012,
                max_segment=float(getattr(self, 'lift_cartesian_retry_segment', 0.08)),
                target_speed=float(getattr(self, 'cooperative_lift_speed', 0.020)),
                settle_sec=float(getattr(self, 'lift_cartesian_settle_sec', 0.0)),
            )
            if not ok:
                self.get_logger().warn("State6 笛卡尔补偿抬升失败，跳过额外抬升直接进入放置")
        else:
            self.get_logger().info("当前高度已足够，跳过 State 6 额外抬升")

        # 主搬运平移：真正把物体从抓取点移动到放置点
        transport_dx = float(getattr(self, 'TRANSPORT_X_OFFSET', 0.0))
        transport_dy = float(getattr(self, 'TRANSPORT_Y_OFFSET', 0.0))
        transport_dz = float(getattr(self, 'TRANSPORT_Z_OFFSET', 0.0))
        transport_norm = float(np.linalg.norm([transport_dx, transport_dy, transport_dz]))
        if transport_norm > 1e-4:
            left_before, _ = await self.get_current_pose('mj_left_link8')
            right_before, _ = await self.get_current_pose('mj_right_link8')
            if left_before is not None and right_before is not None:
                self.get_logger().info(
                    f"开始主搬运平移: delta=({transport_dx:.3f},{transport_dy:.3f},{transport_dz:.3f})m; "
                    f"left: ({left_before[0]:.3f},{left_before[1]:.3f},{left_before[2]:.3f}) -> "
                    f"({left_before[0] + transport_dx:.3f},{left_before[1] + transport_dy:.3f},{left_before[2] + transport_dz:.3f}), "
                    f"right: ({right_before[0]:.3f},{right_before[1]:.3f},{right_before[2]:.3f}) -> "
                    f"({right_before[0] + transport_dx:.3f},{right_before[1] + transport_dy:.3f},{right_before[2] + transport_dz:.3f})")
            transport_speed = float(getattr(self, 'cooperative_transport_speed', 0.018))
            transport_max_step = max(0.002, float(getattr(self, 'transport_cartesian_max_step', 0.006)))
            transport_retry_segment = max(0.02, float(getattr(self, 'transport_cartesian_retry_segment', 0.08)))
            transport_margin = max(0.01, float(getattr(self, 'transport_cartesian_one_shot_margin', 0.02)))
            transport_one_shot_segment = max(transport_norm + transport_margin, transport_retry_segment + 0.02)
            transport_settle = max(0.0, float(getattr(self, 'transport_cartesian_settle_sec', 0.0)))

            ok_transport = await self._move_dual_cartesian_delta(
                delta_x=transport_dx,
                delta_y=transport_dy,
                delta_z=transport_dz,
                description="State6主搬运平移(双臂笛卡尔-单段优先)",
                max_step=transport_max_step,
                avoid_collisions=False,
                max_segment=transport_one_shot_segment,
                target_speed=transport_speed,
                settle_sec=transport_settle,
            )
            if not ok_transport:
                self.get_logger().warn("主搬运单段执行失败，回退分段连续执行")
                ok_transport = await self._move_dual_cartesian_delta(
                    delta_x=transport_dx,
                    delta_y=transport_dy,
                    delta_z=transport_dz,
                    description="State6主搬运平移(双臂笛卡尔-分段回退)",
                    max_step=transport_max_step,
                    avoid_collisions=False,
                    max_segment=transport_retry_segment,
                    target_speed=max(0.006, transport_speed * 0.9),
                    settle_sec=0.0,
                )
            if not ok_transport:
                msg = "State6 主搬运平移失败，未到达放置点"
                if self._is_force_continue_enabled():
                    self.get_logger().warn(f"{msg}，中期演示模式下继续执行")
                else:
                    self.get_logger().error(msg)
                    self.current_state = TaskState.ERROR
                    return
        else:
            self.get_logger().warn("TRANSPORT 偏移量接近 0，跳过主搬运平移")
        
        await asyncio.sleep(0.5)
        self.get_logger().info("✓ 搬运动作执行完成，进入开爪放置阶段")
        self.current_state = TaskState.OPEN_GRIPPERS

    async def state_open_grippers(self):
        """State 7: 到达放置点，解除闭合与附着（detach），开启夹爪"""
        self.get_logger().info(">> State 7: open_grippers() - 下放物体并 detach 分离...")
        try:
            await self.stop_gripper_hold("进入放置阶段，准备开爪")
        except Exception:
            pass
        
        # 受控缓慢下放：循环步进直到接近放置高度，避免“下放不够就开爪”
        current_ee_z = await self._estimate_dual_ee_z()
        z_start = current_ee_z
        place_clearance = float(getattr(self, 'place_release_clearance', 0.010))
        place_extra_down = max(0.0, float(getattr(self, 'place_release_extra_down', 0.0)))
        target_release_ee_z = float(self.BAR_RESTING_Z + self.TCP_OFFSET + place_clearance - place_extra_down)

        descent_speed = max(0.005, float(getattr(self, 'place_descent_speed', 0.015)))
        max_step_down = max(0.02, abs(float(getattr(self, 'release_down_max_delta', -0.10))))
        target_tol = max(0.002, float(getattr(self, 'place_release_target_tol', 0.006)))
        max_iters = max(1, int(getattr(self, 'place_release_max_iters', 3)))
        min_total_down = max(0.01, float(getattr(self, 'place_release_min_total_down', 0.06)))

        self.get_logger().info(
            f"放置缓降目标: start_z={current_ee_z}, target_z={target_release_ee_z:.3f}, "
            f"step_max={max_step_down:.3f}, tol={target_tol:.3f}, max_iters={max_iters}")

        for attempt in range(1, max_iters + 1):
            if current_ee_z is None:
                step_delta = -min(max_step_down, abs(float(getattr(self, 'release_down_default_delta', -0.08))))
                self.get_logger().warn(
                    f"放置缓降[{attempt}/{max_iters}] 无法读取高度，采用默认步进 delta={step_delta:.3f}")
            else:
                remain = target_release_ee_z - float(current_ee_z)
                if remain >= -target_tol:
                    self.get_logger().info(
                        f"放置缓降已到位: current_z={current_ee_z:.4f}, target_z={target_release_ee_z:.4f}")
                    break
                step_delta = max(-max_step_down, remain)
                step_delta = min(-0.01, step_delta)

            step_duration = max(1.8, abs(step_delta) / descent_speed)
            self.get_logger().info(
                f"放置缓降[{attempt}/{max_iters}] current_z={current_ee_z}, "
                f"target_z={target_release_ee_z:.4f}, delta={step_delta:.4f}, duration={step_duration:.2f}s")

            step_ok = await self._move_dual_cartesian_delta_z(
                delta_z=step_delta,
                description=f"放置缓降步进#{attempt}(双臂笛卡尔)",
            )
            if not step_ok:
                self.get_logger().warn("放置笛卡尔步进失败，回退伺服步进")
                await self.servo_move_delta_z(step_delta, duration=step_duration)

            await asyncio.sleep(0.25)
            z_next = await self._estimate_dual_ee_z()
            if current_ee_z is not None and z_next is not None:
                moved = float(current_ee_z - z_next)
                self.get_logger().info(
                    f"[放置步进验收] moved={moved:.4f}, new_z={z_next:.4f}, "
                    f"remain={target_release_ee_z - float(z_next):.4f}")
            current_ee_z = z_next

        # 放置高度到位验收：不到位则禁止开爪，避免高空掉落
        if current_ee_z is not None:
            if current_ee_z > target_release_ee_z + target_tol:
                self.get_logger().error(
                    f"放置高度仍未达标: current_z={current_ee_z:.4f}, "
                    f"target_z<={target_release_ee_z + target_tol:.4f}，保持夹持并进入 ERROR")
                self.current_state = TaskState.ERROR
                return
            if z_start is not None:
                total_down = float(z_start - current_ee_z)
                if total_down < min_total_down:
                    self.get_logger().error(
                        f"放置总下放不足: total_down={total_down:.4f} < min_required={min_total_down:.4f}，"
                        f"保持夹持并进入 ERROR")
                    self.current_state = TaskState.ERROR
                    return
        else:
            self.get_logger().error("放置阶段无法验收末端高度，禁止开爪并进入 ERROR")
            self.current_state = TaskState.ERROR
            return
        
        self.get_logger().info("解除夹持目标，打开双臂夹爪...")
        await self.ensure_grippers_open()
        await asyncio.sleep(1.0)

        # 开爪后轻微回抬，避免手指拖拽物体造成二次冲击
        release_lift = max(0.0, float(getattr(self, 'post_release_lift_delta', 0.020)))
        if release_lift > 1e-4:
            await self.servo_move_delta_z(
                release_lift,
                duration=max(1.2, release_lift / max(0.005, float(getattr(self, 'place_retreat_speed', 0.018))))
            )
        
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

        # 先执行“退出上抬”并做位移验收，确保真正脱离桌面/物体后再回 Home
        retreat_delta = max(
            0.0,
            float(getattr(self, 'return_lift_delta', getattr(self, 'servo_phase_lift_delta', 0.10)))
        )
        retreat_speed = max(0.006, float(getattr(self, 'return_lift_speed', 0.02)))
        retreat_verify_min = max(0.002, float(getattr(self, 'return_lift_verify_min_abs', 0.015)))
        retreat_verify_ratio = max(0.10, min(0.95, float(getattr(self, 'return_lift_verify_ratio', 0.35))))
        retreat_retry_max = max(0.01, float(getattr(self, 'return_lift_retry_max', 0.06)))

        z_before = await self._estimate_dual_ee_z()
        retreat_ok = True
        moved_up = None
        if retreat_delta > 1e-6:
            retreat_duration = max(1.5, retreat_delta / retreat_speed)
            self.get_logger().info(
                f"退出上抬: start_z={z_before}, delta={retreat_delta:.3f}, duration={retreat_duration:.2f}s")

            retreat_ok = await self._move_dual_cartesian_delta_z(
                delta_z=retreat_delta,
                description="State8退出上抬(双臂笛卡尔)",
            )
            if not retreat_ok:
                self.get_logger().warn("State8 笛卡尔退出上抬失败，回退伺服上抬")
                retreat_ok = await self.servo_move_delta_z(retreat_delta, duration=retreat_duration)

            await asyncio.sleep(0.25)

            z_after = await self._estimate_dual_ee_z()
            can_verify = z_before is not None and z_after is not None
            required_up = max(retreat_verify_min, retreat_delta * retreat_verify_ratio)
            if can_verify:
                moved_up = float(z_after - z_before)
                self.get_logger().info(
                    f"[退出上抬验收] required>={required_up:.4f}, actual={moved_up:.4f}, "
                    f"start={z_before:.4f}, end={z_after:.4f}")
            else:
                self.get_logger().warn("[退出上抬验收] 无法获取有效 TF 高度，跳过位移验收")

            if can_verify and moved_up < required_up:
                retry_delta = min(retreat_retry_max, max(0.01, required_up - moved_up))
                self.get_logger().warn(
                    f"退出上抬不足，执行二次兜底上抬: retry_delta={retry_delta:.3f}")
                retry_ok = await self._move_dual_cartesian_delta_z(
                    delta_z=retry_delta,
                    description="State8退出上抬二次兜底(双臂笛卡尔)",
                )
                if not retry_ok:
                    await self._dual_delta_z_fallback(
                        delta_z=retry_delta,
                        description="State8退出上抬二次兜底(IK)",
                    )
                await asyncio.sleep(0.25)
                z_after2 = await self._estimate_dual_ee_z()
                if z_before is not None and z_after2 is not None:
                    moved_up = float(z_after2 - z_before)
                    self.get_logger().info(
                        f"[退出上抬二次验收] required>={required_up:.4f}, actual={moved_up:.4f}")

            # 保护逻辑：可验收但未真正上抬，不允许直接回 Home
            if can_verify and (moved_up is None or moved_up < required_up):
                msg = "退出上抬未生效，禁止直接回 Home"
                if self._is_force_continue_enabled():
                    self.get_logger().warn(f"{msg}，中期兜底模式下继续")
                else:
                    self.get_logger().error(msg)
                    self.current_state = TaskState.ERROR
                    return
            if (not retreat_ok) and (not can_verify):
                msg = "退出上抬命令失败且无法验收位移"
                if self._is_force_continue_enabled():
                    self.get_logger().warn(f"{msg}，中期兜底模式下继续")
                else:
                    self.get_logger().error(msg)
                    self.current_state = TaskState.ERROR
                    return
        else:
            self.get_logger().warn("退出上抬位移配置<=0，跳过上抬")

        await self.switch_to_trajectory_mode()
        
        self.get_logger().info("安全退避完毕，执行返回Home坐标轨迹.")
        if await self.move_to_home_joints():
            self.current_state = TaskState.FINISHED
        else:
            self.current_state = TaskState.ERROR


def main(args=None):
    rclpy.init(args=args)
    node = DualArmTaskNode()
