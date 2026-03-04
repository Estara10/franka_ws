#!/usr/bin/env python3
#伺服，柔顺控制

import asyncio

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


class ServoOpsMixin:
    def _build_scurve_scale(self, ratio: float) -> float:
        ratio = max(0.0, min(1.0, float(ratio)))
        return 0.5 * (1.0 - np.cos(np.pi * ratio))

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
        
        # 阻抗模型参数（可由 test2.py 覆盖，默认低晃动优先）
        Kd_z = max(1.0, float(getattr(self, 'compliance_kd_z', 340.0)))
        Kd_y = max(1.0, float(getattr(self, 'compliance_kd_y', 280.0)))
        Kp_dist = float(getattr(self, 'compliance_kp_dist', 0.45))
        force_y_threshold = float(getattr(self, 'compliance_force_y_threshold', 28.0))
        force_gain = float(getattr(self, 'compliance_force_gain', 0.85))
        alpha = max(0.5, min(0.98, float(getattr(self, 'compliance_lpf_alpha', 0.94))))
        max_v_z = max(0.002, float(getattr(self, 'compliance_max_v_z', 0.020)))
        max_v_y = max(0.002, float(getattr(self, 'compliance_max_v_y', 0.018)))
        
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
                target_v_z = (delta_f_z / Kd_z) * force_gain + dist_p_term_z
                self.compliance_v_z = alpha * self.compliance_v_z + (1.0 - alpha) * target_v_z
            else:
                # 无显著力差时，仅靠弹簧项维持间距同步
                self.compliance_v_z = alpha * self.compliance_v_z + (1.0 - alpha) * dist_p_term_z
                
            # ---- Y轴柔顺补偿 ----
            # 内应力超过 20N 阈值时，向外（远离物体）释放
            if delta_f_y > force_y_threshold:
                target_v_y = ((delta_f_y - force_y_threshold) / Kd_y) * force_gain + dist_p_term_y
                # 符号取反：内应力增大 -> 从臂向外移动（y 变小）
                self.compliance_v_y = alpha * self.compliance_v_y + (1.0 - alpha) * (-target_v_y)
            else:
                self.compliance_v_y = alpha * self.compliance_v_y + (1.0 - alpha) * dist_p_term_y

            self.compliance_v_z = max(-max_v_z, min(max_v_z, float(self.compliance_v_z)))
            self.compliance_v_y = max(-max_v_y, min(max_v_y, float(self.compliance_v_y)))

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

        target_delta_z = 0.0
        if phase == 'lift':
            target_delta_z = float(getattr(self, 'servo_phase_lift_delta', 0.15))
        elif phase == 'release_down':
            target_delta_z = float(getattr(self, 'servo_phase_release_down_delta', -0.10))

        return await self.servo_move_delta_z(target_delta_z, duration=duration)

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
        v_nominal = float(delta_z) / duration
        ramp_ratio = max(0.05, min(0.45, float(getattr(self, 'servo_ramp_ratio', 0.20))))
        ramp_steps = max(1, int(steps * ramp_ratio))

        left_twist = TwistStamped()
        right_twist = TwistStamped()
        left_twist.header.frame_id = self.planning_frame
        right_twist.header.frame_id = self.planning_frame

        for i in range(steps):
            if i < ramp_steps:
                scale = self._build_scurve_scale(float(i + 1) / float(ramp_steps))
            elif i >= steps - ramp_steps:
                remain = max(0, steps - i)
                scale = self._build_scurve_scale(float(remain) / float(ramp_steps))
            else:
                scale = 1.0
            v_z = v_nominal * scale

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

