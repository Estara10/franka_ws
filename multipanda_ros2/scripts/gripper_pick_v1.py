#!/usr/bin/env python3
"""
双臂夹取系统 v1 - 直接关节轨迹控制版本（最可靠）

这个版本：
1. 不依赖MoveIt2规划（规避规划失败问题）
2. 使用直接关节轨迹控制（更稳定）
3. 逐步执行，便于调试
4. 包含完整的状态反馈

推荐流程：
  Terminal 1: ./start_interactive_sim.sh
  Terminal 2: source install/setup.bash && python3 src/multipanda_ros2/scripts/gripper_pick_v1.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import time
import math


class GripperPickSystem(Node):
    """直接关节控制的夹取系统"""
    
    def __init__(self):
        super().__init__('gripper_pick_system', callback_group=ReentrantCallbackGroup())
        
        # ==================== 关节定义 ====================
        self.left_joints = [
            'mj_left_joint1', 'mj_left_joint2', 'mj_left_joint3',
            'mj_left_joint4', 'mj_left_joint5', 'mj_left_joint6', 'mj_left_joint7'
        ]
        
        self.right_joints = [
            'mj_right_joint1', 'mj_right_joint2', 'mj_right_joint3',
            'mj_right_joint4', 'mj_right_joint5', 'mj_right_joint6', 'mj_right_joint7'
        ]
        
        # 夹爪关节（如果可用）
        self.left_gripper_joint = 'mj_left_finger_joint1'
        self.right_gripper_joint = 'mj_right_finger_joint1'
        
        # ==================== 状态存储 ====================
        self.current_joint_state = None
        self.current_left_positions = [0.0] * 7
        self.current_right_positions = [0.0] * 7
        self.current_left_gripper = 0.0
        self.current_right_gripper = 0.0
        
        # ==================== Action 客户端 ====================
        print("[INIT] 正在连接轨迹控制器...")
        self.trajectory_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/dual_panda_arm_controller/follow_joint_trajectory'
        )
        
        if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("❌ 轨迹控制器不可用! 请运行: ./start_interactive_sim.sh")
        
        print("[SUCCESS] ✓ 轨迹控制器连接成功")
        
        # ==================== 状态订阅 ====================
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        print("[INIT] ✓ 订阅关节状态")
        
        # 等待第一条消息
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_joint_state is not None:
                break
        
        if self.current_joint_state is None:
            raise RuntimeError("❌ 无法获取关节状态! 检查仿真是否运行")
        
        print("[SUCCESS] ✓ 已获得关节状态")
        self._print_current_state()
    
    def _joint_state_callback(self, msg):
        """更新当前关节状态"""
        self.current_joint_state = msg
        
        # 提取左臂位置
        for i, joint in enumerate(self.left_joints):
            if joint in msg.name:
                idx = msg.name.index(joint)
                self.current_left_positions[i] = msg.position[idx]
        
        # 提取右臂位置
        for i, joint in enumerate(self.right_joints):
            if joint in msg.name:
                idx = msg.name.index(joint)
                self.current_right_positions[i] = msg.position[idx]
        
        # 提取夹爪位置
        if self.left_gripper_joint in msg.name:
            idx = msg.name.index(self.left_gripper_joint)
            self.current_left_gripper = msg.position[idx]
        
        if self.right_gripper_joint in msg.name:
            idx = msg.name.index(self.right_gripper_joint)
            self.current_right_gripper = msg.position[idx]
    
    def _print_current_state(self):
        """打印当前状态"""
        print("\n" + "="*60)
        print("📊 当前关节状态：")
        print("="*60)
        
        print("左臂:")
        for i, pos in enumerate(self.current_left_positions):
            print(f"  joint{i+1}: {pos:7.4f} rad")
        
        print(f"\n左夹爪: {self.current_left_gripper:.4f} rad")
        
        print("\n右臂:")
        for i, pos in enumerate(self.current_right_positions):
            print(f"  joint{i+1}: {pos:7.4f} rad")
        
        print(f"\n右夹爪: {self.current_right_gripper:.4f} rad")
        print("="*60 + "\n")
    
    def execute_trajectory(self, joint_names, positions, duration=3.0, description="执行轨迹"):
        """
        执行单个轨迹
        
        Args:
            joint_names: 关节名称列表
            positions: 目标位置列表（对应joint_names）
            duration: 执行时间（秒）
            description: 任务描述
        
        Returns:
            bool: 是否成功
        """
        print(f"\n[ACTION] {description}...")
        print(f"  时间: {duration:.1f}s")
        
        # 创建轨迹
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        
        # 起点：当前位置
        start_point = JointTrajectoryPoint()
        start_point.positions = []
        for joint in joint_names:
            if joint in self.left_joints:
                idx = self.left_joints.index(joint)
                start_point.positions.append(self.current_left_positions[idx])
            elif joint in self.right_joints:
                idx = self.right_joints.index(joint)
                start_point.positions.append(self.current_right_positions[idx])
            elif joint == self.left_gripper_joint:
                start_point.positions.append(self.current_left_gripper)
            elif joint == self.right_gripper_joint:
                start_point.positions.append(self.current_right_gripper)
            else:
                start_point.positions.append(0.0)
        
        start_point.time_from_start = Duration(sec=0, nanosec=0)
        
        # 终点：目标位置
        end_point = JointTrajectoryPoint()
        end_point.positions = positions
        end_point.time_from_start = Duration(
            sec=int(duration), 
            nanosec=int((duration % 1.0) * 1e9)
        )
        
        trajectory.points = [start_point, end_point]
        
        # 发送轨迹
        try:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            
            print(f"  📤 发送轨迹到执行器...")
            future = self.trajectory_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=15)
            
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                print(f"  ❌ 轨迹被拒绝!")
                return False
            
            print(f"  ⏳ 等待执行完成...")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 10)
            
            result = result_future.result()
            if result and result.result.error_code == 0:
                print(f"  ✓ 完成!")
                time.sleep(0.5)  # 等待状态更新
                return True
            else:
                error_code = result.result.error_code if result else "Unknown"
                print(f"  ❌ 失败! 错误码: {error_code}")
                return False
        
        except Exception as e:
            print(f"  ❌ 异常: {e}")
            return False
    
    # ==================== 预定义动作 ====================
    
    def home_position(self):
        """回到初始位置"""
        # 标准初始位置
        left_home = [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]
        right_home = [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]
        
        self.execute_trajectory(
            self.left_joints + self.right_joints,
            left_home + right_home,
            duration=3.0,
            description="回到初始位置"
        )
    
    def open_grippers(self):
        """打开夹爪"""
        # 打开位置（较大的值）
        self.execute_trajectory(
            [self.left_gripper_joint, self.right_gripper_joint],
            [0.04, 0.04],  # 打开
            duration=1.0,
            description="打开夹爪"
        )
    
    def close_grippers(self):
        """关闭夹爪"""
        self.execute_trajectory(
            [self.left_gripper_joint, self.right_gripper_joint],
            [0.0, 0.0],  # 关闭
            duration=1.5,
            description="关闭夹爪"
        )
    
    def approach_object(self):
        """靠近物体（从两侧）"""
        # 物体在 x=0.5, z=0.45, y=±0.15
        # 使用 IK 解来靠近
        left_approach = [0.0, -1.0, 0.0, -2.0, 0.0, 1.0, 0.0]
        right_approach = [0.0, -1.0, 0.0, -2.0, 0.0, 1.0, 0.0]
        
        self.execute_trajectory(
            self.left_joints + self.right_joints,
            left_approach + right_approach,
            duration=2.0,
            description="靠近物体"
        )
    
    def pick_object(self):
        """执行夹取流程"""
        print("\n" + "="*60)
        print("🤖 开始双臂夹取演示")
        print("="*60)
        
        try:
            # 步骤1: 打开夹爪
            self.open_grippers()
            time.sleep(1)
            
            # 步骤2: 靠近物体
            self.approach_object()
            time.sleep(1)
            
            # 步骤3: 关闭夹爪
            self.close_grippers()
            time.sleep(1)
            
            # 步骤4: 抬起物体
            left_lift = [0.0, -0.5, 0.0, -2.356, 0.0, 1.57, 0.785]
            right_lift = [0.0, -0.5, 0.0, -2.356, 0.0, 1.57, 0.785]
            
            self.execute_trajectory(
                self.left_joints + self.right_joints,
                left_lift + right_lift,
                duration=2.0,
                description="抬起物体"
            )
            time.sleep(1)
            
            # 步骤5: 打开夹爪放下
            self.open_grippers()
            time.sleep(1)
            
            # 步骤6: 回到初始位置
            self.home_position()
            
            print("\n" + "="*60)
            print("✅ 夹取演示完成!")
            print("="*60)
            return True
        
        except Exception as e:
            print(f"\n❌ 演示失败: {e}")
            return False


def main():
    rclpy.init()
    
    try:
        print("\n" + "🚀 双臂夹取系统 v1 初始化...")
        system = GripperPickSystem()
        
        # 打印当前状态
        system._print_current_state()
        
        # 执行夹取
        system.pick_object()
        
    except KeyboardInterrupt:
        print("\n⚠️  用户中断")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
