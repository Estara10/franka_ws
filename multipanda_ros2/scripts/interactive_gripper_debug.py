#!/usr/bin/env python3
#交互式命令行调试工具。你可以直接在终端里面敲命令（例如：open打开夹爪、close关闭夹爪、approach靠近、
# move L ...控制左臂单动）。主要帮助你在开发中找最佳的夹取点和关节姿态。
"""
双臂夹取系统 - 交互式调试工具

用途：逐步调试每个动作，找到最优的关节角度

使用方法：
  python3 interactive_gripper_debug.py
  
  命令：
    home          - 回到初始位置
    approach      - 靠近物体
    open          - 打开夹爪
    close         - 关闭夹爪
    pick          - 执行完整夹取
    set j0 0.5    - 设置关节0到0.5弧度
    move L 0.1 -1.0 0.0 ...  - 移动左臂到指定关节角度
    move R 0.1 -1.0 0.0 ...  - 移动右臂到指定关节角度
    state         - 打印当前状态
    exit          - 退出
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

import time
import threading


class InteractiveDebugger(Node):
    """交互式调试工具"""
    
    def __init__(self):
        super().__init__('interactive_debugger', callback_group=ReentrantCallbackGroup())
        
        self.left_joints = [
            'mj_left_joint1', 'mj_left_joint2', 'mj_left_joint3',
            'mj_left_joint4', 'mj_left_joint5', 'mj_left_joint6', 'mj_left_joint7'
        ]
        
        self.right_joints = [
            'mj_right_joint1', 'mj_right_joint2', 'mj_right_joint3',
            'mj_right_joint4', 'mj_right_joint5', 'mj_right_joint6', 'mj_right_joint7'
        ]
        
        self.left_gripper_joint = 'mj_left_finger_joint1'
        self.right_gripper_joint = 'mj_right_finger_joint1'
        
        # 状态
        self.current_joint_state = None
        self.current_left_positions = [0.0] * 7
        self.current_right_positions = [0.0] * 7
        self.current_left_gripper = 0.0
        self.current_right_gripper = 0.0
        
        # 连接轨迹控制器
        print("[INIT] 连接轨迹控制器...")
        self.trajectory_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/dual_panda_arm_controller/follow_joint_trajectory'
        )
        
        if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("❌ 轨迹控制器不可用!")
        
        print("[SUCCESS] ✓ 轨迹控制器连接成功")
        
        # 订阅关节状态
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_joint_state is not None:
                break
        
        if self.current_joint_state is None:
            raise RuntimeError("❌ 无法获取关节状态!")
        
        print("[SUCCESS] ✓ 系统就绪")
        self.print_state()
    
    def _joint_state_callback(self, msg):
        self.current_joint_state = msg
        for i, joint in enumerate(self.left_joints):
            if joint in msg.name:
                idx = msg.name.index(joint)
                self.current_left_positions[i] = msg.position[idx]
        for i, joint in enumerate(self.right_joints):
            if joint in msg.name:
                idx = msg.name.index(joint)
                self.current_right_positions[i] = msg.position[idx]
        if self.left_gripper_joint in msg.name:
            idx = msg.name.index(self.left_gripper_joint)
            self.current_left_gripper = msg.position[idx]
        if self.right_gripper_joint in msg.name:
            idx = msg.name.index(self.right_gripper_joint)
            self.current_right_gripper = msg.position[idx]
    
    def print_state(self):
        print("\n" + "="*60)
        print("📊 当前状态：")
        print("="*60)
        print("左臂:")
        for i, pos in enumerate(self.current_left_positions):
            print(f"  L{i+1}: {pos:7.4f} rad ({math.degrees(pos):7.2f}°)")
        print(f"左夹爪: {self.current_left_gripper:.4f} rad")
        
        print("\n右臂:")
        for i, pos in enumerate(self.current_right_positions):
            print(f"  R{i+1}: {pos:7.4f} rad ({math.degrees(pos):7.2f}°)")
        print(f"右夹爪: {self.current_right_gripper:.4f} rad")
        print("="*60 + "\n")
    
    def execute_trajectory(self, joint_names, positions, duration=2.0):
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        
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
        
        end_point = JointTrajectoryPoint()
        end_point.positions = positions
        end_point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1.0) * 1e9))
        
        trajectory.points = [start_point, end_point]
        
        try:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            
            print(f"📤 执行轨迹 (时间: {duration}s)...")
            future = self.trajectory_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=15)
            
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                print("❌ 轨迹被拒绝!")
                return
            
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 10)
            
            result = result_future.result()
            if result and result.result.error_code == 0:
                print("✓ 完成!")
                time.sleep(0.5)
            else:
                print(f"❌ 失败! 错误码: {result.result.error_code if result else 'Unknown'}")
        
        except Exception as e:
            print(f"❌ 异常: {e}")
    
    # 预定义命令
    def cmd_home(self):
        left_home = [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]
        right_home = [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]
        self.execute_trajectory(self.left_joints + self.right_joints, left_home + right_home, duration=3.0)
    
    def cmd_approach(self):
        left_approach = [0.0, -1.0, 0.0, -2.0, 0.0, 1.0, 0.0]
        right_approach = [0.0, -1.0, 0.0, -2.0, 0.0, 1.0, 0.0]
        self.execute_trajectory(self.left_joints + self.right_joints, left_approach + right_approach, duration=2.0)
    
    def cmd_open(self):
        self.execute_trajectory([self.left_gripper_joint, self.right_gripper_joint], [0.04, 0.04], duration=1.0)
    
    def cmd_close(self):
        self.execute_trajectory([self.left_gripper_joint, self.right_gripper_joint], [0.0, 0.0], duration=1.5)
    
    def cmd_pick(self):
        print("\n执行夹取流程...")
        self.cmd_open()
        time.sleep(1)
        self.cmd_approach()
        time.sleep(1)
        self.cmd_close()
        time.sleep(1)
        left_lift = [0.0, -0.5, 0.0, -2.356, 0.0, 1.57, 0.785]
        right_lift = [0.0, -0.5, 0.0, -2.356, 0.0, 1.57, 0.785]
        self.execute_trajectory(self.left_joints + self.right_joints, left_lift + right_lift, duration=2.0)
        time.sleep(1)
        self.cmd_open()
        time.sleep(1)
        self.cmd_home()
    
    def cmd_move_arm(self, side, *args):
        if len(args) != 7:
            print("❌ 需要7个关节角度值")
            return
        
        positions = [float(x) for x in args]
        
        if side.upper() == 'L':
            self.execute_trajectory(self.left_joints, positions, duration=2.0)
        elif side.upper() == 'R':
            self.execute_trajectory(self.right_joints, positions, duration=2.0)
        else:
            print("❌ 使用 'move L' 或 'move R'")
    
    def process_command(self, cmd_line):
        parts = cmd_line.strip().split()
        if not parts:
            return
        
        cmd = parts[0].lower()
        args = parts[1:]
        
        if cmd == 'home':
            self.cmd_home()
        elif cmd == 'approach':
            self.cmd_approach()
        elif cmd == 'open':
            self.cmd_open()
        elif cmd == 'close':
            self.cmd_close()
        elif cmd == 'pick':
            self.cmd_pick()
        elif cmd == 'state':
            self.print_state()
        elif cmd == 'move':
            if args:
                self.cmd_move_arm(args[0], *args[1:])
            else:
                print("用法: move L 0.0 -0.785 0.0 -2.356 0.0 1.57 0.785")
        elif cmd == 'exit':
            return False
        else:
            print(f"❌ 未知命令: {cmd}")
            print("可用命令: home, approach, open, close, pick, move, state, exit")
        
        return True


def main():
    import math
    
    rclpy.init()
    executor = MultiThreadedExecutor()
    
    try:
        print("\n" + "="*60)
        print("🔧 双臂夹取系统 - 交互式调试工具")
        print("="*60)
        
        debugger = InteractiveDebugger()
        executor.add_node(debugger)
        
        print("\n输入 'help' 查看命令列表")
        print("输入 'exit' 退出\n")
        
        while True:
            try:
                cmd = input(">> ").strip()
                if not debugger.process_command(cmd):
                    break
            except KeyboardInterrupt:
                print("\n⚠️  中断")
                break
            except Exception as e:
                print(f"❌ 错误: {e}")
    
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    import math
    main()
