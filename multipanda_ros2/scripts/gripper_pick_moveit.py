#!/usr/bin/env python3
"""
双臂机械夹取系统 - MoveIt2规划版本

使用MoveIt2规划框架实现机械臂夹取
- 支持完整的夹取流程
- 使用MoveIt2轨迹规划
- 包含碰撞检测和约束

推荐启动流程：
  Terminal 1: ./start_interactive_sim.sh
  Terminal 2: source install/setup.bash && python3 src/multipanda_ros2/scripts/gripper_pick_moveit.py
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, CollisionObject
from moveit_msgs.action import MoveGroup
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient

import time
import math


class GripperPickMoveIt(Node):
    """基于MoveIt2的双臂夹取系统"""
    
    def __init__(self):
        super().__init__('gripper_pick_moveit', callback_group=ReentrantCallbackGroup())
        
        print("[INIT] 初始化MoveIt2夹取系统...")
        
        # ==================== 关节定义 ====================
        self.left_joints = [
            'mj_left_joint1', 'mj_left_joint2', 'mj_left_joint3',
            'mj_left_joint4', 'mj_left_joint5', 'mj_left_joint6', 'mj_left_joint7'
        ]
        
        self.right_joints = [
            'mj_right_joint1', 'mj_right_joint2', 'mj_right_joint3',
            'mj_right_joint4', 'mj_right_joint5', 'mj_right_joint6', 'mj_right_joint7'
        ]
        
        # ==================== 状态 ====================
        self.current_joint_state = None
        self.current_left_positions = [0.0] * 7
        self.current_right_positions = [0.0] * 7
        
        # ==================== MoveIt2 Action客户端 ====================
        print("[INIT] 连接MoveIt2...")
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        if not self.move_group_client.wait_for_server(timeout_sec=15.0):
            print("[ERROR] MoveIt2服务器不可用!")
            raise RuntimeError("MoveIt2 Action Server not available")
        
        print("[SUCCESS] ✓ MoveIt2连接成功")
        
        # ==================== 夹爪Action客户端 ====================
        self.left_gripper = ActionClient(
            self, GripperCommand, 
            '/mj_left_gripper_sim_node/gripper_action'
        )
        self.right_gripper = ActionClient(
            self, GripperCommand, 
            '/mj_right_gripper_sim_node/gripper_action'
        )
        
        # ==================== 订阅关节状态 ====================
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        
        # 等待第一条消息
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_joint_state is not None:
                break
        
        if self.current_joint_state is None:
            raise RuntimeError("无法获取关节状态!")
        
        print("[SUCCESS] ✓ 系统初始化完成")
    
    def _joint_state_callback(self, msg):
        """更新关节状态"""
        self.current_joint_state = msg
        
        for i, joint in enumerate(self.left_joints):
            if joint in msg.name:
                self.current_left_positions[i] = msg.position[msg.name.index(joint)]
        
        for i, joint in enumerate(self.right_joints):
            if joint in msg.name:
                self.current_right_positions[i] = msg.position[msg.name.index(joint)]
    
    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """欧拉角转四元数"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy
        )
    
    def plan_and_move(self, group_name, target_pose, planning_time=10.0, 
                      description="规划运动"):
        """
        使用MoveIt2规划并执行运动
        
        Args:
            group_name: 运动组名称（"mj_left_arm" 或 "mj_right_arm"）
            target_pose: 目标位置 (Pose对象)
            planning_time: 规划时间（秒）
            description: 任务描述
            
        Returns:
            bool: 是否成功
        """
        print(f"\n[{group_name}] {description}...")
        print(f"  目标位置: x={target_pose.position.x:.3f}, "
              f"y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}")
        
        try:
            # 创建MoveGroup目标
            goal = MoveGroup.Goal()
            goal.request.group_name = group_name
            goal.request.allowed_planning_time = planning_time
            goal.request.num_planning_attempts = 10
            goal.request.max_velocity_scaling_factor = 1.0
            goal.request.max_acceleration_scaling_factor = 1.0
            
            # 工作空间设置
            goal.request.workspace_parameters.header.frame_id = "world"
            ws = goal.request.workspace_parameters
            ws.min_corner.x = -2.0
            ws.min_corner.y = -2.0
            ws.min_corner.z = -1.0
            ws.max_corner.x = 3.0
            ws.max_corner.y = 3.0
            ws.max_corner.z = 3.0
            
            # 添加位置约束
            pc = PositionConstraint()
            pc.header.frame_id = "world"
            pc.link_name = "mj_left_link8" if "left" in group_name.lower() else "mj_right_link8"
            
            # 使用球体约束
            pc.constraint_region.primitives.append(
                SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.1])
            )
            pc.constraint_region.primitive_poses.append(target_pose)
            pc.weight = 1.0
            
            constraints = Constraints()
            constraints.position_constraints.append(pc)
            goal.request.goal_constraints.append(constraints)
            
            # 发送规划请求
            print(f"  📤 发送规划请求...")
            future = self.move_group_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=20)
            
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                print(f"  ❌ 规划被拒绝")
                return False
            
            print(f"  ✓ 规划成功，等待执行...")
            
            # 等待执行完成
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, 
                                            timeout_sec=planning_time + 30)
            
            result = result_future.result()
            if not result:
                print(f"  ❌ 执行超时")
                return False
            
            error_code = result.result.error_code.val
            if error_code == 1:  # SUCCESS
                print(f"  ✓ 执行成功!")
                time.sleep(0.5)
                return True
            else:
                print(f"  ⚠️  执行完成，错误码: {error_code}")
                return False
        
        except Exception as e:
            print(f"  ❌ 异常: {e}")
            return False
    
    def control_gripper(self, side, position, force=50.0):
        """控制夹爪"""
        client = self.left_gripper if side == "left" else self.right_gripper
        
        try:
            if not client.wait_for_server(timeout_sec=2.0):
                print(f"[{side}] 夹爪不可用")
                return False
            
            goal = GripperCommand.Goal()
            goal.command.position = position
            goal.command.max_effort = force
            
            future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5)
            
            goal_handle = future.result()
            if goal_handle and goal_handle.accepted:
                print(f"[{side}] 夹爪指令: {'打开' if position > 0.02 else '关闭'}")
                return True
            return False
        
        except Exception as e:
            print(f"[{side}] 错误: {e}")
            return False
    
    def pick_with_moveit(self):
        """使用MoveIt2执行夹取任务"""
        print("\n" + "="*60)
        print("🤖 开始MoveIt2夹取演示")
        print("="*60)
        
        try:
            # 物体位置
            object_x, object_y, object_z = 0.5, 0.0, 0.45
            approach_height = 0.55
            grasp_height = 0.48
            lift_height = 0.65
            
            # 步骤1: 打开夹爪
            print("\n[步骤1] 打开夹爪...")
            self.control_gripper("left", 0.04)
            self.control_gripper("right", 0.04)
            time.sleep(1)
            
            # 步骤2: 规划靠近物体
            print("\n[步骤2] 规划靠近物体...")
            
            left_approach = Pose()
            left_approach.position.x = object_x
            left_approach.position.y = object_y + 0.15
            left_approach.position.z = approach_height
            qx, qy, qz, qw = self.euler_to_quaternion(math.pi, 0.0, 0.0)
            left_approach.orientation.x = qx
            left_approach.orientation.y = qy
            left_approach.orientation.z = qz
            left_approach.orientation.w = qw
            
            right_approach = Pose()
            right_approach.position.x = object_x
            right_approach.position.y = object_y - 0.15
            right_approach.position.z = approach_height
            right_approach.orientation.x = qx
            right_approach.orientation.y = qy
            right_approach.orientation.z = qz
            right_approach.orientation.w = qw
            
            self.plan_and_move("mj_left_arm", left_approach, description="左臂靠近物体")
            time.sleep(0.5)
            self.plan_and_move("mj_right_arm", right_approach, description="右臂靠近物体")
            time.sleep(1)
            
            # 步骤3: 规划下降到抓取位置
            print("\n[步骤3] 规划下降抓取...")
            
            left_grasp = Pose()
            left_grasp.position.x = object_x
            left_grasp.position.y = object_y + 0.15
            left_grasp.position.z = grasp_height
            left_grasp.orientation.x = qx
            left_grasp.orientation.y = qy
            left_grasp.orientation.z = qz
            left_grasp.orientation.w = qw
            
            right_grasp = Pose()
            right_grasp.position.x = object_x
            right_grasp.position.y = object_y - 0.15
            right_grasp.position.z = grasp_height
            right_grasp.orientation.x = qx
            right_grasp.orientation.y = qy
            right_grasp.orientation.z = qz
            right_grasp.orientation.w = qw
            
            self.plan_and_move("mj_left_arm", left_grasp, description="左臂下降")
            time.sleep(0.5)
            self.plan_and_move("mj_right_arm", right_grasp, description="右臂下降")
            time.sleep(1)
            
            # 步骤4: 关闭夹爪
            print("\n[步骤4] 关闭夹爪...")
            self.control_gripper("left", 0.0)
            self.control_gripper("right", 0.0)
            time.sleep(1.5)
            
            # 步骤5: 规划抬起
            print("\n[步骤5] 规划抬起物体...")
            
            left_lift = Pose()
            left_lift.position.x = object_x
            left_lift.position.y = object_y + 0.15
            left_lift.position.z = lift_height
            left_lift.orientation.x = qx
            left_lift.orientation.y = qy
            left_lift.orientation.z = qz
            left_lift.orientation.w = qw
            
            right_lift = Pose()
            right_lift.position.x = object_x
            right_lift.position.y = object_y - 0.15
            right_lift.position.z = lift_height
            right_lift.orientation.x = qx
            right_lift.orientation.y = qy
            right_lift.orientation.z = qz
            right_lift.orientation.w = qw
            
            self.plan_and_move("mj_left_arm", left_lift, description="左臂抬起")
            time.sleep(0.5)
            self.plan_and_move("mj_right_arm", right_lift, description="右臂抬起")
            time.sleep(1)
            
            # 步骤6: 打开夹爪放下
            print("\n[步骤6] 打开夹爪放下...")
            self.control_gripper("left", 0.04)
            self.control_gripper("right", 0.04)
            time.sleep(1)
            
            print("\n" + "="*60)
            print("✅ MoveIt2夹取演示完成!")
            print("="*60)
            return True
        
        except Exception as e:
            print(f"\n❌ 演示失败: {e}")
            import traceback
            traceback.print_exc()
            return False


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    
    try:
        print("\n" + "="*60)
        print("🚀 MoveIt2机械臂夹取系统")
        print("="*60 + "\n")
        
        picker = GripperPickMoveIt()
        executor.add_node(picker)
        
        # 执行夹取任务
        picker.pick_with_moveit()
        
    except KeyboardInterrupt:
        print("\n⚠️  用户中断")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
