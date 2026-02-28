#!/usr/bin/env python3
#靶向消息诊断。用于在发送动作目标（Goal）前检查目标位姿（Pose）和约束消息（Constraints）拼装是否合法。
"""
检查目标消息内容的诊断脚本
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetMotionPlan
import time


class TargetDiagnostics(Node):
    def __init__(self):
        super().__init__('target_diagnostics')
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        print("[INFO] 等待 MoveIt2 服务器...")
        if not self.move_group_client.wait_for_server(timeout_sec=15.0):
            print("[ERROR] MoveIt2 不可用")
            raise RuntimeError("No server")
        
        # 打印 action server 信息
        print(f"[SUCCESS] 连接到: {self.move_group_client._action_name}")
        print(f"服务器可用: {self.move_group_client.server_is_ready()}")

    def create_simple_goal(self):
        """创建最简单的目标"""
        goal = MoveGroup.Goal()
        
        # 基本配置
        goal.request.group_name = "panda_1"
        goal.request.allowed_planning_time = 10.0
        goal.request.num_planning_attempts = 5
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        
        # 添加一个关节约束
        constraints = Constraints()
        jc = JointConstraint()
        jc.joint_name = "mj_left_joint1"
        jc.position = 0.1
        jc.tolerance_above = 0.1
        jc.tolerance_below = 0.1
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(constraints)
        
        return goal

    def test_send_goal(self):
        """发送目标并诊断"""
        print("\n创建目标消息...")
        goal = self.create_simple_goal()
        
        print(f"  组名: {goal.request.group_name}")
        print(f"  规划时间: {goal.request.allowed_planning_time} s")
        print(f"  规划次数: {goal.request.num_planning_attempts}")
        print(f"  约束数: {len(goal.request.goal_constraints)}")
        print(f"  关节约束数: {len(goal.request.goal_constraints[0].joint_constraints)}")
        
        if goal.request.goal_constraints[0].joint_constraints:
            jc = goal.request.goal_constraints[0].joint_constraints[0]
            print(f"    - {jc.joint_name}: {jc.position:.4f} rad (tolerance: ±{jc.tolerance_above:.4f})")
        
        print("\n发送目标...")
        try:
            future = self.move_group_client.send_goal_async(goal)
            print(f"✓ 发送成功，future: {future}")
            
            print("等待目标被接受...")
            rclpy.spin_until_future_complete(self, future, timeout_sec=15)
            
            try:
                goal_handle = future.result()
            except Exception as e:
                print(f"❌ result() 异常: {e}")
                goal_handle = None
            
            print(f"目标句柄: {goal_handle}")
            
            if not goal_handle:
                print("❌ 目标句柄为 None")
                print("尝试检查 future 状态...")
                print(f"  future done: {future.done()}")
                print(f"  future cancelled: {future.cancelled()}")
                if future.exception():
                    print(f"  future exception: {future.exception()}")
                return
            
            print(f"✓ 目标已接受: {goal_handle.accepted}")
            
            if not goal_handle.accepted:
                print("❌ 目标被拒绝")
                return
            
            print("\n等待执行结果...")
            result_future = goal_handle.get_result_async()
            print(f"result_future: {result_future}")
            
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=30)
            
            try:
                result = result_future.result()
            except Exception as e:
                print(f"❌ result() 异常: {e}")
                result = None
            
            if not result:
                print("❌ 结果为 None")
                return
            
            print(f"✓ 结果类型: {type(result)}")
            print(f"  result: {result.result}")
            print(f"  error_code: {result.result.error_code.val}")
            
        except Exception as e:
            print(f"❌ 异常: {e}")
            import traceback
            traceback.print_exc()


def main():
    rclpy.init()
    
    try:
        diag = TargetDiagnostics()
        diag.test_send_goal()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
