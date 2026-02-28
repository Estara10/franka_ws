#!/usr/bin/env python3
#moveit_error_diagnosis.py：专门捕获和诊断 MoveIt2 返回的错误码（例如由于碰撞检测或无法求取逆运动学导致的规划失败）。
"""
MoveIt2 错误码诊断脚本
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
import time


class MoveItErrorDiagnostics(Node):
    def __init__(self):
        super().__init__('moveit_diagnostics')
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        print("[INFO] 等待 MoveIt2 服务器...")
        if not self.move_group_client.wait_for_server(timeout_sec=15.0):
            print("[ERROR] MoveIt2 服务器不可用")
            raise RuntimeError("No server")
        print("[SUCCESS] 服务器就绪")

    def test_movement(self):
        """测试一个简单的运动"""
        print("\n" + "="*60)
        print("测试单个关节移动")
        print("="*60)
        
        # 非常小的运动
        goal = MoveGroup.Goal()
        goal.request.group_name = "panda_1"
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 10
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        
        # 获取当前状态第一个关节值
        current_value = 0.05669774571649041  # 从上面的输出中获取
        
        # 设置目标为极小的位置变化
        constraints = Constraints()
        jc = JointConstraint()
        jc.joint_name = "mj_left_joint1"
        jc.position = current_value + 0.01  # 只移动 0.01 弧度
        jc.tolerance_above = 0.05
        jc.tolerance_below = 0.05
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(constraints)
        
        print(f"\n发送目标: {jc.joint_name} = {jc.position:.4f} rad")
        
        try:
            future = self.move_group_client.send_goal_async(goal)
            print("等待规划结果...")
            rclpy.spin_until_future_complete(self, future, timeout_sec=15)
            
            goal_handle = future.result()
            if not goal_handle:
                print("❌ 没有收到 goal_handle")
                return
            
            if not goal_handle.accepted:
                print("❌ 目标被拒绝")
                return
            
            print("✓ 目标被接受")
            
            result_future = goal_handle.get_result_async()
            print("等待执行结果...")
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=30)
            
            result = result_future.result()
            if not result:
                print("❌ 没有结果")
                return
            
            error_code = result.result.error_code.val
            print(f"返回码: {error_code}")
            
            # 查看错误码含义
            error_names = {
                1: "SUCCESS",
                99999: "FAILURE (通用失败)",
                2: "PLANNING_FAILED",
                3: "INVALID_MOTION_PLAN",
                4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
                5: "CONTROL_FAILED",
                6: "TIMED_OUT",
                7: "PREEMPTED",
                8: "START_STATE_IN_COLLISION",
                9: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
                10: "GOAL_IN_COLLISION",
                11: "GOAL_VIOLATES_PATH_CONSTRAINTS",
                12: "GOAL_CONSTRAINTS_VIOLATED",
                13: "INVALID_GROUP_NAME",
                14: "INVALID_GOAL_CONSTRAINTS",
                15: "INVALID_ROBOT_STATE",
                16: "INVALID_LINK_NAME",
                17: "INVALID_OBJECT_NAME",
                18: "FRAME_TRANSFORM_FAILURE",
                19: "COLLISION_CHECKING_UNAVAILABLE",
                20: "ROBOT_STATE_STALE",
                21: "SENSOR_INFO_STALE",
                22: "NO_IK_SOLUTION",
            }
            
            error_name = error_names.get(error_code, f"未知错误码 {error_code}")
            print(f"错误含义: {error_name}")
            
            # 检查消息
            if hasattr(result.result, 'error_message'):
                print(f"错误消息: {result.result.error_message}")
            
            # 检查规划结果
            if hasattr(result, 'action_result'):
                print(f"动作结果属性: {dir(result.action_result)}")
                
        except Exception as e:
            print(f"❌ 异常: {e}")
            import traceback
            traceback.print_exc()


def main():
    rclpy.init()
    
    try:
        diag = MoveItErrorDiagnostics()
        diag.test_movement()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
