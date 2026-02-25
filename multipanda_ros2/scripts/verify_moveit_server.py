#!/usr/bin/env python3
"""
检查 MoveIt2 服务器是否正确启动
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
import time

class MoveItServerVerifier(Node):
    def __init__(self):
        super().__init__('moveit_verifier')
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
    def verify(self):
        print("\n" + "="*60)
        print("🔍 检查 MoveIt2 服务器状态")
        print("="*60)
        
        # 检查 action server 是否存在
        print("\n[步骤 1] 等待 /move_action action server...")
        for i in range(10):
            if self.move_group_client.wait_for_server(timeout_sec=1.0):
                print(f"✅ Action server 可用！")
                break
            print(f"   尝试 {i+1}/10... (等待中)")
        else:
            print("❌ /move_action action server 不可用！")
            print("\n💡 解决方案:")
            print("   1. 确保 MoveIt2 已启动: ros2 launch franka_bringup ...")
            print("   2. 或检查日志: ros2 topic list | grep move")
            print("   3. 或运行诊断脚本: check_system.py")
            return False
        
        # 列出所有 action server
        print("\n[步骤 2] 列出可用的 action servers...")
        self.list_action_servers()
        
        # 列出所有 topics
        print("\n[步骤 3] 列出关键 topics...")
        self.list_key_topics()
        
        return True
    
    def list_action_servers(self):
        import subprocess
        try:
            result = subprocess.run(['ros2', 'action', 'list'], 
                                   capture_output=True, text=True, timeout=5)
            actions = result.stdout.strip().split('\n')
            important_actions = [a for a in actions if 'move' in a or 'gripper' in a]
            
            if important_actions:
                for action in important_actions:
                    print(f"   ✓ {action}")
            else:
                print(f"   Found {len(actions)} actions total (listing all):")
                for action in actions[:10]:
                    if action:
                        print(f"   • {action}")
        except Exception as e:
            print(f"   ⚠️  无法列出 actions: {e}")
    
    def list_key_topics(self):
        import subprocess
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                   capture_output=True, text=True, timeout=5)
            topics = result.stdout.strip().split('\n')
            
            key_patterns = ['move', 'joint', 'robot_state', 'gripper']
            key_topics = [t for t in topics if any(p in t for p in key_patterns)]
            
            if key_topics:
                for topic in key_topics[:15]:
                    print(f"   ✓ {topic}")
            else:
                print(f"   Found {len(topics)} topics total (no 'move' related)")
        except Exception as e:
            print(f"   ⚠️  无法列出 topics: {e}")

def main():
    rclpy.init()
    
    try:
        verifier = MoveItServerVerifier()
        success = verifier.verify()
        
        if success:
            print("\n" + "="*60)
            print("✅ MoveIt2 服务器检查通过！")
            print("="*60)
            print("\n可以运行: python3 dual_arm_demo.py")
        else:
            print("\n" + "="*60)
            print("❌ MoveIt2 服务器检查失败")
            print("="*60)
            
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
