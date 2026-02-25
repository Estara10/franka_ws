#!/usr/bin/env python3
"""
系统诊断脚本 - 检查 ROS2 系统状态
"""
import subprocess
import sys

def run_cmd(cmd):
    """运行命令并返回输出"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, timeout=5, text=True)
        return result.stdout.strip()
    except:
        return None

def check_nodes():
    """检查正在运行的节点"""
    print("\n" + "="*60)
    print("📋 正在运行的 ROS2 节点：")
    print("="*60)
    output = run_cmd("ros2 node list 2>/dev/null")
    if output:
        for line in output.split('\n'):
            print(f"  {line}")
    else:
        print("  ❌ 无法列出节点（ROS2 环境可能未初始化）")

def check_actions():
    """检查可用的 Actions"""
    print("\n" + "="*60)
    print("🎯 可用的 Action 服务器：")
    print("="*60)
    output = run_cmd("ros2 action list 2>/dev/null")
    if output:
        for line in output.split('\n'):
            if line:
                print(f"  {line}")
    else:
        print("  ❌ 无法列出 Actions")

def check_topics():
    """检查关键 Topics"""
    print("\n" + "="*60)
    print("📡 关键 Topics：")
    print("="*60)
    
    topics_to_check = [
        '/joint_states',
        '/robot_description',
        '/clock',
        '/move_group/status',
        '/move_group/feedback'
    ]
    
    output = run_cmd("ros2 topic list 2>/dev/null")
    if output:
        available_topics = output.split('\n')
        for topic in topics_to_check:
            if topic in available_topics:
                print(f"  ✓ {topic}")
            else:
                print(f"  ✗ {topic} (缺失)")
    else:
        print("  ❌ 无法列出 Topics")

def check_move_group():
    """检查 MoveGroup 服务器"""
    print("\n" + "="*60)
    print("🤖 MoveGroup 服务器状态：")
    print("="*60)
    
    # 检查 /move_group action
    actions = run_cmd("ros2 action list 2>/dev/null")
    if actions and '/move_group' in actions:
        print("  ✓ /move_group action 可用")
    else:
        print("  ❌ /move_group action 不可用")
        print("\n  ❌ MoveIt2 可能未启动！")
        print("\n  解决方案：")
        print("  1. 在终端 1 运行:")
        print("     cd ~/franka_ws && ./start_interactive_sim.sh")
        print("")
        print("  2. 等待 30-60 秒让系统完全启动")
        print("")
        print("  3. 在终端 2 运行脚本:")
        print("     cd ~/franka_ws && source install/setup.bash")
        print("     python3 src/multipanda_ros2/scripts/dual_arm_demo.py")

def main():
    print("\n")
    print("█" * 60)
    print("  ROS2 + MoveIt2 系统诊断工具")
    print("█" * 60)
    
    # 检查 ROS2 是否初始化
    print("\n检查 ROS2 环境...")
    ros_domain = run_cmd("echo $ROS_DOMAIN_ID")
    print(f"  ROS_DOMAIN_ID: {ros_domain if ros_domain else '未设置 (使用默认 0)'}")
    
    check_nodes()
    check_topics()
    check_actions()
    check_move_group()
    
    print("\n" + "="*60)
    print("诊断完成")
    print("="*60 + "\n")

if __name__ == '__main__':
    main()
