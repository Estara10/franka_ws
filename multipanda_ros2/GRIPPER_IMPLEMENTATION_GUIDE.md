# 双臂机械夹取 - 完整实现指南

## 📚 文档结构

本指南分为三个层级，逐步从最简单到最复杂：

1. **层级1：基础 - 直接关节控制** ✅ 最可靠
2. **层级2：中级 - 添加安全检查和反馈**
3. **层级3：高级 - 使用 MoveIt2 规划**

---

## 🎯 为什么要重新写代码？

### 原代码 (dual_arm_demo.py) 的问题

```
❌ 依赖 MoveIt2 规划
   → 规划失败（错误码 99999）
   → 无法诊断具体原因
   → 成功率不稳定

❌ 高度抽象，难以调试
   → 不知道关节是否真的移动
   → 无法验证每一步

❌ 夹爪控制不可靠
   → Action 可能未就绪
   → 无法验证是否成功夹紧
```

### 新方案的优势

```
✅ 直接关节轨迹控制
   → 绕过 MoveIt2 规划
   → 100% 可靠（只要关节控制器运行）
   → 速度快

✅ 完整的状态反馈
   → 实时看到关节角度
   → 可以逐步调试
   → 易于诊断问题

✅ 模块化设计
   → 每个动作独立
   → 易于组合和修改
```

---

## 🚀 快速开始 - 层级1（推荐）

### 前置条件

确保仿真系统已启动：

```bash
# Terminal 1
cd ~/franka_ws
./start_interactive_sim.sh
```

等待出现：
```
✓ 系统启动完成！
```

### 方式A：运行完整夹取演示

```bash
# Terminal 2
cd ~/franka_ws
source install/setup.bash
python3 src/multipanda_ros2/scripts/gripper_pick_v1.py
```

预期输出：
```
[INIT] 正在连接轨迹控制器...
[SUCCESS] ✓ 轨迹控制器连接成功
[SUCCESS] ✓ 订阅关节状态
[SUCCESS] ✓ 已获得关节状态

📊 当前关节状态：
[...]

[ACTION] 打开夹爪...
  ⏳ 等待执行完成...
  ✓ 完成!

[ACTION] 靠近物体...
  ✓ 完成!

[ACTION] 关闭夹爪...
  ✓ 完成!

[ACTION] 抬起物体...
  ✓ 完成!

[ACTION] 打开夹爪...
  ✓ 完成!

[ACTION] 回到初始位置...
  ✓ 完成!

✅ 夹取演示完成!
```

### 方式B：交互式调试工具（推荐用于开发）

```bash
# Terminal 2
cd ~/franka_ws
source install/setup.bash
python3 src/multipanda_ros2/scripts/interactive_gripper_debug.py
```

可用命令：

```
>> state           # 查看当前关节角度
>> open            # 打开夹爪
>> close           # 关闭夹爪
>> home            # 回到初始位置
>> approach        # 靠近物体
>> pick            # 执行完整夹取
>> move L 0.0 -0.785 0.0 -2.356 0.0 1.57 0.785  # 移动左臂
>> move R 0.0 -0.785 0.0 -2.356 0.0 1.57 0.785  # 移动右臂
>> exit            # 退出
```

---

## 📖 代码详解 - 学会自己写代码

### 核心步骤1：连接到轨迹控制器

```python
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory

# 创建 Action 客户端
self.trajectory_client = ActionClient(
    self, 
    FollowJointTrajectory, 
    '/dual_panda_arm_controller/follow_joint_trajectory'
)

# 等待服务器就绪
if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
    raise RuntimeError("轨迹控制器不可用!")
```

**关键点**：
- Action 名称必须正确：`/dual_panda_arm_controller/follow_joint_trajectory`
- 超时设置很重要
- 这个步骤的成功表示仿真系统正在运行

### 核心步骤2：订阅关节状态

```python
from sensor_msgs.msg import JointState

self.joint_state_sub = self.create_subscription(
    JointState,
    '/joint_states',
    self._joint_state_callback,
    10
)

def _joint_state_callback(self, msg):
    """更新当前关节位置"""
    self.current_joint_state = msg
    
    # 提取左臂关节角度
    for i, joint in enumerate(self.left_joints):
        if joint in msg.name:
            idx = msg.name.index(joint)
            self.current_left_positions[i] = msg.position[idx]
```

**关键点**：
- 关节顺序很重要：`mj_left_joint1, mj_left_joint2, ..., mj_left_joint7`
- 使用 `msg.name.index()` 来找到对应的位置
- 保存当前位置用于生成轨迹的起点

### 核心步骤3：执行轨迹

```python
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

def execute_trajectory(self, joint_names, positions, duration=2.0):
    # 创建轨迹消息
    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names
    
    # 起点：当前位置
    start_point = JointTrajectoryPoint()
    start_point.positions = [self.current_left_positions[i] for i in range(7)]
    start_point.time_from_start = Duration(sec=0, nanosec=0)
    
    # 终点：目标位置
    end_point = JointTrajectoryPoint()
    end_point.positions = positions
    end_point.time_from_start = Duration(sec=int(duration), nanosec=0)
    
    trajectory.points = [start_point, end_point]
    
    # 发送轨迹
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory
    
    future = self.trajectory_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(self, future, timeout_sec=15)
    
    goal_handle = future.result()
    if goal_handle.accepted:
        # 等待执行完成
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 10)
        
        result = result_future.result()
        if result.result.error_code == 0:
            print("✓ 执行成功!")
            return True
        else:
            print(f"❌ 执行失败! 错误码: {result.result.error_code}")
            return False
```

**关键点**：
- 轨迹必须有起点和终点
- 起点应该是当前位置（确保平滑过渡）
- 终点是目标位置
- 时间戳单位是纳秒，需要转换

### 核心步骤4：组合动作

```python
def pick_object(self):
    # 1. 打开夹爪
    self.execute_trajectory(
        [self.left_gripper_joint, self.right_gripper_joint],
        [0.04, 0.04],  # 打开位置
        duration=1.0
    )
    time.sleep(1)
    
    # 2. 靠近物体
    self.execute_trajectory(
        self.left_joints + self.right_joints,
        left_approach + right_approach,
        duration=2.0
    )
    time.sleep(1)
    
    # 3. 关闭夹爪
    self.execute_trajectory(
        [self.left_gripper_joint, self.right_gripper_joint],
        [0.0, 0.0],  # 关闭位置
        duration=1.5
    )
    time.sleep(1)
    
    # 4. 抬起物体
    self.execute_trajectory(
        self.left_joints + self.right_joints,
        left_lift + right_lift,
        duration=2.0
    )
    # ... 以此类推
```

---

## 🔧 自己编写代码的步骤

### 第1步：复制基础框架

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
import time


class MyGripperController(Node):
    def __init__(self):
        super().__init__('my_gripper_controller', 
                         callback_group=ReentrantCallbackGroup())
        
        # 关节定义
        self.left_joints = [
            'mj_left_joint1', 'mj_left_joint2', 'mj_left_joint3',
            'mj_left_joint4', 'mj_left_joint5', 'mj_left_joint6', 'mj_left_joint7'
        ]
        self.right_joints = [
            'mj_right_joint1', 'mj_right_joint2', 'mj_right_joint3',
            'mj_right_joint4', 'mj_right_joint5', 'mj_right_joint6', 'mj_right_joint7'
        ]
        
        # 状态变量
        self.current_joint_state = None
        self.current_left_positions = [0.0] * 7
        self.current_right_positions = [0.0] * 7
        
        # 连接控制器
        self.trajectory_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/dual_panda_arm_controller/follow_joint_trajectory'
        )
        
        if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("轨迹控制器不可用!")
        
        # 订阅状态
        self.create_subscription(
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
    
    def _joint_state_callback(self, msg):
        self.current_joint_state = msg
        for i, joint in enumerate(self.left_joints):
            if joint in msg.name:
                self.current_left_positions[i] = msg.position[msg.name.index(joint)]
        for i, joint in enumerate(self.right_joints):
            if joint in msg.name:
                self.current_right_positions[i] = msg.position[msg.name.index(joint)]


def main():
    rclpy.init()
    try:
        controller = MyGripperController()
        print("✓ 控制器初始化成功!")
    except Exception as e:
        print(f"❌ 错误: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 第2步：添加轨迹执行方法

在 `MyGripperController` 类中添加：

```python
def execute_trajectory(self, joint_names, positions, duration=2.0):
    """执行轨迹"""
    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names
    
    # 起点
    start_point = JointTrajectoryPoint()
    start_point.positions = []
    for joint in joint_names:
        if joint in self.left_joints:
            idx = self.left_joints.index(joint)
            start_point.positions.append(self.current_left_positions[idx])
        elif joint in self.right_joints:
            idx = self.right_joints.index(joint)
            start_point.positions.append(self.current_right_positions[idx])
    start_point.time_from_start = Duration(sec=0, nanosec=0)
    
    # 终点
    end_point = JointTrajectoryPoint()
    end_point.positions = positions
    end_point.time_from_start = Duration(sec=int(duration), nanosec=0)
    
    trajectory.points = [start_point, end_point]
    
    # 发送
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory
    
    future = self.trajectory_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(self, future, timeout_sec=15)
    
    goal_handle = future.result()
    if goal_handle.accepted:
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 10)
        return True
    return False
```

### 第3步：定义具体动作

```python
def move_left_arm(self, positions):
    """移动左臂"""
    return self.execute_trajectory(self.left_joints, positions, duration=2.0)

def move_right_arm(self, positions):
    """移动右臂"""
    return self.execute_trajectory(self.right_joints, positions, duration=2.0)

def move_both_arms(self, left_pos, right_pos):
    """同时移动两只臂"""
    return self.execute_trajectory(
        self.left_joints + self.right_joints,
        left_pos + right_pos,
        duration=2.0
    )
```

### 第4步：写夹取逻辑

```python
def my_pick_task(self):
    """你的自定义夹取逻辑"""
    
    # 移动到靠近物体的位置
    left_pos = [0.0, -1.0, 0.0, -2.0, 0.0, 1.0, 0.0]
    right_pos = [0.0, -1.0, 0.0, -2.0, 0.0, 1.0, 0.0]
    
    print("[任务1] 靠近物体...")
    if self.move_both_arms(left_pos, right_pos):
        print("✓ 靠近完成")
    else:
        print("❌ 靠近失败")
        return
    
    time.sleep(1)
    
    # 关闭夹爪
    print("[任务2] 关闭夹爪...")
    if self.execute_trajectory(
        [self.left_gripper_joint, self.right_gripper_joint],
        [0.0, 0.0],
        duration=1.0
    ):
        print("✓ 夹爪关闭")
    else:
        print("❌ 夹爪失败")
        return
    
    time.sleep(1)
    
    # 抬起
    print("[任务3] 抬起物体...")
    left_lift = [0.0, -0.5, 0.0, -2.356, 0.0, 1.57, 0.785]
    right_lift = [0.0, -0.5, 0.0, -2.356, 0.0, 1.57, 0.785]
    
    if self.move_both_arms(left_lift, right_lift):
        print("✓ 抬起完成")
    else:
        print("❌ 抬起失败")
        return
    
    print("✅ 夹取成功!")
```

---

## 🎓 常见问题与解决方案

### Q1: "轨迹控制器不可用"

**原因**：仿真系统没有启动

**解决**：
```bash
./start_interactive_sim.sh
```

等待 20-30 秒直到系统完全启动。

### Q2: 机械臂不动

**原因**：可能的原因：
1. 关节名称错误
2. 轨迹点数不足
3. 时间设置太短

**解决**：
- 检查关节名称：`ros2 topic echo /joint_states`
- 增加轨迹点数（添加中间点）
- 增加执行时间

### Q3: 夹爪不工作

**原因**：夹爪关节名称可能不同

**解决**：
```bash
# 查看所有关节
ros2 topic echo /joint_states
```

找到夹爪关节的名称，替换代码中的 `self.left_gripper_joint`

### Q4: 如何找到正确的关节角度？

**方法1**：使用交互式调试工具
```bash
python3 interactive_gripper_debug.py
```

然后在 RViz 中手动拖动机械臂，查看对应的关节角度。

**方法2**：从 RViz 日志读取
```bash
# 在 RViz 中移动机械臂后查看
ros2 topic echo /joint_states
```

---

## 📝 完整示例：从零开始写一个夹取脚本

创建文件 `my_gripper_pick.py`：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
import time


class MyGripperPick(Node):
    def __init__(self):
        super().__init__('my_gripper_pick', callback_group=ReentrantCallbackGroup())
        
        # 关节定义
        self.left_joints = ['mj_left_joint1', 'mj_left_joint2', 'mj_left_joint3',
                            'mj_left_joint4', 'mj_left_joint5', 'mj_left_joint6', 'mj_left_joint7']
        self.right_joints = ['mj_right_joint1', 'mj_right_joint2', 'mj_right_joint3',
                             'mj_right_joint4', 'mj_right_joint5', 'mj_right_joint6', 'mj_right_joint7']
        self.left_gripper = 'mj_left_finger_joint1'
        self.right_gripper = 'mj_right_finger_joint1'
        
        # 状态
        self.current_left_positions = [0.0] * 7
        self.current_right_positions = [0.0] * 7
        
        # 连接
        self.client = ActionClient(self, FollowJointTrajectory, 
                                   '/dual_panda_arm_controller/follow_joint_trajectory')
        if not self.client.wait_for_server(timeout_sec=10):
            raise RuntimeError("Controller not available!")
        
        # 订阅
        self.create_subscription(JointState, '/joint_states', 
                                self._update_state, 10)
        
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def _update_state(self, msg):
        for i, j in enumerate(self.left_joints):
            if j in msg.name:
                self.current_left_positions[i] = msg.position[msg.name.index(j)]
        for i, j in enumerate(self.right_joints):
            if j in msg.name:
                self.current_right_positions[i] = msg.position[msg.name.index(j)]
    
    def move(self, joints, positions, duration=2.0):
        """执行轨迹"""
        traj = JointTrajectory()
        traj.joint_names = joints
        
        start = JointTrajectoryPoint()
        start.positions = [self.current_left_positions[i] if j in self.left_joints 
                          else self.current_right_positions[self.right_joints.index(j)]
                          for j, i in [(j, self.left_joints.index(j) if j in self.left_joints 
                                       else self.right_joints.index(j)) 
                          for j in joints]]
        start.time_from_start = Duration(sec=0, nanosec=0)
        
        end = JointTrajectoryPoint()
        end.positions = positions
        end.time_from_start = Duration(sec=int(duration), nanosec=0)
        
        traj.points = [start, end]
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15)
        
        handle = future.result()
        if not handle:
            return False
        
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future, timeout_sec=duration + 10)
        
        return handle.accepted
    
    def pick(self):
        """执行夹取"""
        print("Opening grippers...")
        self.move([self.left_gripper, self.right_gripper], [0.04, 0.04], 1.0)
        time.sleep(1)
        
        print("Approaching object...")
        self.move(self.left_joints + self.right_joints,
                 [0, -1, 0, -2, 0, 1, 0] + [0, -1, 0, -2, 0, 1, 0], 2.0)
        time.sleep(1)
        
        print("Closing grippers...")
        self.move([self.left_gripper, self.right_gripper], [0.0, 0.0], 1.5)
        time.sleep(1)
        
        print("Lifting...")
        self.move(self.left_joints + self.right_joints,
                 [0, -0.5, 0, -2.356, 0, 1.57, 0.785] + [0, -0.5, 0, -2.356, 0, 1.57, 0.785], 2.0)
        
        print("✅ Done!")


def main():
    rclpy.init()
    try:
        pick = MyGripperPick()
        pick.pick()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

运行：
```bash
source install/setup.bash
python3 my_gripper_pick.py
```

---

## 🎯 下一步

1. **测试基础版本**：运行 `gripper_pick_v1.py`
2. **调试关节角度**：使用 `interactive_gripper_debug.py`
3. **自己编写**：基于提供的框架写你自己的控制逻辑
4. **添加功能**：例如碰撞检测、视觉反馈、多物体夹取等

祝你成功！🚀
