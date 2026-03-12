# 双臂协作搬运任务

## 功能概述

该功能包实现了 Franka 双臂机器人协作搬运铝合金杆的演示任务。

**任务流程:**
1. **INIT** - 双臂移动到初始就绪位置
2. **APPROACH** - 双臂接近物体两侧
3. **GRASP** - 双臂夹紧物体
4. **LIFT** - 协同抬起物体

## 快速启动

### 方式1: 独立启动任务节点

```bash
# 1. 启动 MuJoCo 仿真和 MoveIt2
cd ~/mujoco_franka/franka_ws
./start_interactive_sim.sh

# 2. 在新终端中启动任务节点
source ~/mujoco_franka/franka_ws/install/setup.bash
ros2 launch dual_arm_carry_task dual_arm_carry_task.launch.py
```

### 方式2: 集成到自动启动脚本

修改 `start_interactive_sim.sh`，在最后添加:

```bash
# 启动双臂搬运任务
sleep 5
gnome-terminal -- bash -c "source ~/mujoco_franka/franka_ws/install/setup.bash && ros2 launch dual_arm_carry_task dual_arm_carry_task.launch.py; exec bash"
```

## 参数配置

可在启动时配置以下参数:

```bash
ros2 launch dual_arm_carry_task dual_arm_carry_task.launch.py \
  left_arm_group:=mj_left_arm \
  right_arm_group:=mj_right_arm \
  approach_height:=0.15 \
  grasp_height:=0.06 \
  lift_height:=0.25
```

### 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `left_arm_group` | `mj_left_arm` | 左臂 MoveGroup 名称 |
| `right_arm_group` | `mj_right_arm` | 右臂 MoveGroup 名称 |
| `dual_arm_group` | `dual_panda` | 双臂协同组名称 |
| `approach_height` | `0.15` | 接近物体时末端高度 (m) |
| `grasp_height` | `0.06` | 抓取物体时末端高度 (m) |
| `lift_height` | `0.25` | 抬起物体后的高度 (m) |

## 场景配置

物体定义在 `franka_description/mujoco/franka/objects.xml`:

```xml
<body name="obj_rod_01" pos="0.5 0.0 0.04">
  <geom type="box" size="0.2 0.02 0.02" mass="0.5" 
        rgba="0.8 0.8 0.8 1" friction="1 0.005 0.0001"/>
  <freejoint/>
</body>
```

- **位置**: (0.5, 0.0, 0.04) m
- **尺寸**: 40cm × 4cm × 4cm
- **质量**: 0.5 kg

## 日志输出

任务执行过程中会输出详细日志:

```
[状态: INIT] 移动到初始位置...
  ✓ 左臂规划成功
  ✓ 左臂移动完成
  ✓ 右臂规划成功
  ✓ 右臂移动完成
✓ 初始化完成，进入接近阶段

[状态: APPROACH] 接近物体...
  左臂目标: [0.50, 0.28, 0.15]
  右臂目标: [0.50, -0.28, 0.15]
  ✓ 左臂规划成功，开始执行...
  ✓ 左臂移动完成
  ✓ 右臂规划成功，开始执行...
  ✓ 右臂移动完成
✓ 接近完成，进入抓取阶段

[状态: GRASP] 夹紧物体...
  ✓ 左臂到达抓取位置
  ✓ 右臂到达抓取位置
  [模拟] 闭合夹爪...
✓ 抓取完成，进入抬起阶段

[状态: LIFT] 抬起物体...
  目标高度: 0.25 m
  ✓ 左臂抬升完成
  ✓ 右臂抬升完成
✓ 抬起完成

========================================
    ✓ 任务完成！
========================================
```

## 故障排查

### 规划失败

**症状**: 日志显示 "✗ 规划失败"

**可能原因**:
- 目标位置超出工作空间
- 存在碰撞风险
- IK 无解

**解决方法**:
1. 检查物体位置是否在双臂可达范围内
2. 调整 `approach_height` 和 `grasp_height` 参数
3. 在 RViz 中检查机械臂初始位置

### MoveGroup 未找到

**症状**: "No planning group named 'mj_left_arm'"

**解决方法**:
```bash
# 检查可用的 MoveGroup
ros2 topic echo /move_group/planning_scene

# 确认 SRDF 配置
cat ~/mujoco_franka/franka_ws/src/multipanda_ros2/multi_panda_moveit_config/config/multi_panda.srdf
```

### MuJoCo 仿真未同步

**症状**: RViz 中机械臂移动，但 MuJoCo 窗口无响应

**解决方法**:
```bash
# 检查 mujoco_node 是否运行
ros2 node list | grep mujoco

# 检查控制器状态
ros2 control list_controllers
```

## 代码架构

```
dual_arm_carry_task/
├── src/
│   └── dual_arm_carry_task.cpp   # 主节点实现
├── include/
│   └── dual_arm_carry_task/      # (预留头文件目录)
├── launch/
│   └── dual_arm_carry_task.launch.py  # 启动文件
├── package.xml                    # ROS2 功能包描述
├── CMakeLists.txt                # 构建配置
└── README.md                     # 本文件
```

### 类设计

**DualArmCarryTask**
- 继承自 `rclcpp::Node`
- 成员变量:
  - `left_arm_`: 左臂 MoveGroupInterface
  - `right_arm_`: 右臂 MoveGroupInterface
  - `current_state_`: 任务状态机
- 主要方法:
  - `initialize()`: 初始化 MoveIt 接口
  - `executeTask()`: 执行任务主循环
  - `executeInit()`: INIT 状态处理
  - `executeApproach()`: APPROACH 状态处理
  - `executeGrasp()`: GRASP 状态处理
  - `executeLift()`: LIFT 状态处理

## 扩展开发

### 添加新状态

在 `TaskState` 枚举中添加新状态:

```cpp
enum class TaskState {
    INIT,
    APPROACH,
    GRASP,
    LIFT,
    CARRY,      // 新增: 搬运状态
    PLACE,      // 新增: 放置状态
    DONE,
    ERROR
};
```

实现对应的处理函数:

```cpp
void executeCarry() {
    // 实现搬运逻辑
}

void executePlace() {
    // 实现放置逻辑
}
```

在 `executeTask()` 中添加状态分支:

```cpp
case TaskState::CARRY:
    executeCarry();
    break;
case TaskState::PLACE:
    executePlace();
    break;
```

### 集成夹爪控制

当前夹爪控制是模拟的，需要实现实际的夹爪接口:

```cpp
#include <franka_msgs/action/grasp.hpp>

// 在类中添加 action client
rclcpp_action::Client<franka_msgs::action::Grasp>::SharedPtr left_gripper_client_;
rclcpp_action::Client<franka_msgs::action::Grasp>::SharedPtr right_gripper_client_;

// 在 executeGrasp() 中调用
auto goal = franka_msgs::action::Grasp::Goal();
goal.width = 0.04;  // 夹爪宽度
goal.speed = 0.1;
goal.force = 10.0;
left_gripper_client_->async_send_goal(goal);
```

## 许可证

Apache-2.0

## 联系方式

如有问题请提交 Issue 或联系维护者。
