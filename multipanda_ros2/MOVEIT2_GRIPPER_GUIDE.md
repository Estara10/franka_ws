# MoveIt2夹取系统 - 使用指南

## 📋 概述

这个脚本 `gripper_pick_moveit.py` 是**基于MoveIt2规划框架**实现的双臂机械夹取系统。

### 与之前脚本的区别

| 特性 | gripper_pick_v1.py | gripper_pick_moveit.py |
|------|-------------------|----------------------|
| **规划方式** | 直接轨迹 | ✅ MoveIt2规划 |
| **碰撞检测** | 无 | ✅ 支持 |
| **约束** | 简单 | ✅ 完整支持 |
| **灵活性** | 固定路径 | ✅ 灵活规划 |
| **学习难度** | ⭐⭐ | ⭐⭐⭐ |

---

## 🚀 快速开始

### 前置条件

确保仿真系统已启动：

```bash
# Terminal 1
cd ~/franka_ws
./start_interactive_sim.sh
```

等待输出：
```
✓ 系统启动完成！
```

### 运行MoveIt2夹取脚本

```bash
# Terminal 2（等Terminal 1完成）
cd ~/franka_ws
source install/setup.bash
python3 src/multipanda_ros2/scripts/gripper_pick_moveit.py
```

### 预期输出

```
============================================================
🚀 MoveIt2机械臂夹取系统
============================================================

[INIT] 初始化MoveIt2夹取系统...
[INIT] 连接MoveIt2...
[SUCCESS] ✓ MoveIt2连接成功
[SUCCESS] ✓ 系统初始化完成

============================================================
🤖 开始MoveIt2夹取演示
============================================================

[步骤1] 打开夹爪...
[左] 夹爪指令: 打开

[步骤2] 规划靠近物体...
[mj_left_arm] 左臂靠近物体...
  目标位置: x=0.500, y=0.150, z=0.550
  📤 发送规划请求...
  ✓ 规划成功，等待执行...
  ✓ 执行成功!

...（继续）

✅ MoveIt2夹取演示完成!
```

---

## 🎯 核心功能

### 1. MoveIt2规划

脚本使用MoveIt2的 `MoveGroup` Action来规划运动：

```python
goal = MoveGroup.Goal()
goal.request.group_name = "mj_left_arm"  # 选择运动组
goal.request.allowed_planning_time = 10.0  # 规划时间
goal.request.num_planning_attempts = 10  # 规划尝试次数
```

### 2. 约束定义

支持位置约束（球体约束）：

```python
pc = PositionConstraint()
pc.header.frame_id = "world"
pc.link_name = "mj_left_link8"  # 末端执行器
pc.constraint_region.primitives.append(
    SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.1])
)
```

### 3. 完整夹取流程

1. **打开夹爪** - 准备接近物体
2. **规划靠近** - 两臂从两侧靠近
3. **规划下降** - 到达物体高度
4. **关闭夹爪** - 夹紧物体
5. **规划抬起** - 共同抬起物体
6. **打开夹爪** - 放下物体

---

## 📝 代码详解

### 关键参数

```python
# 物体位置（MuJoCo场景中）
object_x, object_y, object_z = 0.5, 0.0, 0.45

# 两臂的相对位置
left_approach_y = object_y + 0.15   # 左臂从左侧靠近
right_approach_y = object_y - 0.15  # 右臂从右侧靠近

# 高度参数
approach_height = 0.55  # 靠近高度
grasp_height = 0.48     # 抓取高度
lift_height = 0.65      # 抬起高度
```

### 修改为你的应用

修改以下部分来适配你的需求：

```python
# 1. 修改物体位置
object_x, object_y, object_z = 0.5, 0.0, 0.45  # 改这里

# 2. 修改靠近距离
object_y + 0.15  # 改这个距离

# 3. 修改规划时间
planning_time=10.0  # 改这个值，更长=更仔细

# 4. 修改夹爪位置
goal.command.position = 0.04  # 打开位置
goal.command.position = 0.0   # 关闭位置
```

---

## 🔧 故障排查

### 问题1：MoveIt2规划失败

```
❌ 规划被拒绝
```

**原因**：
- MoveIt2规划器没找到有效路径
- 目标位置超出工作空间
- 起始状态存在碰撞

**解决**：
- 增加 `planning_time` 参数（例如20秒）
- 检查目标位置是否合理
- 使用RViz查看当前状态

### 问题2：执行超时

```
❌ 执行超时
```

**原因**：
- 规划成功但执行时间过长
- 轨迹可能太复杂

**解决**：
- 增加超时时间
- 使用速度缩放参数

### 问题3：夹爪不工作

```
[左] 夹爪不可用
```

**原因**：
- 夹爪Action服务未就绪
- 仿真未启动

**解决**：
- 确保仿真系统完全启动
- 检查夹爪Action名称

---

## 🎓 进阶用法

### 添加自定义运动

在 `pick_with_moveit()` 中添加新的规划步骤：

```python
# 自定义位置
custom_pose = Pose()
custom_pose.position.x = 0.6
custom_pose.position.y = 0.2
custom_pose.position.z = 0.5
custom_pose.orientation.x = qx
custom_pose.orientation.y = qy
custom_pose.orientation.z = qz
custom_pose.orientation.w = qw

# 规划并执行
self.plan_and_move("mj_left_arm", custom_pose, 
                   description="自定义运动")
```

### 修改约束

修改约束球体大小：

```python
# 增大约束球体（更宽松）
dimensions=[0.2]  # 从0.1改为0.2

# 减小约束球体（更严格）
dimensions=[0.05]  # 从0.1改为0.05
```

### 调整规划参数

```python
# 增加规划尝试次数
goal.request.num_planning_attempts = 20  # 从10增加到20

# 改变速度缩放
goal.request.max_velocity_scaling_factor = 0.5  # 降低速度

# 改变加速度缩放
goal.request.max_acceleration_scaling_factor = 0.5
```

---

## 📊 系统架构

```
MoveIt2夹取系统
├─ 初始化
│  ├─ 连接MoveIt2 Action Server
│  ├─ 连接夹爪控制器
│  └─ 订阅关节状态
│
├─ 规划循环
│  ├─ 创建MoveGroup目标
│  ├─ 添加约束
│  ├─ 发送规划请求
│  ├─ 等待规划完成
│  └─ 执行轨迹
│
└─ 完整夹取流程
   ├─ 打开夹爪
   ├─ 规划靠近
   ├─ 规划下降
   ├─ 关闭夹爪
   ├─ 规划抬起
   └─ 打开夹爪放下
```

---

## 💡 提示

1. **规划时间很重要** - 给更多时间让MoveIt2找到更好的路径
2. **使用RViz调试** - 在RViz中可以看到规划结果
3. **逐步测试** - 先测试单个步骤，再整合完整流程
4. **保存成功的轨迹** - 记录成功的参数，用于生产环境

---

## 📝 完整参数参考

| 参数 | 类型 | 默认值 | 说明 |
|------|------|-------|------|
| group_name | str | - | 运动组名称 |
| target_pose | Pose | - | 目标姿态 |
| planning_time | float | 10.0 | 规划时间（秒） |
| num_planning_attempts | int | 10 | 规划尝试次数 |
| max_velocity_scaling | float | 1.0 | 最大速度缩放 |
| max_acceleration_scaling | float | 1.0 | 最大加速度缩放 |

---

## 🚀 下一步

1. **运行演示** - 测试默认参数
2. **查看日志** - 理解规划过程
3. **修改参数** - 针对你的应用调整
4. **集成到项目** - 使用你自己的逻辑

祝你成功！🎉
