# MoveIt2 规划失败诊断报告

## 问题摘要
MoveIt2 规划失败，返回错误码 99999（通用失败），无法为双臂 Panda 机械臂生成运动轨迹。

## 诊断结果

### ✓ 系统状态
- **MuJoCo 仿真服务器**: 正在运行 ✓
- **ROS2 MoveIt2**: 正在运行 ✓  
- **Action Server (`/move_action`)**: 可用且响应正常 ✓
- **规划服务 (`/plan_kinematic_path`)**: 可用 ✓

### ❌ 规划失败的表现
```
[输入] 关节约束: mj_left_joint1 = 0.1 rad (tolerance: ±0.1)
[输入] 组名: panda_1
[输入] 规划时间: 5.0 秒

[输出] 错误码: 99999 (通用失败)
[输出] 轨迹点数: 0
[输出] 规划耗时: 0.00 秒
```

即使使用完整的 7 个关节约束，仍然失败。

## 可能原因

### 1. **起始状态问题**
当前机械臂关节状态：
```
mj_left_joint1: 0.0567 rad (≈ 3.25°)
mj_left_joint2: -0.5898 rad (≈ -33.8°)
mj_left_joint3: 0.1294 rad (≈ 7.4°)
... (其他关节)
```

这个起始状态可能：
- 不是标准的初始化位置
- 可能存在自碰撞或与环境碰撞
- 导致规划器无法找到从该状态出发的有效路径

### 2. **约束定义不完整**
仅指定单个或部分关节约束时，MoveIt2 需要：
- 使用逆运动学求解器来确定其他关节的值
- 如果 IK 失败或存在多个解，规划器可能无法确定目标

### 3. **MoveIt2 配置问题**
- OMPL 规划器可能没有正确加载所有插件
- 运动规划管道可能缺少必要的步骤（如碰撞检查、轨迹平滑)
- 关节限制或约束定义可能有误

## 临时解决方案

### 方案 A: 重置机械臂到标准初始位置
在发送任何规划请求之前，先将机械臂移动到标准初始位置：

```bash
ros2 service call /set_model_configuration \
  moveit_msgs/SetModelConfiguration \
  "state: {joint_state: {name: [mj_left_joint1, mj_left_joint2, ...], \
  position: [0, -0.785, 0, -2.356, 0, 1.57, 0.785]}}"
```

### 方案 B: 使用 RViz + MoveIt2 GUI 进行手动规划和执行
```bash
ros2 launch franka_moveit_config moveit_rviz.launch.py
```
然后在 RViz 中使用"规划"按钮来测试 MoveIt2 是否能在 GUI 中正常工作。

### 方案 C: 直接发送关节轨迹到执行器
不依赖 MoveIt2 规划，直接向 `joint_trajectory_controller` 发送轨迹：

```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {header: {frame_id: '', stamp: {sec: 0, nsec: 0}}, ...}}"
```

### 方案 D: 检查 MoveIt2 配置文件
检查以下配置文件：
- `franka_moveit_config/config/kinematics.yaml` - 逆运动学配置
- `franka_moveit_config/config/ompl_planning.yaml` - 规划器配置
- `franka_moveit_config/config/joint_limits.yaml` - 关节限制
- `franka_moveit_config/config/motion_planning.yaml` - 规划管道配置

## 建议的排查步骤

1. **在 RViz 中测试规划**
   ```bash
   ros2 launch franka_moveit_config demo.launch.py
   ```
   看看 GUI 规划是否工作

2. **检查碰撞状态**
   ```bash
   ros2 service call /check_state_validity \
     moveit_msgs/GetStateValidity \
     "group_name: panda_1"
   ```

3. **查看 MoveIt2 日志**
   ```bash
   ros2 launch franka_moveit_config moveit_rviz.launch.py log_level:=debug
   ```

4. **尝试更简单的约束**
   - 不使用位置约束，仅使用关节约束
   - 减少关节约束的紧密度（增加 tolerance）

## 注意事项

- 当前仿真中的机械臂可能处于某种特殊的配置，导致规划困难
- MoveIt2 的规划成功率取决于许多因素：初始状态、目标配置、环境障碍物等
- 如果规划失败，可能需要通过 RViz GUI 进行手动干预和重新初始化

## 下一步

1. 首先，尝试在 RViz 中进行手动规划测试
2. 如果 RViz 规划也失败，说明是 MoveIt2 配置问题
3. 如果只有脚本规划失败而 RViz 成功，可能是消息格式或参数问题
4. 考虑直接控制关节轨迹作为备选方案
