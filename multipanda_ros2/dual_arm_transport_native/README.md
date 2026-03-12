# dual_arm_transport_native

这个包是 `scripts/dual_arm_transport_demo.py` 的原生迁移通道，目标是：

1. 保留现有 Python 代码可运行。
2. 将性能敏感路径迁移到 C++。
3. 最终把状态机和控制执行全部转到原生实现。

## 为什么采用 C++ 原生实现

在 ROS2 + MoveIt2 场景里，动作客户端、服务客户端、消息生态的主流生产实现是 C++ (`rclcpp`)。

- 纯 C (`rcl`) 能做底层通信，但工程复杂度会显著升高。
- MoveIt2 常用调用链在 C++ 里最完整。

因此本包采用统一 C++ 实现：

- ROS2 集成层：C++
- 控制核心（串级 PID）：C++ (`cascade_pid.cpp`)

这在工程上兼顾了效率、可维护性和交付风险。

## 已实现能力

- 订阅 `JointState`
- 订阅合并后的 14 关节轨迹：`/dual_arm_transport_native/merged_trajectory`
- 订阅笛卡尔增量命令：`/dual_arm_transport_native/cartesian_delta_cmd`
- 订阅笛卡尔绝对目标命令：`/dual_arm_transport_native/cartesian_targets_cmd`
- 订阅 State6 高层任务命令：`/dual_arm_transport_native/state6_task_cmd`
- 串级 PID（外环位置 + 内环速度）按关节独立执行
- 自动切换控制器：
  - `dual_panda_arm_controller` <-> `dual_joint_group_velocity_controller`
- 串级失败时自动回退 `FollowJointTrajectory`
- 支持原生执行双臂笛卡尔 delta（内部调用 `/compute_cartesian_path` 并合并执行）
- 支持原生执行 State6 平移任务（补偿抬升 + 主搬运平移）
- 支持原生执行 State6 中心旋转任务（含失败后二分步重试）

## Python 到 Native 对应关系（当前阶段）

- Python `_execute_merged_trajectory` -> Native `execute_with_cascade_pid` + `execute_with_action_fallback`
- Python `switch_to_servo_mode/switch_to_trajectory_mode` -> Native `switch_to_velocity_controller/switch_to_trajectory_controller`
- Python 关节状态缓存 -> Native `joint_state_callback`
- Python `_move_dual_cartesian_delta`（State5/State6） -> Native `cartesian_delta_callback`（第二阶段）
- Python `_move_dual_cartesian_to_targets`（旋转/对齐） -> Native `cartesian_targets_callback`（第三阶段）
- Python `state_execute_with_compliance`（平移/旋转） -> Native `state6_task_callback`（第四阶段）

## 编译

在工作区根目录执行：

```bash
colcon build --packages-select dual_arm_transport_native
```

## 运行

```bash
source install/setup.bash
ros2 run dual_arm_transport_native dual_arm_transport_native_node \
  --ros-args --params-file src/multipanda_ros2/dual_arm_transport_native/config/dual_arm_transport_native.yaml
```

## C++ 一键全流程运行

新增可执行节点：`dual_arm_transport_full_runner`

功能：

- 自动启动仿真 (`dual_franka_sim.launch.py`)
- 自动启动 MoveIt (`sim_dual_moveit.launch.py`)
- 自动启动原生执行器 (`dual_arm_transport_native_node`)
- 自动下发搬运任务（平移或旋转）
- 等待执行结果并自动收尾退出

默认参数文件：

- `src/multipanda_ros2/dual_arm_transport_native/config/dual_arm_transport_full_runner.yaml`

一键脚本（工作区根目录）：

```bash
./run_cpp_full_transport.sh
```

切换旋转模式示例：

```bash
./run_cpp_full_transport.sh --ros-args -p task_mode:=rotate
```

## 与现有 Python 并行运行方式

当前建议先保持 `dual_arm_transport_demo.py` 作为任务状态机，
将其生成的合并轨迹转发到：

- 话题：`/dual_arm_transport_native/merged_trajectory`
- 类型：`trajectory_msgs/msg/JointTrajectory`

原生节点会接管执行。

第二阶段可再进一步：把笛卡尔增量命令直接发给原生节点，由原生完成规划+执行。
命令类型：

- 话题：`/dual_arm_transport_native/cartesian_delta_cmd`
- 类型：`std_msgs/msg/Float64MultiArray`
- 数据格式：`[dx, dy, dz, max_step, target_speed, time_scale]`
  - 后三项可省略，省略时使用参数默认值。

绝对目标命令：

- 话题：`/dual_arm_transport_native/cartesian_targets_cmd`
- 类型：`std_msgs/msg/Float64MultiArray`
- 数据格式：
  `[lx,ly,lz,lqx,lqy,lqz,lqw, rx,ry,rz,rqx,rqy,rqz,rqw, max_step, target_speed, time_scale, avoid_collisions]`

State6 任务命令：

- 话题：`/dual_arm_transport_native/state6_task_cmd`
- 类型：`std_msgs/msg/Float64MultiArray`
- mode=0（平移任务）数据格式：
  `[0, lift_dz, dx, dy, dz, max_step, lift_speed, transport_speed, time_scale, transport_one_shot_segment, transport_retry_segment]`
- mode=1（中心旋转任务）数据格式：
  `[1, center_x, center_y, total_deg, step_limit_deg, rotate_orientation, max_step, target_speed, time_scale, try_without_orientation, avoid_collisions, fraction_threshold, settle_sec]`

## 下一步迁移建议

1. 先替换执行层：Python 仍规划，Native 执行。
2. 再迁移状态 5/6 笛卡尔段逻辑到 C++（已覆盖到 State6 高层任务）。
3. 最后迁移完整状态机与 IK/规划调用。
