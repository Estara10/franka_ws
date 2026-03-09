# multipanda_ros2/scripts 说明

本目录包含基于 ROS2 + MoveIt2 + MuJoCo 的双臂协作控制脚本，适用于 Franka Emika Panda 的仿真开发与中期演示。

## 主脚本与当前入口

- 主入口：`src/multipanda_ros2/scripts/dual_arm_transport_demo.py`
  - 功能：双臂协作抓取与搬运主控脚本（状态机）
  - 模式：
    - `--mode demo`：演示容错模式（优先跑通流程）
    - `--mode eval`：严格评估模式（结果判定更严）

> 兼容说明：历史文档中的 `test2.py` 已演进为 `dual_arm_transport_demo.py`。

## 核心能力（保留旧版能力说明）

- 双臂协同抓取/搬运/放置完整流程
- MoveIt2 规划（关节空间 + 笛卡尔插补）
- 轨迹同步执行与误差监控
- 夹爪控制与抓取重试机制
- 力控/柔顺参数可调（仿真中用于稳态控制）
- 预抓取姿态、抓点偏置、速度与阈值均可在脚本内配置

## 相关核心脚本

- `state_ops.py`：状态机阶段逻辑（抓取、对齐、搬运、放置）
- `servo_ops.py`：Servo 与笛卡尔执行相关逻辑
- `gripper_ops.py`：夹爪动作与开闭校验
- `task_types.py`：状态枚举定义

## 典型开发流程

1. 启动仿真环境（MuJoCo + MoveIt2 + RViz）：
   ```bash
   ./start_interactive_sim.sh
   ```
2. 新终端运行主控脚本：
   ```bash
   source install/setup.bash
   python3 src/multipanda_ros2/scripts/dual_arm_transport_demo.py --mode demo
   ```
3. 需要严格评估时：
   ```bash
   python3 src/multipanda_ros2/scripts/dual_arm_transport_demo.py --mode eval
   ```

## 常用参数调整指引

在 `dual_arm_transport_demo.py` 的 `DualArmTaskNode.__init__()` 中直接调整：

- 预抓取与对齐：`pregrasp_align_*`、`pregrasp_force_align_*`
- 构型自然化：`natural_ik_*`
- 轨迹平滑：`moveit_joint_*`、`sync_plan_*`、`dual_traj_*`
- 抓取与下探：`grasp_*`
- 夹爪策略：`gripper_*`、`use_direct_grasp_close`

## 常用诊断脚本

- MoveIt 基础诊断：
  ```bash
  python3 src/multipanda_ros2/scripts/moveit_diagnosis.py
  ```
- 同步误差日志：
  ```bash
  python3 src/multipanda_ros2/scripts/sync_error_logger.py
  ```
- 目标/TF 诊断：
  ```bash
  python3 src/multipanda_ros2/scripts/target_diagnostics.py
  ```

## 依赖说明

- ROS2 Humble 及以上
- MoveIt2
- MuJoCo 3.2.0 及 ROS2 插件
- Franka Panda 模型与对应控制配置

详见：
- `src/multipanda_ros2/README.md`
- `src/multipanda_ros2/README_SOLUTION.md`
- `src/multipanda_ros2/IMPLEMENTATION_SUMMARY.md`

## 注意事项

- `long_bar/target_bar` 可由仿真 launch 的兜底静态 TF 提供，不必强依赖外部感知。
- 仅修改 Python 脚本时通常不需要 `colcon build`，重启脚本即可生效。
- `dual_arm_transport_demo backup.py`、`dual_arm_transport_demo copy.py` 为备份文件，不建议作为运行入口。
