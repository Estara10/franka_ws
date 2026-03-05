# multipanda_ros2 脚本说明（当前版本）

这个文档只描述现在实际在用的入口和调试方法，避免和旧脚本名混淆。

## 现在的主入口

- `src/multipanda_ros2/scripts/dual_arm_transport_demo.py`
  - 双臂协作搬运状态机主程序（ROS2 + MoveIt2 + MuJoCo）
  - 支持 `--mode demo`（演示容错）和 `--mode eval`（严格评估）

> 备注：之前的 `test2.py` 逻辑已迁移到 `dual_arm_transport_demo.py`。

## 相关核心脚本

- `src/multipanda_ros2/scripts/state_ops.py`：状态机各阶段逻辑（抓取、对齐、搬运、放置）
- `src/multipanda_ros2/scripts/servo_ops.py`：Servo/笛卡尔执行相关
- `src/multipanda_ros2/scripts/gripper_ops.py`：夹爪动作与校验
- `src/multipanda_ros2/scripts/task_types.py`：状态枚举定义

## 推荐启动顺序

1. 在工作区根目录启动仿真与 MoveIt2：
   ```bash
   ./start_interactive_sim.sh
   ```
2. 新终端运行任务脚本：
   ```bash
   source install/setup.bash
   python3 src/multipanda_ros2/scripts/dual_arm_transport_demo.py --mode demo
   ```
3. 需要严格验收时：
   ```bash
   python3 src/multipanda_ros2/scripts/dual_arm_transport_demo.py --mode eval
   ```

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

## 参数该改哪里

主要在 `dual_arm_transport_demo.py` 的 `DualArmTaskNode.__init__()` 里调：

- 预抓取与对齐：`pregrasp_align_*`、`pregrasp_force_align_*`
- 构型自然度：`natural_ik_*`
- 轨迹平滑：`moveit_joint_*`、`sync_plan_*`、`dual_traj_*`
- 夹取与下探：`grasp_*`

## 注意事项

- `long_bar/target_bar` 可以由仿真 launch 中的兜底静态 TF 提供，不必强依赖外部感知。
- 只改 Python 脚本通常不需要 `colcon build`，重启脚本即可生效。
- `dual_arm_transport_demo backup.py`、`dual_arm_transport_demo copy.py` 属于备份文件，不建议作为运行入口。
