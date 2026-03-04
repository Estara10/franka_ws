# multipanda_ros2/scripts 说明

本目录包含基于 ROS2 + MoveIt2 + MuJoCo 的双臂协作机器人主要控制脚本，适用于 Franka Emika Panda 机械臂仿真与实机。

---

## 主要脚本简介

- **test2.py**
  - 功能：双臂协作抓取与搬运主控脚本，集成了运动规划、力控、抓取、仿真/实机切换等完整流程。
  - 典型用法：
    ```bash
    python3 src/multipanda_ros2/scripts/test2.py
    ```
  - 主要特性：
    - 支持双臂协作、主从/对等模式
    - 支持仿真与实机无缝切换
    - 运动规划基于 MoveIt2，支持笛卡尔/关节空间多种模式
    - 力控/阻抗参数可调，支持末端力传感器反馈
    - 预抓取高度、夹取补偿等参数可直接在脚本内调整（如 `pregrasp_left_z_offset`/`pregrasp_right_z_offset`）
    - 详细注释，便于二次开发

---

## 快速参数调整指引

- **预抓取高度补偿**：
  - `pregrasp_left_z_offset`：左臂预抓取高度补偿（单位：米，负值为降低）
  - `pregrasp_right_z_offset`：右臂预抓取高度补偿
  - 直接在 `test2.py` 约第 150 行附近修改即可

- **其它常用参数**：
  - 夹爪开合宽度、夹持力度、运动速度、力控阈值等，均可在 `__init__` 构造函数内直观调整

---

## 依赖说明

- ROS2 Humble 及以上
- MoveIt2
- MuJoCo 3.2.0 及其 ROS2 插件
- Franka Emika Panda 机械臂（仿真或实机）
- 详见上级目录 [README.md](../README.md) 获取完整依赖与安装说明

---

## 典型开发流程

1. 启动仿真环境：
   ```bash
   ./start_interactive_sim.sh
   ```
2. 新开终端，激活环境并运行主控脚本：
   ```bash
   source install/setup.bash
   python3 src/multipanda_ros2/scripts/test2.py
   ```
3. 如需调整参数，直接编辑 `test2.py`，保存后重启脚本即可生效。

---

## 参考文档

- [../README.md](../README.md)：项目总览、依赖与安装
- [../README_SOLUTION.md](../README_SOLUTION.md)：中文快速上手与常见问题
- [../IMPLEMENTATION_SUMMARY.md](../IMPLEMENTATION_SUMMARY.md)：方案原理与关键文件说明

---

如有问题欢迎提 issue 或 PR。
