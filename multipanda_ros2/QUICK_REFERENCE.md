# ⚡ 快速命令参考卡

## 🚀 3条命令快速启动

```bash
# Terminal 1 - 启动仿真（20-30秒）
cd ~/franka_ws && ./start_interactive_sim.sh

# Terminal 2 - 运行夹取演示（等Terminal 1完成后）
cd ~/franka_ws && source install/setup.bash && python3 src/multipanda_ros2/scripts/gripper_pick_v1.py

# Terminal 3（可选）- 交互式调试
cd ~/franka_ws && source install/setup.bash && python3 src/multipanda_ros2/scripts/interactive_gripper_debug.py
```

---

## 🎮 交互式调试工具命令

```
>> state              # 查看当前关节角度
>> home               # 回到初始位置
>> open               # 打开夹爪
>> close              # 关闭夹爪
>> approach           # 靠近物体
>> pick               # 执行完整夹取
>> move L 0.0 -0.785 0.0 -2.356 0.0 1.57 0.785    # 移动左臂
>> move R 0.0 -0.785 0.0 -2.356 0.0 1.57 0.785    # 移动右臂
>> exit               # 退出
```

---

## 🔧 ROS 诊断命令

```bash
# 检查节点是否运行
ros2 node list | grep -E "mujoco|move|rviz"

# 查看所有话题
ros2 topic list

# 实时查看关节状态
ros2 topic echo /joint_states

# 检查轨迹控制器
ros2 action info /dual_panda_arm_controller/follow_joint_trajectory

# 列出所有可用的Action
ros2 action list
```

---

## 📚 文档速查

| 需要 | 文件 |
|------|------|
| 快速开始 | `README_SOLUTION.md` ← 👈 从这里开始 |
| 新旧对比 | `QUICK_START_COMPARISON.md` |
| 完整方案 | `IMPLEMENTATION_SUMMARY.md` |
| 代码讲解 | `GRIPPER_IMPLEMENTATION_GUIDE.md` |

---

## 🐛 常见问题速修

| 问题 | 原因 | 解决 |
|------|------|------|
| "轨迹控制器不可用" | 仿真没启 | `./start_interactive_sim.sh` |
| "无法获取关节状态" | 仿真没启 | 等待仿真完全启动 |
| 机械臂不动 | 关节名称错 | `ros2 topic echo /joint_states` |
| 夹爪不工作 | 夹爪关节名错 | 在关节状态中查找夹爪名称 |
| RViz黑屏 | X11转发 | 检查DISPLAY环境变量 |

---

## 💾 关键文件位置

```
scripts/
├── gripper_pick_v1.py              ← 运行这个
├── interactive_gripper_debug.py    ← 或这个
└── dual_arm_demo.py                ← 不用这个 ❌

docs/
├── README_SOLUTION.md              ← 从这里开始
├── QUICK_START_COMPARISON.md       
├── IMPLEMENTATION_SUMMARY.md       
└── GRIPPER_IMPLEMENTATION_GUIDE.md
```

---

## ⏱️ 时间表

| 活动 | 时间 |
|------|------|
| 运行演示看工作 | 5分钟 |
| 尝试调试工具 | 15分钟 |
| 阅读快速对比 | 15分钟 |
| 理解代码原理 | 30分钟 |
| 自己修改参数 | 20分钟 |
| 自己写代码 | 60分钟 |

---

## ✅ 你现在拥有

- ✅ 完整可工作的夹取脚本
- ✅ 交互式调试工具
- ✅ 详细的代码讲解
- ✅ 从零开始的代码框架
- ✅ 所有常见问题的答案

---

## 🎯 推荐流程

1. **现在** → 运行 `gripper_pick_v1.py` 验证工作
2. **5分钟后** → 尝试 `interactive_gripper_debug.py`
3. **20分钟后** → 读 `QUICK_START_COMPARISON.md`
4. **45分钟后** → 读 `GRIPPER_IMPLEMENTATION_GUIDE.md`
5. **1小时30分后** → 自己写修改代码

---

**现在就开始！** 🚀

```bash
./start_interactive_sim.sh &
sleep 30
source install/setup.bash
python3 src/multipanda_ros2/scripts/gripper_pick_v1.py
```
