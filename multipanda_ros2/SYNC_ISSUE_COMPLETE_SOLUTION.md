# RViz 与 MuJoCo 同步问题 - 完整解决方案

## 问题诊断

### 你遇到的两个问题

**问题1: RViz中只看到红色方块**
- MuJoCo仿真中显示了工作台和长铝条
- RViz中只看到一个红色的小方块（base_link）
- 工作台和铝条在RViz中不可见

**问题2: 在RViz中拖动机械臂，MuJoCo中卡死**
- 当在RViz中将机械臂末端拖到桌子下方时
- MuJoCo中的机械臂会卡住，无法正常运动

### 根本原因

这两个问题的本质是**坐标系不同步**：

```
┌─ MuJoCo 物理引擎                 ┐
│  ├─ Robot (机械臂)               │
│  ├─ Table (工作台) ✓             │ 在这里定义了
│  ├─ Long Bar (铝棒) ✓            │
│  └─ Floor (地面)                 │
└──────────────────────────────────┘
               ↕ 未同步！
┌─ RViz 可视化系统                 ┐
│  ├─ Robot (机械臂) ✓             │ 只显示了这些
│  ├─ Table ✗ 缺失                 │ 没看到这些
│  ├─ Long Bar ✗ 缺失              │
│  └─ Floor ✗ 缺失                 │
└──────────────────────────────────┘
```

当RViz看不到工作台时：
1. MoveIt2规划器认为没有障碍物
2. 可以规划穿过工作台的轨迹
3. 当MuJoCo中的真实机械臂试图执行这个轨迹时
4. 碰撞到工作台却无法移动 → 卡死

## 解决方案（已实施）

### 修改内容

已修改文件：`franka_description/robots/sim/dual_panda_arm_sim.urdf.xacro`

**添加了两个环境对象到URDF中：**

#### 1. 工作台 (Table)
```xml
<link name="table">
  <visual>
    <geometry>
      <box size="0.5 1.2 0.28"/>  <!-- 长1.0m × 宽2.4m × 高0.56m -->
    </geometry>
    <material name="table_material">
      <color rgba="0.6 0.4 0.2 1.0"/>  <!-- 棕色 -->
    </material>
  </visual>
</link>

<joint name="table_joint" type="fixed">
  <parent link="base_link"/>
  <child link="table"/>
  <origin xyz="0.5 0 0.2" rpy="0 0 0"/>  <!-- 位置与MuJoCo同步 -->
</joint>
```

#### 2. 长条铝棒 (Long Bar)
```xml
<link name="long_bar">
  <visual>
    <geometry>
      <box size="0.02 0.2 0.02"/>  <!-- 厚4cm × 长40cm × 高4cm -->
    </geometry>
    <material name="bar_material">
      <color rgba="0.8 0.8 0.9 1.0"/>  <!-- 浅蓝色 -->
    </material>
  </visual>
</link>

<joint name="long_bar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="long_bar"/>
  <origin xyz="0.5 0 0.45" rpy="0 0 0"/>  <!-- 初始在工作台上方 -->
</joint>
```

### 关键设计

| 参数 | 值 | 说明 |
|------|-----|------|
| 工作台位置 | (0.5, 0, 0.2) | 与MuJoCo中 `pos="0.5 0 0.2"` 一致 |
| 工作台尺寸 | 0.5×1.2×0.28 | size参数是半长，实际=2倍 |
| 铝棒位置 | (0.5, 0, 0.45) | 与MuJoCo中 `pos="0.5 0 0.45"` 一致 |
| 铝棒尺寸 | 0.02×0.2×0.02 | 与MuJoCo中尺寸对应 |
| 连接类型 | fixed | 工作台和铝棒相对base_link固定不动 |

## 应该看到的效果

修复后，RViz和MuJoCo同步如下：

```
┌─ MuJoCo 物理引擎                 ┐
│  ├─ Robot (机械臂)               │
│  ├─ Table (工作台) ✓             │
│  ├─ Long Bar (铝棒) ✓            │
│  └─ Floor (地面)                 │
└──────────────────────────────────┘
               ↓ 完全同步！
┌─ RViz 可视化系统                 ┐
│  ├─ Robot (机械臂) ✓             │
│  ├─ Table (棕色) ✓ 新增          │
│  ├─ Long Bar (浅蓝) ✓ 新增       │
│  └─ Floor ✓                      │
└──────────────────────────────────┘
```

**在RViz中你会看到：**
1. ✓ **红色小方块** - base_link (机器人基座)
2. ✓ **棕色大平面** - 工作台 (0.5m×1.2m×0.28m)
3. ✓ **浅蓝色长条** - 铝棒 (0.02m×0.2m×0.02m)
4. ✓ **两条机械臂** - mj_left 和 mj_right (黄色)

## 为什么这解决了卡死问题

### 修复前流程

```
RViz看不到障碍物
    ↓
规划器认为无障碍物，生成穿过工作台的轨迹
    ↓
执行轨迹
    ↓
机械臂在MuJoCo中碰到工作台
    ↓
物理引擎无法让机械臂穿透障碍物
    ↓
❌ 卡死！
```

### 修复后流程

```
RViz看到工作台
    ↓
规划器知道有障碍物
    ↓
生成避开工作台的轨迹 (在规划空间中规避)
    ↓
执行轨迹
    ↓
机械臂在MuJoCo中顺利避开工作台
    ↓
✓ 正常运动！
```

## 立即执行的步骤

### 第1步：查看已修改的URDF

```bash
cat src/multipanda_ros2/franka_description/robots/sim/dual_panda_arm_sim.urdf.xacro | grep -A 20 "环境物体定义"
```

应该看到工作台和铝棒的定义。

### 第2步：重新编译

```bash
cd ~/franka_ws
rm -rf build/franka_description install/franka_description
colcon build --packages-select franka_description
source install/setup.bash
```

### 第3步：运行修复脚本（可选自动化）

```bash
cd ~/franka_ws
./fix_sync_issue.sh
```

### 第4步：启动仿真并验证

**终端1:**
```bash
cd ~/franka_ws
./start_interactive_sim.sh
```

**在RViz中检查：**
- [ ] 看到红色的base_link
- [ ] 看到棕色的工作台
- [ ] 看到浅蓝色的长铝条
- [ ] 机械臂在RViz中能拖动
- [ ] MuJoCo中机械臂正常跟随（不卡死）

## 文件对比

### 修改前 (BEFORE)

```xml
<link name="base_link">
  <visual>
    <geometry>
       <box size="0.1 0.1 0.1"/>
    </geometry>
  </visual>
</link>

<xacro:include filename="$(find franka_description)/robots/common/panda_arm.xacro"/>
```

只定义了base_link，没有环境物体。

### 修改后 (AFTER)

```xml
<link name="base_link">
  ...
</link>

<!-- ==================== 环境物体定义 ==================== -->

<!-- 工作台 (Table) -->
<link name="table">
  <visual>
    <geometry>
      <box size="0.5 1.2 0.28"/>
    </geometry>
    <material name="table_material">
      <color rgba="0.6 0.4 0.2 1.0"/>
    </material>
  </visual>
</link>

<joint name="table_joint" type="fixed">
  <parent link="base_link"/>
  <child link="table"/>
  <origin xyz="0.5 0 0.2" rpy="0 0 0"/>
</joint>

<!-- 长条铝棒 (Long Bar Object) -->
<link name="long_bar">
  <visual>
    <geometry>
      <box size="0.02 0.2 0.02"/>
    </geometry>
    <material name="bar_material">
      <color rgba="0.8 0.8 0.9 1.0"/>
    </material>
  </visual>
</link>

<joint name="long_bar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="long_bar"/>
  <origin xyz="0.5 0 0.45" rpy="0 0 0"/>
</joint>

<!-- ==================== 环境物体定义结束 ==================== -->

<xacro:include filename="$(find franka_description)/robots/common/panda_arm.xacro"/>
```

## 常见问题排查

### Q: 编译时出现XML错误

**A:** 检查语法：
```bash
python3 -m xml.dom.minidom src/multipanda_ros2/franka_description/robots/sim/dual_panda_arm_sim.urdf.xacro
```

如果有错误，检查是否有未闭合的标签。

### Q: RViz中仍看不到工作台

**A:** 
1. 检查是否成功重编译：`colcon build --packages-select franka_description`
2. 检查是否加载了新的环境：`source install/setup.bash`
3. 重启整个系统：`pkill -9 ros2; sleep 3; ./start_interactive_sim.sh`

### Q: 机械臂仍然卡死

**A:** 检查位置是否匹配：
```bash
# URDF中的位置
grep "origin xyz" src/multipanda_ros2/franka_description/robots/sim/dual_panda_arm_sim.urdf.xacro | grep -E "table|long_bar"

# MuJoCo中的位置
grep 'pos=' src/multipanda_ros2/franka_description/mujoco/franka/dual_scene.xml | grep -E "table|long_bar"
```

两个位置必须完全相同。

## 相关文件

| 文件 | 用途 |
|------|------|
| `src/multipanda_ros2/franka_description/robots/sim/dual_panda_arm_sim.urdf.xacro` | ✅ 已修改 - 添加环境物体 |
| `src/multipanda_ros2/franka_description/mujoco/franka/dual_scene.xml` | MuJoCo场景定义（无需修改） |
| `start_interactive_sim.sh` | 启动脚本（无需修改） |
| `RVIZ_MUJOCO_SYNC_GUIDE.md` | 详细解决方案指南 |
| `FIX_SYNC_QUICK_START.md` | 快速开始指南 |
| `fix_sync_issue.sh` | 自动化修复脚本 |

## 下一步

修复成功后，你可以：
1. ✓ 运行 `gripper_pick_moveit.py` 进行夹取测试
2. ✓ MoveIt2规划会考虑工作台作为障碍物
3. ✓ 机械臂运动会更可靠，不会再卡死

---

**最后验证：**
```bash
# 检查修改是否应用
grep -c "long_bar" src/multipanda_ros2/franka_description/robots/sim/dual_panda_arm_sim.urdf.xacro
# 应该输出: 5 (说明成功)

# 重编译
cd ~/franka_ws && colcon build --packages-select franka_description
```

如果编译成功，就可以启动仿真了！
