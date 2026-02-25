# RViz 与 MuJoCo 同步问题诊断与解决方案

## 问题分析

### 问题 1: RViz 中只显示红色方块，看不到桌子和长铝条

**根本原因：**
- MuJoCo的场景文件（`dual_scene.xml`）中定义了环境物体（table、long_bar）
- 但这些物体**仅存在于MuJoCo物理仿真中**，没有在URDF中定义
- RViz只能显示URDF中定义的物体

**当前情况：**
```
MuJoCo (物理仿真)          RViz (可视化)
├─ Robot              ├─ Robot ✓
├─ Table ✓            ├─ Table ✗ (缺失)
├─ Long Bar ✓         ├─ Long Bar ✗ (缺失)
└─ Floor ✓            └─ Floor ✗ (缺失)
```

### 问题 2: 在RViz中拖动机械臂到桌子下方，MuJoCo中机械臂卡死

**根本原因：**
- RViz中的机械臂是通过MoveIt2规划的轨迹控制的
- 当你在RViz中拖动末端执行器时，改变了基座位置估计
- MuJoCo中的物理约束（碰撞检测）和RViz的预期位置不一致
- 导致控制器试图强制执行不可能的位置，机械臂被"锁定"

## 解决方案

### 方案 1: 将环境物体添加到URDF中（推荐）

这是正确的做法 - 让RViz和MuJoCo看到相同的场景。

**步骤 1: 修改 URDF XACRO 文件**

编辑 `franka_description/robots/sim/dual_panda_arm_sim.urdf.xacro`，在`base_link`之后添加环境物体：

```xml
  <link name="base_link">
    <visual>
      <geometry>
         <box size="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </visual>
    <collision>
      <geometry>
         <box size="0.1 0.1 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>
  </link>

  <!-- ==================== 环境物体 ==================== -->
  
  <!-- 桌子 -->
  <link name="table">
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.5 1.2 0.28"/>  <!-- 与MuJoCo相同: x=0.5, y=1.2, z=0.28 -->
      </geometry>
      <material name="table_material">
        <color rgba="0.6 0.4 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 1.2 0.28"/>
      </geometry>
    </collision>
  </link>

  <!-- 桌子与base_link的固定连接 -->
  <joint name="table_joint" type="fixed">
    <parent link="base_link"/>
    <child link="table"/>
    <origin xyz="0.5 0 0.2" rpy="0 0 0"/>  <!-- 与MuJoCo相同位置 -->
  </joint>

  <!-- 长条铝棒 -->
  <link name="long_bar">
    <inertial>
      <mass value="0.5"/>
      <!-- 长条物体: 长0.4m(Y), 宽0.04m(X), 高0.04m(Z) -->
      <inertia ixx="0.00268" ixy="0.0" ixz="0.0" iyy="0.00268" iyz="0.0" izz="0.00003"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.04 0.4 0.04"/>  <!-- X=0.04, Y=0.4, Z=0.04 (直径是size的2倍) -->
      </geometry>
      <material name="bar_material">
        <color rgba="0.8 0.8 0.9 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.04 0.4 0.04"/>
      </geometry>
    </collision>
  </link>

  <!-- 长条物体与base_link的连接 (使用floating base) -->
  <joint name="long_bar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="long_bar"/>
    <origin xyz="0.5 0 0.45" rpy="0 0 0"/>  <!-- 初始位置在桌子上方 -->
  </joint>

  <!-- ==================== 环境物体结束 ==================== -->

  <xacro:include filename="$(find franka_description)/robots/common/panda_arm.xacro"/>
  ...
```

**步骤 2: 验证MuJoCo场景中的坐标**

检查 `franka_description/mujoco/franka/dual_scene.xml` 中的位置参数，确保与上面URDF中的`<origin>`标签一致：

```xml
<!-- MuJoCo 中的定义 -->
<body name="table" pos="0.5 0 0.2">           <!-- 对应 xyz="0.5 0 0.2" -->
  <geom type="box" size="0.25 0.6 0.14" ... />  <!-- size是半长，所以实际=2倍 -->
</body>

<body name="long_bar" pos="0.5 0 0.45">       <!-- 对应 xyz="0.5 0 0.45" -->
  <geom name="bar_geom" type="box" size="0.02 0.2 0.02" ... />
</body>
```

### 方案 2: 在RViz中添加静态网格/碰撞对象（临时方案）

如果不想修改URDF，可以在RViz中手动添加对象显示：

1. 在RViz的`Displays`面板中点击"Add"
2. 选择"Marker"或"Mesh"
3. 手动定义table和long_bar的位置

**缺点：** 这只是可视化，不会影响MoveIt2的规划计算

## 解决同步问题的关键步骤

### 步骤 1: 确保正确的TF树结构

RViz和MuJoCo的坐标系必须一致：

```
base_link (MuJoCo和RViz的共同参考)
  ├─ mj_left_link0 -> ... -> mj_left_link8 (RViz显示)
  ├─ mj_right_link0 -> ... -> mj_right_link8 (RViz显示)
  ├─ table (新增)
  └─ long_bar (新增)
```

### 步骤 2: 使用joint_state_publisher同步状态

检查 `dual_franka_sim.launch.py` 中的配置：

```python
jsp_source_list = [concatenate_ns(ns, 'joint_states', True)]
if(load_gripper):
    jsp_source_list.append(concatenate_ns(ns, 'mj_left_gripper_sim_node/joint_states/joint_states', True))
    jsp_source_list.append(concatenate_ns(ns, 'mj_right_gripper_sim_node/joint_states/joint_states', True))
```

这确保了RViz从MuJoCo获取实时关节状态。

### 步骤 3: 防止RViz命令超出边界

修改 `sim_dual_moveit.launch.py` 中的约束：

```python
ompl_planning_pipeline_config['move_group'].update({
    'start_state_max_bounds_error': 0.1,
    'workspace_parameters': {
        'min_corner': [-1, -1, 0],      # 工作空间最小值
        'max_corner': [1, 1, 1.5]       # 工作空间最大值
    }
})
```

## 实施步骤

### 快速修复（推荐）

1. **备份原文件：**
   ```bash
   cp franka_description/robots/sim/dual_panda_arm_sim.urdf.xacro \
      franka_description/robots/sim/dual_panda_arm_sim.urdf.xacro.backup
   ```

2. **编辑URDF文件** - 添加环境物体（见上面的代码）

3. **重新构建和测试：**
   ```bash
   cd ~/franka_ws
   colcon build --packages-select franka_description
   source install/setup.bash
   ./start_interactive_sim.sh
   ```

4. **在RViz中验证：**
   - 应该看到红色的base_link（小方块）
   - 应该看到棕色的table（大平面）
   - 应该看到浅蓝色的long_bar（长条）
   - 三个物体的位置应该与MuJoCo中一致

### 如果RViz中仍然卡死

原因和解决方案：

| 问题 | 原因 | 解决方案 |
|------|------|--------|
| 机械臂卡在桌子里 | 碰撞检测过于敏感 | 在MoveIt2配置中增加padding参数 |
| 坐标不对齐 | URDF和MuJoCo的坐标系不同 | 检查所有`<origin>`标签和MuJoCo中的`pos`属性 |
| 双手臂不同步 | 关节状态发布器配置错误 | 检查`joint_state_publisher_gui`的订阅主题 |

## 调试命令

**查看TF树：**
```bash
ros2 run tf2_tools view_frames
# 生成frames.pdf，用PDF查看器打开查看
```

**监听关节状态：**
```bash
ros2 topic echo /joint_states
```

**检查MuJoCo物体状态：**
```bash
ros2 topic echo /mujoco/data/body_xpos
ros2 topic echo /mujoco/data/body_xquat
```

**查看MoveIt2规划参数：**
```bash
ros2 param list | grep moveit
ros2 param get /move_group start_state_max_bounds_error
```

## 文件位置参考

关键文件位置：
- URDF定义: `src/multipanda_ros2/franka_description/robots/sim/dual_panda_arm_sim.urdf.xacro`
- MuJoCo场景: `src/multipanda_ros2/franka_description/mujoco/franka/dual_scene.xml`
- MuJoCo对象: `src/multipanda_ros2/franka_description/mujoco/franka/objects.xml`
- 启动脚本: `start_interactive_sim.sh`
- MoveIt2启动: `src/multipanda_ros2/franka_moveit_config/launch/sim_dual_moveit.launch.py`
- RViz配置: `src/multipanda_ros2/franka_moveit_config/rviz/dual_moveit.rviz`

## 测试检查表

- [ ] RViz中看到table、long_bar、base_link三个物体
- [ ] 物体位置与MuJoCo中的位置相同
- [ ] 在RViz中拖动机械臂，MuJoCo中机械臂能正常跟随
- [ ] RViz命令机械臂避过table（碰撞检测生效）
- [ ] 双手臂状态同步（左右手臂位置一致）
- [ ] 夹爪状态正常（开/关同步）

---

**下一步建议：** 按照"快速修复"章节的步骤1-4操作，这应该能解决RViz和MuJoCo的同步问题。
