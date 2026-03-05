#  Copyright (c) 2021 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
def concatenate_ns(ns1, ns2, absolute=False):
    
    if(len(ns1) == 0):
        return ns2
    if(len(ns2) == 0):
        return ns1
    
    # check for /s at the end and start
    if(ns1[0] == '/'):
        ns1 = ns1[1:]
    if(ns1[-1] == '/'):
        ns1 = ns1[:-1]
    if(ns2[0] == '/'):
        ns2 = ns2[1:]
    if(ns2[-1] == '/'):
        ns2 = ns2[:-1]
    if(absolute):
        ns1 = '/' + ns1
    return ns1 + '/' + ns2

def generate_launch_description():
    arm_id_1_param = "arm_id_1"
    arm_id_2_param = "arm_id_2"
    initial_positions_1_param = 'initial_positions_1'
    initial_positions_2_param = 'initial_positions_2'
    use_rviz_param = 'use_rviz'
    publish_fallback_target_tf_param = 'publish_fallback_target_tf'
    fallback_bar_x_param = 'fallback_bar_x'
    fallback_bar_y_param = 'fallback_bar_y'
    fallback_bar_z_param = 'fallback_bar_z'
    fallback_bar_yaw_param = 'fallback_bar_yaw'
    mujoco_verbose_param = 'mujoco_verbose'
    mujoco_headless_param = 'mujoco_headless'
    mujoco_realtime_param = 'mujoco_realtime'
    mujoco_threads_param = 'mujoco_threads'

    arm_id_1 = LaunchConfiguration(arm_id_1_param)
    arm_id_2 = LaunchConfiguration(arm_id_2_param)
    initial_positions_1 = LaunchConfiguration(initial_positions_1_param)
    initial_positions_2 = LaunchConfiguration(initial_positions_2_param)
    use_rviz = LaunchConfiguration(use_rviz_param)
    publish_fallback_target_tf = LaunchConfiguration(publish_fallback_target_tf_param)
    fallback_bar_x = LaunchConfiguration(fallback_bar_x_param)
    fallback_bar_y = LaunchConfiguration(fallback_bar_y_param)
    fallback_bar_z = LaunchConfiguration(fallback_bar_z_param)
    fallback_bar_yaw = LaunchConfiguration(fallback_bar_yaw_param)
    mujoco_verbose = LaunchConfiguration(mujoco_verbose_param)
    mujoco_headless = LaunchConfiguration(mujoco_headless_param)
    mujoco_realtime = LaunchConfiguration(mujoco_realtime_param)
    mujoco_threads = LaunchConfiguration(mujoco_threads_param)

    # Fixed variables
    load_gripper = True # We make gripper a fixed variable, mainly because parsing the argument 
                        # within generate_launch_description is a fairly unintuitive process, 
                        # and it's not worth doing just for a single boolean.
                        
    if(load_gripper): # mujoco scene file must be manually adjusted since there's no way to pass parameters
        scene_file = 'dual_scene.xml'
    else:
        scene_file = 'dual_scene_ng.xml'
    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots', 'sim',
                                     "dual_panda_arm_sim.urdf.xacro")
    xml_file = os.path.join(get_package_share_directory('franka_description'), 'mujoco', 'franka', scene_file)
    mjros_config_file = os.path.join(get_package_share_directory('franka_bringup'), 'config', 'sim',
                                     'dual_sim_controllers.yaml')
    franka_bringup_path = get_package_share_directory('franka_bringup')
    ns=""

    # Robot state publisher setup
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, 
            ' arm_id_1:=', arm_id_1, 
            ' arm_id_2:=', arm_id_2,
            ' hand_1:=', str(load_gripper).lower(),
            ' hand_2:=', str(load_gripper).lower(),
            ' initial_positions_1:=', initial_positions_1,
            ' initial_positions_2:=', initial_positions_2])

    params = {'robot_description': robot_description}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace= ns,
        parameters=[params, {'use_sim_time': True}]
    )

    # Joint state publisher setup
    jsp_source_list = [concatenate_ns(ns, 'joint_states', True)]
    if(load_gripper):
        jsp_source_list.append(concatenate_ns(ns, 'mj_left_gripper_sim_node/joint_states/joint_states', True))
        jsp_source_list.append(concatenate_ns(ns, 'mj_right_gripper_sim_node/joint_states/joint_states', True))

    node_joint_state_publisher = Node( # RVIZ dependency
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace= ns,
            parameters=[
                {'source_list': jsp_source_list,
                 'rate': 30,
                 'use_sim_time': True}],
    )

    # 兜底TF（默认启用）：
    # - base_link -> long_bar（默认桌面中心位姿）
    # - long_bar -> target_bar（零偏移别名）
    # 这样即使感知/外部TF未启动，任务脚本也能立即拿到目标帧。
    node_long_bar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fallback_long_bar_tf_pub',
        arguments=[
            '--x', fallback_bar_x,
            '--y', fallback_bar_y,
            '--z', fallback_bar_z,
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', fallback_bar_yaw,
            '--frame-id', 'base_link',
            '--child-frame-id', 'long_bar',
        ],
        output='screen',
        condition=IfCondition(publish_fallback_target_tf),
    )
    node_target_bar_alias_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fallback_target_bar_tf_pub',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'long_bar',
            '--child-frame-id', 'target_bar',
        ],
        output='screen',
        condition=IfCondition(publish_fallback_target_tf),
    )

    # Others
    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_dual_franka.rviz')
    

    return LaunchDescription([
        # Launch args
        DeclareLaunchArgument(
            use_rviz_param,
            default_value='false',
            description='Visualize the robot in Rviz'),
        DeclareLaunchArgument(
            publish_fallback_target_tf_param,
            default_value='true',
            description='Publish fallback base_link->long_bar and long_bar->target_bar static TFs.'
        ),
        DeclareLaunchArgument(
            fallback_bar_x_param,
            default_value='0.5',
            description='Fallback long_bar x in base_link frame.'
        ),
        DeclareLaunchArgument(
            fallback_bar_y_param,
            default_value='0.0',
            description='Fallback long_bar y in base_link frame.'
        ),
        DeclareLaunchArgument(
            fallback_bar_z_param,
            default_value='0.36',
            description='Fallback long_bar z in base_link frame.'
        ),
        DeclareLaunchArgument(
            fallback_bar_yaw_param,
            default_value='0.0',
            description='Fallback long_bar yaw (rad) in base_link frame.'
        ),
        DeclareLaunchArgument(
            mujoco_verbose_param,
            default_value='true',
            description='MuJoCo server verbose log level.'
        ),
        DeclareLaunchArgument(
            mujoco_headless_param,
            default_value='false',
            description='Run MuJoCo in headless mode.'
        ),
        DeclareLaunchArgument(
            mujoco_realtime_param,
            default_value='1.0',
            description='MuJoCo realtime factor (0,1], -1 as fast as possible.'
        ),
        DeclareLaunchArgument(
            mujoco_threads_param,
            default_value='2',
            description='MuJoCo simulation thread count.'
        ),
        DeclareLaunchArgument(
            arm_id_1_param,
            default_value='mj_left',
            description='Unique name of robot 1.'
        ),
        DeclareLaunchArgument(
            arm_id_2_param,
            default_value='mj_right',
            description='Unique name of robot 2.'
        ),
        DeclareLaunchArgument(
            initial_positions_1_param,
            default_value='"0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"',
            description='Initial joint positions of robot 1. Must be enclosed in quotes, and in pure number.'
                        'Defaults to the "communication_test" pose.'),
        DeclareLaunchArgument(
            initial_positions_2_param,
            default_value='"0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"',
            description='Initial joint positions of robot 2. Must be enclosed in quotes, and in pure number.'
                        'Defaults to the "communication_test" pose.'),

        # Mujoco ros2 server launch
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(franka_bringup_path + '/launch/sim/launch_mujoco_ros_server.launch'),
            launch_arguments={
                'use_sim_time': "true",
                'modelfile': xml_file,
                'verbose': mujoco_verbose,
                'headless': mujoco_headless,
                'realtime': mujoco_realtime,
                'mujoco_threads': mujoco_threads,
                'ns': ns,
                'mujoco_plugin_config': mjros_config_file
                # 'mujoco_plugin_config': os.path.join(mjr2_control_path, 'example', 'ros2_control_plugins_example.yaml')

            }.items()
        ),

        # Miscellaneous
        node_robot_state_publisher,
        node_joint_state_publisher,
        node_long_bar_tf,
        node_target_bar_alias_tf,

        Node( # RVIZ dependency
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '-c', concatenate_ns(ns, 'controller_manager', True),
                '--controller-manager-timeout', '120'
            ],
            output='screen',
        ),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file],
             condition=IfCondition(use_rviz)
             )

    ])
