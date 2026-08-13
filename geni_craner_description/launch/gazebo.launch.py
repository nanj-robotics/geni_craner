import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    package_name = 'geni_craner_description'

    # ========== 1. 加载带 Gazebo 控制插件的完整机器人模型 ==========
    xacro_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'geni_craner.urdf.xacro'
    )
    robot_desc = Command(['xacro ', xacro_path])

    # 机器人生成位置
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.0'

    # ========== 2. 启动 Gazebo 仿真环境 ==========
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'verbose': 'false'}.items()
    )

    # ========== 3. 发布机器人状态与 TF 树 ==========
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # ========== 4. 将机器人生成到 Gazebo 中 ==========
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'geni_craner',
            '-x', spawn_x_val,
            '-y', spawn_y_val,
            '-z', spawn_z_val,
            '-Y', spawn_yaw_val
        ],
        output='screen'
    )

    # ========== 5. 加载并激活 ros2_control 控制器 ==========
    # 关节状态广播器：发布关节角度，给 MoveIt/RViz 做状态反馈
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # 关节轨迹控制器：接收 MoveIt 的轨迹指令，驱动机械臂运动
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
    )

    # 事件：机器人生成成功后再加载控制器（顺序不能乱）
    load_controllers_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster, load_joint_trajectory_controller]
        )
    )

    # ========== 汇总所有节点 ==========
    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        load_controllers_event
    ])

