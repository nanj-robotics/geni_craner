"""
Geni Craner Real Robot MoveIt Launch File
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2",
        )
    )
    use_rviz = LaunchConfiguration("use_rviz")

    # URDF（你的机械臂）
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("geni_craner_description"), "urdf", "geni_craner.urdf.xacro"]),
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # SRDF
    robot_description_semantic_content = Command([
        "cat ",
        PathJoinSubstitution([FindPackageShare("geni_craner_moveit_config"), "config", "geni_craner.srdf"]),
    ])
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}

    # 运动学
    kinematics_yaml = load_yaml("geni_craner_moveit_config", "config/kinematics.yaml")

    # 关节限位
    joint_limits_yaml = load_yaml("geni_craner_moveit_config", "config/joint_limits.yaml")
    robot_description_planning = {"robot_description_planning": joint_limits_yaml}

    # OMPL
    ompl_planning_yaml = load_yaml("geni_craner_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config = {"move_group": ompl_planning_yaml}

    # MoveIt 控制器映射
    moveit_controllers_yaml = load_yaml("geni_craner_moveit_config", "config/moveit_controllers.yaml")

    # 轨迹执行参数
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 2.0,
        "trajectory_execution.allowed_goal_duration_margin": 1.0,
        "trajectory_execution.allowed_start_tolerance": 0.1,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # ros2_control 控制器配置
    ros2_controllers_yaml = PathJoinSubstitution([
        FindPackageShare("geni_craner_moveit_config"), "config", "ros2_controllers.yaml"
    ])

    # 节点
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_yaml],
        output="both",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager", "--controller-manager-timeout", "60"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager", "--controller-manager-timeout", "60"],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers_yaml,
            planning_scene_monitor_parameters,
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("geni_craner_moveit_config"), "config", "moveit.rviz"])],
        parameters=[robot_description, robot_description_semantic, robot_description_planning, {"robot_description_kinematics": kinematics_yaml},],
        condition=IfCondition(use_rviz),
    )

    # 顺序启动：先 broadcaster，再 arm controller，最后 move_group
    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    delay_move_group = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[move_group_node],
        )
    )

    nodes = [
        ros2_control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller,
        delay_move_group,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
