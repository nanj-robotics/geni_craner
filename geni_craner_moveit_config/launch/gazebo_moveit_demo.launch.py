from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from moveit_configs_utils.launches import generate_moveit_rviz_launch

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    moveit_config = MoveItConfigsBuilder(
        "geni_craner",
        package_name="geni_craner_moveit_config"
    ).to_moveit_configs()

    ld = generate_move_group_launch(moveit_config)
    ld.add_action(generate_moveit_rviz_launch(moveit_config))

    # 强制所有节点用仿真时间
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))

    for action in ld.entities:
        if isinstance(action, Node):
            action.parameters.append({'use_sim_time': use_sim_time})

    return ld

