import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
import yaml


def generate_launch_description():
    node_package = 'ucsd_robocar_control2_pkg'
    config_file = 'lqr_config.yaml'
    node_name = 'lqr_node'

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory(node_package),
        'config',
        config_file)

    control_node = Node(
        package=node_package,
        executable=node_name,
        output='screen',
        parameters=[config])

    ld.add_action(control_node)
    return ld
