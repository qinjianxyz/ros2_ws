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

    pkg_name = 'ucsd_robocar_nav2_pkg'
    pkg_dir = '/home/projects/ros2_ws/src/ucsd_robocar_hub2/ucsd_robocar_nav2_pkg/rviz'
    config_file = 'sensor_visualization.rviz'
    all_components_launch = 'all_components.launch.py'
    
    config = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        config_file)
    ld = LaunchDescription()

    components_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(pkg_name),
                    'launch',
                    all_components_launch)
            )
        )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [config]])

    ld.add_action(components_launch)
    ld.add_action(rviz2_node)
    return ld
