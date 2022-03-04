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
    some_config = 'joy_teleop.yaml'
    nav_pkg ='ucsd_robocar_nav2_pkg'
    path_pkg ='ucsd_robocar_path2_pkg'
    all_components_launch = 'all_components.launch.py'
    
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory(nav_pkg),
        'config',
        some_config)
    
    components_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(nav_package),
                    all_components_launch)
            )
        )

    simple_obstacle_detection_node=Node(
        package=path_pkg,
        executable='simple_obstacle_detection_node',
        name='simple_obstacle_detection_node')
    
    ld.add_action(simple_obstacle_detection_node)
    return ld
