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
    nav_package = 'ucsd_robocar_nav2_pkg'
    lane_detection2_package = 'ucsd_robocar_lane_detection2_pkg'
    all_components_launch = 'all_components.launch.py'
    cal_node_name = 'calibration_node'
    calibration_file = 'ros_racer_calibration.yaml'

    config = os.path.join(
        get_package_share_directory(lane_detection2_package),
        'config',
        calibration_file)
    
    ld = LaunchDescription()
    components_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(nav_package),
                    'launch',
                    all_components_launch)
            )
        )
    
    calibration_node = Node(
        package=lane_detection2_package,
        executable=cal_node_name,
        output='screen',
        parameters=[config])

    ld.add_action(components_launch)
    ld.add_action(calibration_node)
    return ld
