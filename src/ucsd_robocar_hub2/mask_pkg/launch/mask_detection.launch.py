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

    mask_detection_package = 'mask_pkg'
    mask_node_name = 'mask_node'
    ld = LaunchDescription()


    config = os.path.join(
        get_package_share_directory(mask_detection_package),
        'config')

    mask_detection_node = Node(
        package=mask_detection_package,
        executable=mask_node_name,
        output='screen',
        parameters=[config])

    ld.add_action(mask_detection_node)

    return ld
