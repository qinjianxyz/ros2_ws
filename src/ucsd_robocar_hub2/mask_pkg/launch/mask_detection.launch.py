import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def generate_launch_description():

    mask_detection_package = 'mask_pkg'
    mask_node_name = 'mask_node'
    ld = LaunchDescription()

    mask_detection_node = Node(
        package=mask_detection_package,
        executable=mask_node_name,
        output='screen')

    ld.add_action(mask_detection_node)

    return ld
