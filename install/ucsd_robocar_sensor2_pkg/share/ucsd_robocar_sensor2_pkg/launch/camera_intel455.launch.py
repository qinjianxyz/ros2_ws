import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    some_package = 'realsense_ros2_camera'
    some_node = 'realsense_ros2_camera'

    original_topic_name = '/camera/color/image_raw'
    new_topic_name = LaunchConfiguration('topic_name', default=original_topic_name)

    ld = LaunchDescription()
    node=Node(
        package=some_package,
        node_executable=some_node,
        output='screen',
        remappings=[(original_topic_name,new_topic_name)]
    )
    ld.add_action(node)
    return ld
