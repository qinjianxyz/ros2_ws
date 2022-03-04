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
    actuator_pkg ='ucsd_robocar_actuator2_pkg'
    # all_components_launch = 'all_components.launch.py'
    
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory(nav_pkg),
        'config',
        some_config)

    vesc_twist_node=Node(
        package=actuator_pkg,
        executable='vesc_twist_node',
        name='vesc_twist_node')

    joy_node=Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters = [config])

    joy_teleop_twist_node=Node(
        package='teleop_twist_joy', 
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters = [config])

    # ld.add_action(components_launch)
    ld.add_action(vesc_twist_node)
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_twist_node)
    return ld
