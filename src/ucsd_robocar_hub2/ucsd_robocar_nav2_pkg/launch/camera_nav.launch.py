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
    lane_detection_launch = 'lane_detection.launch.py'
    mask_detection_package = 'mask_pkg'
    mask_detection_launch = 'mask_detection.launch.py'


    ld = LaunchDescription()
    components_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(nav_package),
                    'launch',
                    all_components_launch)
            )
        )

    '''
    cam_nav_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(lane_detection2_package),
                    'launch',
                    lane_detection_launch)
            )
        )
    '''

    mask_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
		os.path.join(
            get_package_share_directory(mask_detection_package),
            'launch',
            mask_detection_launch)
        )
	)

    ld.add_action(components_launch)
    # ld.add_action(cam_nav_launch)
    ld.add_action(mask_launch)
    return ld
