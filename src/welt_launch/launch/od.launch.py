from pathlib import Path

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    return LaunchDescription([
        # object detection
        DeclareLaunchArgument("debug",
                              default_value='True'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(
                get_package_share_directory('welt_launch'), 'cam.launch.py'))),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(
                get_package_share_directory('sauvc_launch'), 'od.launch.py'))),
            launch_arguments={
                'debug': LaunchConfiguration("debug"),
            }.items(),
        ),
    ])
