import os
from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    # load ros config
    return LaunchDescription([
        Node(
            package='welt_communication',
            executable='hardware_bridge_node',
            name='hardware_bridge_node',
            respawn=True,
            respawn_delay=0.5,
        ),
        Node(
            package='welt_communication',
            executable='udp_driver_node',
            name='udp_driver_node',
            parameters=[
                {'send_to_ip': '192.168.1.11'},
                {'send_to_port': 13053},
                {'receive_from_ip': "192.168.1.173"},
                {'receive_from_port': 13050},
            ],
            respawn=True,
            respawn_delay=0.5,
        ),
        
    ])
