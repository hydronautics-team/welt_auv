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
        # front camera
        DeclareLaunchArgument("enable_front_camera",
                              default_value="false",
                              description="Включить (true) или отключить (false) ноду нижней камеры"),
        DeclareLaunchArgument("enable_recording_front_camera",
                              default_value="true",
                              description="Включить (true) или отключить (false) ноду нижней камеры"),
        DeclareLaunchArgument("front_camera_path",
                              default_value='/dev/video0'),
        DeclareLaunchArgument("front_camera_calibration_path",
                              default_value="package://welt_cam/configs/front_camera.yaml"),
        DeclareLaunchArgument('front_camera_output_width',
                              default_value='640',
                              description='Ширина видео'),
        DeclareLaunchArgument('front_camera_output_height',
                              default_value='360',
                              description='Высота видео'),
        # bottom camera
        DeclareLaunchArgument("enable_bottom_camera",
                              default_value="true",
                              description="Включить (true) или отключить (false) ноду нижней камеры"),
        DeclareLaunchArgument("enable_recording_bottom_camera",
                              default_value="true",
                              description="Включить(true) или отключить(false) ноду нижней камеры"),
        DeclareLaunchArgument("bottom_camera_path",
                              default_value='/dev/video4'),
        DeclareLaunchArgument("bottom_camera_calibration_path",
                              default_value="package://welt_cam/configs/bottom_camera.yaml"),
        DeclareLaunchArgument('bottom_camera_output_width',
                              default_value='640',
                              description='Ширина видео'),
        DeclareLaunchArgument('bottom_camera_output_height',
                              default_value='480',
                              description='Высота видео'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(
                get_package_share_directory('sauvc_launch'), 'cam.launch.py'))),
            launch_arguments={
                'enable_front_camera': LaunchConfiguration("enable_front_camera"),
                'enable_recording_front_camera': LaunchConfiguration("enable_recording_front_camera"),
                'front_camera_path': LaunchConfiguration("front_camera_path"),
                'front_camera_calibration_path': LaunchConfiguration("front_camera_calibration_path"),
                'front_camera_output_width': LaunchConfiguration("front_camera_output_width"),
                'front_camera_output_height': LaunchConfiguration("front_camera_output_height"),
                'enable_bottom_camera': LaunchConfiguration("enable_bottom_camera"),
                'enable_recording_bottom_camera': LaunchConfiguration("enable_recording_bottom_camera"),
                'bottom_camera_path': LaunchConfiguration("bottom_camera_path"),
                'bottom_camera_calibration_path': LaunchConfiguration("bottom_camera_calibration_path"),
                'bottom_camera_output_width': LaunchConfiguration("bottom_camera_output_width"),
                'bottom_camera_output_height': LaunchConfiguration("bottom_camera_output_height"),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(
                get_package_share_directory('sauvc_launch'), 'od.launch.py'))),
            launch_arguments={
                'debug': LaunchConfiguration("debug"),
            }.items(),
        ),
    ])
