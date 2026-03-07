from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # front camera
    front_camera_topic_arg = DeclareLaunchArgument(
        "front_camera_topic", default_value='/stingray/topics/camera/front'
    )
    front_camera_info_topic_arg = DeclareLaunchArgument(
        "front_camera_info_topic", default_value='/stingray/topics/camera/front/camera_info'
    )
    front_camera_path_arg = DeclareLaunchArgument(
        "front_camera_path", default_value='/dev/video0'
    )
    front_camera_calibration_path_arg = DeclareLaunchArgument(
        "front_camera_calibration_path", default_value="package://welt_cam/configs/front_camera.yaml"
    )
    # bottom camera
    bottom_camera_topic_arg = DeclareLaunchArgument(
        "bottom_camera_topic", default_value='/stingray/topics/camera/bottom'
    )
    bottom_camera_info_topic_arg = DeclareLaunchArgument(
        "bottom_camera_info_topic", default_value='/stingray/topics/camera/bottom/camera_info'
    )
    bottom_camera_path_arg = DeclareLaunchArgument(
        "bottom_camera_path", default_value='/dev/v4l/by-id/usb-Microsoft_Microsoft®_LifeCam_HD-3000-video-index0'
    )
    bottom_camera_calibration_path_arg = DeclareLaunchArgument(
        "bottom_camera_calibration_path", default_value="package://welt_cam/configs/bottom_camera.yaml"
    )

    # load ros config
    return LaunchDescription([
        front_camera_topic_arg,
        front_camera_info_topic_arg,
        front_camera_path_arg,
        front_camera_calibration_path_arg,
        bottom_camera_topic_arg,
        bottom_camera_info_topic_arg,
        bottom_camera_path_arg,
        bottom_camera_calibration_path_arg,

        Node(
            package='stingray_object_detection',
            executable='video_recorder',
            name='video_recorder',
            parameters=[
                {"source_topic": "/stingray/topics/camera/front"},
                {"output_width": 640},
                {"output_height": 480},
                {"output_fps": 30},
                {"output_format": 'mp4v'},
                {"record_dir": "./records/"},
            ],
            respawn=True,
            respawn_delay=1,
        )

    ])
