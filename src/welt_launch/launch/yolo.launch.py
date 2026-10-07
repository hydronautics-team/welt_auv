from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    detector_launch = Path(get_package_share_directory('stingray_launch'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic_list',
            default_value=(
                '[/zed/zed_node/rgb/color/rect/image,'
                '/stingray/topics/camera/bottom]'),
        ),
        DeclareLaunchArgument(
            'camera_info_topic_list',
            default_value=(
                '[/zed/zed_node/rgb/color/rect/camera_info,'
                '/stingray/topics/camera/bottom/camera_info]'),
        ),
        DeclareLaunchArgument('enable_bottom_camera', default_value='True'),
        DeclareLaunchArgument(
            'bottom_camera_device', default_value='/dev/video2'),
        DeclareLaunchArgument(
            'bottom_camera_info_url',
            default_value='package://welt_cam/configs/bottom_camera.yaml'),
        DeclareLaunchArgument('bottom_camera_width', default_value='640'),
        DeclareLaunchArgument('bottom_camera_height', default_value='480'),
        DeclareLaunchArgument('bottom_camera_framerate', default_value='30.0'),
        DeclareLaunchArgument('weights_path', default_value='/models/yolov8.pt'),
        DeclareLaunchArgument('debug', default_value='False'),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='bottom_camera_node',
            remappings=[
                ('/image_raw', '/stingray/topics/camera/bottom'),
                ('/camera_info',
                 '/stingray/topics/camera/bottom/camera_info'),
            ],
            parameters=[{
                'video_device': LaunchConfiguration('bottom_camera_device'),
                'camera_info_url': LaunchConfiguration(
                    'bottom_camera_info_url'),
                'camera_name': 'bottom_camera',
                'image_width': LaunchConfiguration('bottom_camera_width'),
                'image_height': LaunchConfiguration('bottom_camera_height'),
                'framerate': LaunchConfiguration('bottom_camera_framerate'),
            }],
            respawn=True,
            respawn_delay=1.0,
            condition=IfCondition(LaunchConfiguration('enable_bottom_camera')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(detector_launch / 'od.launch.py')),
            launch_arguments={
                'image_topic_list': LaunchConfiguration('image_topic_list'),
                'camera_info_topic_list': LaunchConfiguration(
                    'camera_info_topic_list'),
                'weights_pkg_name': 'sauvc_object_detection',
                'bbox_attrs_pkg_name': 'sauvc_object_detection',
                'weights_path': LaunchConfiguration('weights_path'),
                'debug': LaunchConfiguration('debug'),
            }.items(),
        ),
    ])
