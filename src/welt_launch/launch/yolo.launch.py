from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    detector_launch = Path(get_package_share_directory('stingray_launch'))

    return LaunchDescription([
        DeclareLaunchArgument('weights_path', default_value='/models/yolov8.pt'),
        DeclareLaunchArgument('debug', default_value='False'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(detector_launch / 'od.launch.py')),
            launch_arguments={
                'image_topic_list': '[/zed/zed_node/rgb/image_rect_color]',
                'camera_info_topic_list': '[/zed/zed_node/rgb/camera_info]',
                'weights_pkg_name': 'sauvc_object_detection',
                'bbox_attrs_pkg_name': 'sauvc_object_detection',
                'weights_path': LaunchConfiguration('weights_path'),
                'debug': LaunchConfiguration('debug'),
            }.items(),
        ),
    ])
