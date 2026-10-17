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
        DeclareLaunchArgument(
            'image_topic',
            default_value='/zed/zed_node/rgb/color/rect/image',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/zed/zed_node/rgb/color/rect/camera_info',
        ),
        DeclareLaunchArgument('enabled_on_start', default_value='True'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(detector_launch / 'od.launch.py')),
            launch_arguments={
                'image_topic_list': [
                    '[', LaunchConfiguration('image_topic'), ']'
                ],
                'camera_info_topic_list': [
                    '[', LaunchConfiguration('camera_info_topic'), ']'
                ],
                'weights_pkg_name': 'sauvc_object_detection',
                'bbox_attrs_pkg_name': 'sauvc_object_detection',
                'weights_path': LaunchConfiguration('weights_path'),
                'debug': LaunchConfiguration('debug'),
                'enabled_on_start': LaunchConfiguration('enabled_on_start'),
            }.items(),
        ),
    ])
