from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    welt_launch = Path(get_package_share_directory('welt_launch'))
    planning_launch = Path(get_package_share_directory('stingray_planning'))

    record_arg = DeclareLaunchArgument(
        'record',
        default_value='false',
        description='Recording mcap data from topics (true/false)'
    )

    control_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(welt_launch / 'control.launch.py')),
            )
    planning_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(planning_launch / 'launch' / 'planning.launch.py')),
            )
    record_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(welt_launch / 'record_rosbag.launch.py')),
                IfCondition(LaunchConfiguration('record'))
            )

    return LaunchDescription([
        control_launch,
        planning_launch,
        record_arg,
        record_launch
    ])
