from pathlib import Path
from datetime import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    records_dir = (
        str(Path('records') / 
            f"record_{datetime.now().strftime('%Y_%m_%d-%H_%M_%S')}")
    )
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-s', 'mcap',
                '-o', records_dir,
                '/vectornav/imu',
                '/stingray_core/pressure_sensor/depth',
                '/dvl/data',
                '/welt/camera/front/image_raw/compressed',
                '/welt/camera/bottom/image_raw/compressed',
            ],
            output='screen'
        )
    ])