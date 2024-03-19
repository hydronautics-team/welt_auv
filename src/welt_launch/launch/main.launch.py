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
   
    # core
    # hardware bridge
    # topic names
    driver_request_topic_arg = DeclareLaunchArgument(
        "driver_request_topic", default_value='/stingray/topics/driver_request'
    )
    driver_response_topic_arg = DeclareLaunchArgument(
        "driver_response_topic", default_value='/stingray/topics/driver_response'
    )
    uv_state_topic_arg = DeclareLaunchArgument(
        "uv_state_topic", default_value='/stingray/topics/uv_state'
    )
    device_state_array_topic_arg = DeclareLaunchArgument(
        "device_state_array_topic", default_value='/stingray/topics/device_state_array'
    )

    # service names
    set_twist_srv_arg = DeclareLaunchArgument(
        "set_twist_srv", default_value='/stingray/services/set_twist'
    )
    set_stabilization_srv_arg = DeclareLaunchArgument(
        "set_stabilization_srv", default_value='/stingray/services/set_stabilization'
    )
    reset_imu_srv_arg = DeclareLaunchArgument(
        "reset_imu_srv", default_value='/stingray/services/reset_imu'
    )
    enable_thrusters_srv_arg = DeclareLaunchArgument(
        "enable_thrusters_srv", default_value='/stingray/services/enable_thrusters'
    )
    set_device_srv_arg = DeclareLaunchArgument(
        "set_device_srv", default_value='/stingray/services/set_device'
    )

    # udp driver
    send_to_ip_arg = DeclareLaunchArgument(
        "send_to_ip", default_value='10.42.0.111'
    )
    send_to_port_arg = DeclareLaunchArgument(
        "send_to_port", default_value='13053'
    )
    receive_from_ip_arg = DeclareLaunchArgument(
        "receive_from_ip", default_value='10.42.0.186'
    )
    receive_from_port_arg = DeclareLaunchArgument(
        "receive_from_port", default_value='13050'
    )

    # load ros config
    return LaunchDescription([
        uv_state_topic_arg,
        set_twist_srv_arg,
        device_state_array_topic_arg,
        reset_imu_srv_arg,
        set_stabilization_srv_arg,
        enable_thrusters_srv_arg,
        set_device_srv_arg,
        driver_request_topic_arg,
        driver_response_topic_arg,
        send_to_ip_arg,
        send_to_port_arg,
        receive_from_ip_arg,
        receive_from_port_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(
                get_package_share_directory('sauvc_launch'), 'main.launch.py'))),
            launch_arguments={
                'uv_state_topic': LaunchConfiguration("uv_state_topic"),
                'device_state_array_topic': LaunchConfiguration("device_state_array_topic"),
                'set_twist_srv': LaunchConfiguration("set_twist_srv"),
                'set_stabilization_srv': LaunchConfiguration("set_stabilization_srv"),
                'reset_imu_srv': LaunchConfiguration("reset_imu_srv"),
                'enable_thrusters_srv': LaunchConfiguration("enable_thrusters_srv"),
                'set_device_srv': LaunchConfiguration("set_device_srv"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(
                get_package_share_directory('welt_launch'), 'com.launch.py'))),
            launch_arguments={
                'driver_request_topic': LaunchConfiguration("driver_request_topic"),
                'driver_response_topic': LaunchConfiguration("driver_response_topic"),
                'uv_state_topic': LaunchConfiguration("uv_state_topic"),
                'device_state_array_topic': LaunchConfiguration("device_state_array_topic"),
                'set_twist_srv': LaunchConfiguration("set_twist_srv"),
                'set_stabilization_srv': LaunchConfiguration("set_stabilization_srv"),
                'reset_imu_srv': LaunchConfiguration("reset_imu_srv"),
                'enable_thrusters_srv': LaunchConfiguration("enable_thrusters_srv"),
                'set_device_srv': LaunchConfiguration("set_device_srv"),
                'send_to_ip': LaunchConfiguration("send_to_ip"),
                'send_to_port': LaunchConfiguration("send_to_port"),
                'receive_from_ip': LaunchConfiguration("receive_from_ip"),
                'receive_from_port': LaunchConfiguration("receive_from_port"),
            }.items(),
        ),
    ])
