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
    # missions
    package_names_arg = DeclareLaunchArgument(
        "package_names", default_value='stingray_missions sauvc_missions'
    )
    transition_srv_arg = DeclareLaunchArgument(
        "transition_srv", default_value='/stingray/services/transition'
    )
    zbar_topic_arg = DeclareLaunchArgument(
        "zbar_topic", default_value='/stingray/topics/zbar'
    )

    # movement
    twist_action_arg = DeclareLaunchArgument(
        "twist_action", default_value='/stingray/actions/twist'
    )
    uv_state_topic_arg = DeclareLaunchArgument(
        "uv_state_topic", default_value='/stingray/topics/uv_state'
    )
    set_twist_srv_arg = DeclareLaunchArgument(
        "set_twist_srv", default_value='/stingray/services/set_twist'
    )

    # devices
    device_action_arg = DeclareLaunchArgument(
        "device_action", default_value='/stingray/actions/device'
    )
    device_state_array_topic_arg = DeclareLaunchArgument(
        "device_state_array_topic", default_value='/stingray/topics/device_state_array'
    )
    set_device_srv_arg = DeclareLaunchArgument(
        "set_device_srv", default_value='/stingray/services/set_device'
    )

    # object detection
    image_topic_list_arg = DeclareLaunchArgument(
        "image_topic_list", default_value='/stingray/topics/front_camera'
    )
    set_enable_object_detection_srv_arg = DeclareLaunchArgument(
        "set_enable_object_detection_srv", default_value='/stingray/services/set_twist'
    )
    weights_pkg_name_arg = DeclareLaunchArgument(
        "weights_pkg_name", default_value='stingray_object_detection'
    )
    debug_arg = DeclareLaunchArgument(
        "debug", default_value='True'
    )

    # core
    # hardware bridge
    # topic names
    driver_request_topic_arg = DeclareLaunchArgument(
        "driver_request_topic", default_value='/stingray/topics/driver_request'
    )
    driver_response_topic_arg = DeclareLaunchArgument(
        "driver_response_topic", default_value='/stingray/topics/driver_response'
    )

    # service names
    set_stabilization_srv_arg = DeclareLaunchArgument(
        "set_stabilization_srv", default_value='/stingray/services/set_stabilization'
    )
    reset_imu_srv_arg = DeclareLaunchArgument(
        "reset_imu_srv", default_value='/stingray/services/reset_imu'
    )
    enable_thrusters_srv_arg = DeclareLaunchArgument(
        "enable_thrusters_srv", default_value='/stingray/services/enable_thrusters'
    )


    # udp driver
    send_to_ip_arg = DeclareLaunchArgument(
        "send_to_ip", default_value='192.168.1.111'
    )
    send_to_port_arg = DeclareLaunchArgument(
        "send_to_port", default_value='13053'
    )
    receive_from_ip_arg = DeclareLaunchArgument(
        "receive_from_ip", default_value='192.168.1.173'
    )
    receive_from_port_arg = DeclareLaunchArgument(
        "receive_from_port", default_value='13050'
    )
    

    # load ros config
    return LaunchDescription([
        front_camera_topic_arg,
        front_camera_path_arg,
        package_names_arg,
        transition_srv_arg,
        zbar_topic_arg,
        twist_action_arg,
        uv_state_topic_arg,
        set_twist_srv_arg,
        device_action_arg,
        device_state_array_topic_arg,
        set_device_srv_arg,
        driver_request_topic_arg,
        driver_response_topic_arg,
        reset_imu_srv_arg,
        set_stabilization_srv_arg,
        enable_thrusters_srv_arg,
        set_device_srv_arg,
        image_topic_list_arg,
        set_enable_object_detection_srv_arg,
        weights_pkg_name_arg,
        debug_arg,
        send_to_ip_arg,
        send_to_port_arg,
        receive_from_ip_arg,
        receive_from_port_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(
                get_package_share_directory('stingray_core_launch'), 'uart.launch.py'))),
            launch_arguments={
                'driver_request_topic': LaunchConfiguration("driver_request_topic"),
                'uv_state_topic': LaunchConfiguration("uv_state_topic"),
                'device_state_array_topic': LaunchConfiguration("device_state_array_topic"),
                'driver_response_topic': LaunchConfiguration("driver_response_topic"),
                'set_twist_srv': LaunchConfiguration("set_twist_srv"),
                'set_stabilization_srv': LaunchConfiguration("set_stabilization_srv"),
                'reset_imu_srv': LaunchConfiguration("reset_imu_srv"),
                'enable_thrusters_srv': LaunchConfiguration("enable_thrusters_srv"),
                'set_device_srv': LaunchConfiguration("set_device_srv"),
                'driver_request_topic': LaunchConfiguration("driver_request_topic"),
                'driver_response_topic': LaunchConfiguration("driver_response_topic"),
                'device': LaunchConfiguration("device"),
                'baudrate': LaunchConfiguration("baudrate"),
            }.items(),
        ),
        Node(
            package='welt_communication',
            executable='hardware_bridge_node',
            name='hardware_bridge_node',
            parameters=[
                {'driver_request_topic': LaunchConfiguration("driver_request_topic")},
                {'uv_state_topic': LaunchConfiguration("uv_state_topic")},
                {'device_state_array_topic': LaunchConfiguration("device_state_array_topic")},
                {'driver_response_topic': LaunchConfiguration("driver_response_topic")},
                {'set_twist_srv': LaunchConfiguration("set_twist_srv")},
                {'set_stabilization_srv': LaunchConfiguration("set_stabilization_srv")},
                {'reset_imu_srv': LaunchConfiguration("reset_imu_srv")},
                {'enable_thrusters_srv': LaunchConfiguration("enable_thrusters_srv")},
                {'set_device_srv': LaunchConfiguration("set_device_srv")},
            ],
            respawn=True,
            respawn_delay=0.5,
        ),
        Node(
            package='welt_communication',
            executable='udp_driver_node',
            name='udp_driver_node',
            parameters=[
                {'send_to_ip': '10.42.0.111'},
                {'send_to_port': 13053},
                {'receive_from_ip': "10.42.0.186"},
                {'receive_from_port': 13050},
            ],
            respawn=True,
            respawn_delay=0.5,
        ),
    ])
