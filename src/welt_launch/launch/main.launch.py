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
    front_camera_topic_arg = DeclareLaunchArgument(
        "front_camera_topic", default_value='/stingray/topics/front_camera'
    )
    front_camera_path_arg = DeclareLaunchArgument(
        "front_camera_path", default_value='/dev/video4'
    )
    transition_srv_arg = DeclareLaunchArgument(
        "transition_srv", default_value='/stingray/services/transition'
    )
    zbar_topic_arg = DeclareLaunchArgument(
        "zbar_topic", default_value='/stingray/topics/zbar'
    )
    twist_action_arg = DeclareLaunchArgument(
        "twist_action", default_value='/stingray/actions/twist'
    )
    reset_imu_srv_arg = DeclareLaunchArgument(
        "reset_imu_srv", default_value='/stingray/services/reset_imu'
    )
    set_stabilization_srv_arg = DeclareLaunchArgument(
        "set_stabilization_srv", default_value='/stingray/services/set_stabilization'
    )
    send_to_ip_arg = DeclareLaunchArgument(
        "send_to_ip", default_value='192.168.1.11'
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
        transition_srv_arg,
        zbar_topic_arg,
        twist_action_arg,
        reset_imu_srv_arg,
        set_stabilization_srv_arg,
        send_to_ip_arg,
        send_to_port_arg,
        receive_from_ip_arg,
        receive_from_port_arg,
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(str(Path(
        #         get_package_share_directory('stingray_core_launch'), 'auv.launch.py'))),
        #     launch_arguments={
        #         'transition_srv': LaunchConfiguration("transition_srv"),
        #         'twist_action': LaunchConfiguration("twist_action"),
        #         'reset_imu_srv': LaunchConfiguration("reset_imu_srv"),
        #         'set_stabilization_srv': LaunchConfiguration("set_stabilization_srv"),
        #         'zbar_topic': LaunchConfiguration("zbar_topic"),
        #         'front_camera_topic': LaunchConfiguration("front_camera_topic"),
        #     }.items(),
        # ),
        Node(
            package='stingray_missions',
            executable='fsm_node',
            name='fsm_node',
            parameters=[
                {'transition_srv': LaunchConfiguration("transition_srv")},
                {'twist_action': LaunchConfiguration("twist_action")},
                {'reset_imu_srv': LaunchConfiguration("reset_imu_srv")},
                {'set_stabilization_srv': LaunchConfiguration(
                    "set_stabilization_srv")},
            ],
            respawn=True,
            respawn_delay=1,
        ),
        Node(
            package='stingray_movement',
            executable='twist_action_server',
            name='twist_action_server',
            respawn=True,
            respawn_delay=1,
        ),
        Node(
            package='stingray_missions',
            executable='qr_trigger_node',
            name='qr_trigger_node',
            parameters=[
                {'transition_srv': LaunchConfiguration("transition_srv")},
                {'zbar_topic': LaunchConfiguration("zbar_topic")}
            ],
            respawn=True,
            respawn_delay=1,
        ),
        Node(
            package='zbar_ros',
            executable='barcode_reader',
            name='qr_reader',
            remappings=[
                ('/image', LaunchConfiguration("front_camera_topic")),
                ('/barcode', LaunchConfiguration("zbar_topic")),
            ],
            respawn=True,
            respawn_delay=1,
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera_driver',
            remappings=[
                ('/image_raw', LaunchConfiguration("front_camera_topic")),
            ],
            parameters=[
                {'video_device': LaunchConfiguration("front_camera_path")},
                # {'camera_name': "/front"},
            ],
            respawn=True,
            respawn_delay=1,
        ),
        # Node(
        #     package='rqt_gui',
        #     executable='rqt_gui',
        #     name='rqt_gui',
        # ),
        Node(
            package='welt_communication',
            executable='hardware_bridge_node',
            name='hardware_bridge_node',
            respawn=True,
            respawn_delay=0.5,
        ),
        # Node(
        #     package='stingray_core_communication',
        #     executable='hardware_bridge_node',
        #     name='hardware_bridge_node',
        #     respawn=True,
        #     respawn_delay=0.5,
        # ),
        Node(
            package='welt_communication',
            executable='uart_driver_node',
            name='uart_driver_node',
            parameters=[
                {'device': "/dev/ttyTHS0"},
                {'baudrate': 115200},
            ],
            respawn=True,
            respawn_delay=0.5,
        ),
        # Node(
        #     package='welt_communication',
        #     executable='udp_driver_node',
        #     name='udp_driver_node',
        #     parameters=[
        #         {'send_to_ip': LaunchConfiguration("send_to_ip")},
        #         {'send_to_port': 13053},
        #         {'receive_from_ip': LaunchConfiguration("receive_from_ip")},
        #         {'receive_from_port': 13050},
        #     ],
        #     respawn=True,
        #     respawn_delay=0.5,
        # ),

    ])
