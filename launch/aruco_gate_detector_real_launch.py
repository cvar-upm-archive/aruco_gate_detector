from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description(ns='drone0'):

    drone_id = DeclareLaunchArgument('drone_id', default_value='drone0')

    config = os.path.join(get_package_share_directory('aruco_gate_detector'),
                          'config/aruco_gate_detector',
                          'real_params.yaml')
    # usbcam_camerainfo = os.path.join(get_package_share_directory('aruco_gate_detector'),
    #                                  'config/usb_cam',
    #                                  'camerainfo.yaml')

    usb_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('usb_camera_interface'),
            'launch', 'usb_camera_interface_launch.py'
        ])]),
    )

    return LaunchDescription([
        drone_id,
        usb_camera,

        Node(
            package='aruco_gate_detector',
            executable='aruco_gate_detector_node',
            namespace=LaunchConfiguration('drone_id'),
            parameters=[config],
            output='screen',
            emulate_tty=True,
            remappings=[
                ("sensor_measurements/camera/image_raw", "sensor_measurements/aruco_camera")]
        ),
    ])
