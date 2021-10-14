from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='drone0'),
        DeclareLaunchArgument('video_device', default_value='/dev/video2'),
        DeclareLaunchArgument('framerate', default_value='30.0'),
        DeclareLaunchArgument('image_width', default_value='1280'),
        DeclareLaunchArgument('image_height', default_value='720'),
        Node(
            # usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2 -p framerate:=30.0 -p image_width:=1280 -p image_height:=720
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node_exe',
            namespace=LaunchConfiguration('drone_id'),
            output='screen',
            emulate_tty=True,
            parameters=[
              {'video_device':LaunchConfiguration('video_device'),
              'framerate':LaunchConfiguration('framerate'),
              'image_width':LaunchConfiguration('image_width'),
              'image_height':LaunchConfiguration('image_height')
              }
            ]
        ),
        Node(
            package='aruco_gate_detector',
            executable='aruco_gate_detector_node',
            name='aruco_gate_detector',
            namespace=LaunchConfiguration('drone_id'),
            output='screen',
            emulate_tty=True
        )
    ])
