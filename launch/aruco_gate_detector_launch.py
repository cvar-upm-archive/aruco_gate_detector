from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from math import pi

# CAMERA_LINK POSITION FROM BASE_LINK
x = 0.1
y = 0
z = 0.03
roll  = -(90/180)*pi
pitch = 0
yaw   = -(90/180)*pi
base_link   = 'base_link'
camera_link = 'camera_link'

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
        ),
        # Node( 
        #     package='tf_publishser',
        #     node_executable='tf_publishser',
        #     node_name='tf_publishser1'
        # ),
        Node(
            # Tf from baselink to RGB cam
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_tf',
            namespace=LaunchConfiguration('drone_id'),
            # arguments=['0.5397', '0.4037', '0.0', '0.920504942522', '0.390730918652', '5.18419788842e-07', '1.22132126011e-06', 'base_link', 'laser_0']
            arguments=[f'{x:.2f}', f'{y:.2f}', f'{z:.2f}', f'{roll:.2f}', f'{pitch:.2f}', f'{yaw:.2f}', f'{base_link}', f'{camera_link}'],
            output='screen',
            emulate_tty=True,
        )
    ])
