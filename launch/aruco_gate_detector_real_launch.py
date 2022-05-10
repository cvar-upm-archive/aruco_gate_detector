from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from math import pi

# CAMERA_LINK POSITION FROM BASE_LINK
x = 0.1
y = 0
z = 0.03

roll = -(90/180)*pi
pitch = 0
yaw = -(90/180)*pi


def staticTransformNode(context, *args, **kwargs):

    ns = LaunchConfiguration('drone_id').perform(context)
    base_link = ns + '/' + 'base_link'
    camera_link = ns + '/' + 'camera_link'

    static_transform_publisher_node = Node(
        # Tf from baselink to RGB cam
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_tf',
        namespace=LaunchConfiguration('drone_id'),
        arguments=[f'{x:.2f}', f'{y:.2f}', f'{z:.2f}', f'{roll:.2f}', f'{pitch:.2f}', f'{yaw:.2f}',
                   f'{base_link}', f'{camera_link}'],
        output='screen',
        emulate_tty=True,
    )

    return [static_transform_publisher_node]


def cameraNode(context, *args, **kwargs):

    ns = LaunchConfiguration('drone_id').perform(context)
    camera_ns = ns + '/' + 'camera1'

    # usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2 -p framerate:=30.0 -p image_width:=1280 -p image_height:=720
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node_exe',
        namespace=camera_ns,
        output='screen',
        emulate_tty=True,
        parameters=[
            {'video_device': LaunchConfiguration('video_device'),
             'framerate':    LaunchConfiguration('framerate'),
             'image_width':  LaunchConfiguration('image_width'),
             'image_height': LaunchConfiguration('image_height')
             }
        ]
    )

    return [camera_node]


def generate_launch_description(ns='drone0'):

    drone_id = DeclareLaunchArgument('drone_id', default_value='drone0')
    video_device = DeclareLaunchArgument(
        'video_device', default_value='/dev/video0')
    framerate = DeclareLaunchArgument('framerate', default_value='30.0')
    image_width = DeclareLaunchArgument('image_width', default_value='1280')
    image_height = DeclareLaunchArgument('image_height', default_value='720')

    return LaunchDescription([
        drone_id, video_device, framerate, image_width, image_height,

        OpaqueFunction(function=cameraNode),

        Node(
            package='aruco_gate_detector',
            executable='aruco_gate_detector_node',
            name='aruco_gate_detector',
            namespace=LaunchConfiguration('drone_id'),
            output='screen',
            emulate_tty=True
        ),

        OpaqueFunction(function=staticTransformNode)
    ])
