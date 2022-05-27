from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def staticTransformNode(context, *args, **kwargs):

    config = os.path.join(get_package_share_directory('aruco_gate_detector'),
                          'config/tf_cam',
                          'real_arguments.yaml')

    with open(config, 'r') as config_file:
        tf_arguments = yaml.load(config_file, Loader=yaml.FullLoader)

    ns = LaunchConfiguration('drone_id').perform(context)

    base_link = ns + '/' + tf_arguments['frame_id']
    camera_link = ns + '/' + tf_arguments['child_frame_id']
    x = tf_arguments['x']
    y = tf_arguments['y']
    z = tf_arguments['z']
    roll = tf_arguments['roll']
    pitch = tf_arguments['pitch']
    yaw = tf_arguments['yaw']

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
    camera_ns = ns + '/sensor_measurements/camera'
    usbcam_params_path = os.path.join(get_package_share_directory('aruco_gate_detector'),'config/usb_cam','params.yaml')

    # print(usbcam_params_path)

    # usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2 -p framerate:=30.0 -p image_width:=1280 -p image_height:=720
    # --ros-args --params-file /path/to/colcon_ws/src/usb_cam/config/params.yaml
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        namespace=camera_ns,
        output='screen',
        emulate_tty=True,
        parameters=[usbcam_params_path]
    )

    return [camera_node]


def generate_launch_description(ns='drone0'):

    drone_id = DeclareLaunchArgument('drone_id', default_value='drone0')
    # video_device = DeclareLaunchArgument(
    #     'video_device', default_value='/dev/video0')
    # framerate = DeclareLaunchArgument('framerate', default_value='60.0')
    # image_width = DeclareLaunchArgument('image_width', default_value='1280')
    # image_height = DeclareLaunchArgument('image_height', default_value='720')
    # pixel_format = DeclareLaunchArgument('pixel_format', default_value='mjpeg')

    config = os.path.join(get_package_share_directory('aruco_gate_detector'),
                          'config/aruco_gate_detector',
                          'real_params.yaml')

    return LaunchDescription([
        drone_id,
        # video_device, framerate, image_width, image_height, pixel_format, 

        OpaqueFunction(function=cameraNode),

        Node(
            package='aruco_gate_detector',
            executable='aruco_gate_detector_node',
            name='aruco_gate_detector',
            namespace=LaunchConfiguration('drone_id'),
            parameters=[config],
            output='screen',
            emulate_tty=True
        ),

        OpaqueFunction(function=staticTransformNode)
    ])
