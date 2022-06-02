from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import yaml


def staticTransformNode(context, *args, **kwargs):
    tf_cam_config = LaunchConfiguration('tf_cam_config').perform(context)

    with open(tf_cam_config, 'r') as config_file:
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

    camera_info = LaunchConfiguration('camera_info').perform(context)
    camera_params = LaunchConfiguration('camera_params').perform(context)

    # usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2 -p framerate:=30.0 -p image_width:=1280 -p image_height:=720
    # --ros-args --params-file /path/to/colcon_ws/src/usb_cam/config/params.yaml
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        namespace=camera_ns,
        output='screen',
        emulate_tty=True,
        parameters=[camera_info, camera_params]
    )

    return [camera_node]


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('aruco_gate_detector'),
        'config/aruco_gate_detector', 'real_params.yaml'
    ])
    tf_cam_config = PathJoinSubstitution([
        FindPackageShare('aruco_gate_detector'),
        'config/tf_cam', 'real_arguments.yaml'
    ])
    camera_info = PathJoinSubstitution([
        FindPackageShare('aruco_gate_detector'),
        'config/usb_cam', 'camera_info.yaml'
    ])
    camera_params = PathJoinSubstitution([
        FindPackageShare('aruco_gate_detector'),
        'config/usb_cam', 'params.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('tf_cam_config', default_value=tf_cam_config),
        DeclareLaunchArgument('camera_info', default_value=camera_info),
        DeclareLaunchArgument('camera_params', default_value=camera_params),
        DeclareLaunchArgument('log_level', default_value='info'),

        OpaqueFunction(function=cameraNode),
        
        Node(
            package='aruco_gate_detector',
            executable='aruco_gate_detector_node',
            namespace=LaunchConfiguration('drone_id'),
            parameters=[LaunchConfiguration('config')],
            output='screen',
            emulate_tty=True,
            remappings=[("sensor_measurements/camera/image_raw", "camera1/image_raw")]
        ),

        OpaqueFunction(function=staticTransformNode)
    ])
