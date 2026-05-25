from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _config(filename):
    return PathJoinSubstitution([
        FindPackageShare('sentinel_bringup'),
        'config',
        filename,
    ])


def generate_launch_description():
    enable_video_rx = LaunchConfiguration('enable_video_rx')
    enable_preprocess = LaunchConfiguration('enable_preprocess')
    enable_yolo_ir = LaunchConfiguration('enable_yolo_ir')
    enable_yolo_eo = LaunchConfiguration('enable_yolo_eo')
    enable_tracker_ir = LaunchConfiguration('enable_tracker_ir')
    enable_tracker_eo = LaunchConfiguration('enable_tracker_eo')
    enable_track_selector = LaunchConfiguration('enable_track_selector')

    video_rx_node = Node(
        package='video_rx_pkg2',
        executable='video_rx_node2',
        name='video_rx_node2',
        output='screen',
        parameters=[_config('video_rx.yaml')],
        condition=IfCondition(enable_video_rx),
    )

    image_preprocess_node = Node(
        package='image_preprocess_pkg',
        executable='image_preprocess_node',
        name='image_preprocess_node',
        output='screen',
        parameters=[_config('image_preprocess.yaml')],
        condition=IfCondition(enable_preprocess),
    )

    yolo_detector_ir_node = Node(
        package='yolo_detector_pkg',
        executable='ultralytics_yolo_node',
        name='yolo_detector_ir_node',
        output='screen',
        parameters=[_config('ultralytics_yolo_ir.yaml')],
        condition=IfCondition(enable_yolo_ir),
    )

    yolo_detector_eo_node = Node(
        package='yolo_detector_pkg',
        executable='yolo_detector_node',
        name='yolo_detector_eo_node',
        output='screen',
        parameters=[_config('yolo_detector_eo.yaml')],
        condition=IfCondition(enable_yolo_eo),
    )

    bytetrack_tracker_ir_node = Node(
        package='yolo_detector_pkg',
        executable='bytetrack_tracker_node',
        name='bytetrack_tracker_ir_node',
        output='screen',
        parameters=[_config('bytetrack_tracker_ir.yaml')],
        condition=IfCondition(enable_tracker_ir),
    )

    bytetrack_tracker_eo_node = Node(
        package='yolo_detector_pkg',
        executable='bytetrack_tracker_node',
        name='bytetrack_tracker_eo_node',
        output='screen',
        parameters=[_config('bytetrack_tracker_eo.yaml')],
        condition=IfCondition(enable_tracker_eo),
    )

    track_selector_node = Node(
        package='yolo_detector_pkg',
        executable='track_selector_node',
        name='track_selector_node',
        output='screen',
        parameters=[_config('track_selector.yaml')],
        condition=IfCondition(enable_track_selector),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_video_rx',
            default_value='true',
            description='Start EO capture-card and IR UDP video receiver node.',
        ),
        DeclareLaunchArgument(
            'enable_preprocess',
            default_value='true',
            description='Start IR/EO image preprocessing node.',
        ),
        DeclareLaunchArgument(
            'enable_yolo_ir',
            default_value='true',
            description='Start IR PyTorch YOLO detector node.',
        ),
        DeclareLaunchArgument(
            'enable_yolo_eo',
            default_value='true',
            description='Start EO PyTorch YOLO detector node.',
        ),
        DeclareLaunchArgument(
            'enable_tracker_ir',
            default_value='true',
            description='Start IR ByteTrack-style tracker node.',
        ),
        DeclareLaunchArgument(
            'enable_tracker_eo',
            default_value='true',
            description='Start EO ByteTrack-style tracker node.',
        ),
        DeclareLaunchArgument(
            'enable_track_selector',
            default_value='true',
            description='Start stream-aware track selector for driver detection.',
        ),
        video_rx_node,
        image_preprocess_node,
        yolo_detector_ir_node,
        yolo_detector_eo_node,
        bytetrack_tracker_ir_node,
        bytetrack_tracker_eo_node,
        track_selector_node,
    ])
