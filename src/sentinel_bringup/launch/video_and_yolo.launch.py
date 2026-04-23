from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_video_rx = LaunchConfiguration('enable_video_rx')
    enable_preprocess = LaunchConfiguration('enable_preprocess')
    enable_yolo_ir = LaunchConfiguration('enable_yolo_ir')
    enable_yolo_eo = LaunchConfiguration('enable_yolo_eo')
    enable_overlay_ir = LaunchConfiguration('enable_overlay_ir')
    enable_overlay_eo = LaunchConfiguration('enable_overlay_eo')

    video_rx_config = PathJoinSubstitution([
        FindPackageShare('sentinel_bringup'),
        'config',
        'video_rx.yaml'
    ])

    image_preprocess_config = PathJoinSubstitution([
        FindPackageShare('sentinel_bringup'),
        'config',
        'image_preprocess.yaml'
    ])

    yolo_ir_config = PathJoinSubstitution([
        FindPackageShare('sentinel_bringup'),
        'config',
        'yolo_detector_ir.yaml'
    ])

    yolo_eo_config = PathJoinSubstitution([
        FindPackageShare('sentinel_bringup'),
        'config',
        'yolo_detector_eo.yaml'
    ])

    video_rx_node = Node(
        package='video_rx_pkg',
        executable='video_rx_node',
        name='video_rx_node',
        output='screen',
        parameters=[video_rx_config],
        condition=IfCondition(enable_video_rx),
    )

    image_preprocess_node = Node(
        package='image_preprocess_pkg',
        executable='image_preprocess_node',
        name='image_preprocess_node',
        output='screen',
        parameters=[image_preprocess_config],
        condition=IfCondition(enable_preprocess),
    )

    yolo_detector_ir_node = Node(
        package='yolo_detector_pkg',
        executable='yolo_detector_node',
        name='yolo_detector_ir_node',
        output='screen',
        parameters=[yolo_ir_config],
        condition=IfCondition(enable_yolo_ir),
    )

    yolo_detector_eo_node = Node(
        package='yolo_detector_pkg',
        executable='yolo_detector_node',
        name='yolo_detector_eo_node',
        output='screen',
        parameters=[yolo_eo_config],
        condition=IfCondition(enable_yolo_eo),
    )

    bbox_overlay_ir_node = Node(
        package='yolo_detector_pkg',
        executable='bbox_overlay_node',
        name='bbox_overlay_ir_node',
        output='screen',
        parameters=[{
            'image_topic': '/yolo/ir/image_raw',
            'detection_topic': '/detections/ir',
            'annotated_image_topic': '/yolo/ir/annotated_image',
            'sync_queue_size': 30,
            'line_thickness': 2,
            'font_scale': 0.5,
            'min_score': 0.0,
        }],
        condition=IfCondition(enable_overlay_ir),
    )

    bbox_overlay_eo_node = Node(
        package='yolo_detector_pkg',
        executable='bbox_overlay_node',
        name='bbox_overlay_eo_node',
        output='screen',
        parameters=[{
            'image_topic': '/yolo/eo/image_raw',
            'detection_topic': '/detections/eo',
            'annotated_image_topic': '/yolo/eo/annotated_image',
            'sync_queue_size': 30,
            'line_thickness': 2,
            'font_scale': 0.5,
            'min_score': 0.0,
        }],
        condition=IfCondition(enable_overlay_eo),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_video_rx',
            default_value='true',
            description='Start UDP video receiver node.',
        ),
        DeclareLaunchArgument(
            'enable_preprocess',
            default_value='true',
            description='Start IR/EO image preprocessing node.',
        ),
        DeclareLaunchArgument(
            'enable_yolo_ir',
            default_value='true',
            description='Start IR YOLO detector node.',
        ),
        DeclareLaunchArgument(
            'enable_yolo_eo',
            default_value='true',
            description='Start EO YOLO detector node.',
        ),
        DeclareLaunchArgument(
            'enable_overlay_ir',
            default_value='true',
            description='Start IR bbox overlay image node.',
        ),
        DeclareLaunchArgument(
            'enable_overlay_eo',
            default_value='true',
            description='Start EO bbox overlay image node.',
        ),
        video_rx_node,
        image_preprocess_node,
        yolo_detector_ir_node,
        yolo_detector_eo_node,
        bbox_overlay_ir_node,
        bbox_overlay_eo_node,
    ])
