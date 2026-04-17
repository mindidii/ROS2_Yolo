from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    video_rx_config = PathJoinSubstitution([
        FindPackageShare('sentinel_bringup'),
        'config',
        'video_rx.yaml'
    ])

    yolo_config = PathJoinSubstitution([
        FindPackageShare('sentinel_bringup'),
        'config',
        'yolo_detector.yaml'
    ])

    video_rx_node = Node(
        package='video_rx_pkg',
        executable='video_rx_node',
        name='video_rx_node',
        output='screen',
        parameters=[video_rx_config],
    )

    yolo_detector_node = Node(
        package='yolo_detector_pkg',
        executable='yolo_detector_node',
        name='yolo_detector_node',
        output='screen',
        parameters=[yolo_config],
    )

    return LaunchDescription([
        video_rx_node,
        yolo_detector_node,
    ])