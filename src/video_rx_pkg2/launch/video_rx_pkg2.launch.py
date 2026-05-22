from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('video_rx_pkg2'),
        'config',
        'video_rx_pkg2.yaml',
    ])

    return LaunchDescription([
        Node(
            package='video_rx_pkg2',
            executable='video_rx_node2',
            name='video_rx_node2',
            output='screen',
            parameters=[config],
        ),
    ])
