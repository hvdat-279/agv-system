from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package='agv_perception',
                executable='color_detector',
                name='color_detector',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ]
    )
