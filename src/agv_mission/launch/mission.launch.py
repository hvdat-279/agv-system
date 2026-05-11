import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('agv_mission')
    
    return LaunchDescription([
        Node(
            package='agv_mission',
            executable='mission_controller',
            name='mission_controller',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config', 'mission_params.yaml')]
        )
    ])
