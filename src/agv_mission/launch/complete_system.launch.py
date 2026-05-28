import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('agv_gazebo')
    nav_pkg = get_package_share_directory('agv_navigation')
    perception_pkg = get_package_share_directory('agv_perception')
    mission_pkg = get_package_share_directory('agv_mission')

    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'headless': headless}.items()
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'use_rviz': use_rviz}.items()
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                perception_pkg, 'launch', 'perception.launch.py'
            )
        )
    )

    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mission_pkg, 'launch', 'mission.launch.py')
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Whether to run Gazebo in headless mode'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to start RViz'
        ),
        gazebo_launch,
        # Chờ gazebo khởi động xong mới bật Nav2 (10s)
        TimerAction(
            period=10.0,
            actions=[nav_launch]
        ),
        # Chờ gazebo khởi động xong mới bật Perception (5s)
        TimerAction(
            period=5.0,
            actions=[perception_launch]
        ),
        # Bật mission controller cuối cùng (15s)
        TimerAction(
            period=15.0,
            actions=[mission_launch]
        )
    ])
