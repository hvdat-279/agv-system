import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    pkg_nav = get_package_share_directory('agv_navigation')

    map_file = LaunchConfiguration('map')
    nav2_params = LaunchConfiguration('nav2_params_file')
    ekf_params = LaunchConfiguration('ekf_params_file')
    rviz_config = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

    localization_launch = os.path.join(pkg_nav, 'launch', 'localization.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_nav, 'maps', 'warehouse_map.yaml'),
            description='Path to map yaml file',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(pkg_nav, 'config', 'nav2_params.yaml'),
            description='Path to Nav2 parameter file',
        ),
        DeclareLaunchArgument(
            'ekf_params_file',
            default_value=os.path.join(pkg_nav, 'config', 'ekf.yaml'),
            description='Path to EKF parameter file',
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(pkg_nav, 'rviz', 'nav2.rviz'),
            description='Path to RViz config file',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to start RViz',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                'map': map_file,
                'nav2_params_file': nav2_params,
                'ekf_params_file': ekf_params,
                'rviz_config_file': rviz_config,
                'use_rviz': use_rviz,
            }.items(),
        ),
    ])
