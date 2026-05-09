import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from xacro import process_file


def generate_launch_description() -> LaunchDescription:
    pkg_nav = get_package_share_directory('agv_navigation')
    pkg_description = get_package_share_directory('agv_description')

    urdf_file = os.path.join(pkg_description, 'urdf', 'agv.urdf.xacro')
    robot_desc = process_file(urdf_file).toxml()

    slam_params = LaunchConfiguration('slam_params_file')
    ekf_params = LaunchConfiguration('ekf_params_file')
    rviz_config = LaunchConfiguration('rviz_config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg_nav, 'config', 'slam_toolbox.yaml'),
            description='Path to SLAM Toolbox parameter file',
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

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_frame_alias_publisher',
            output='screen',
            arguments=['0.2', '0.0', '0.15', '0', '0', '0', 'base_link', 'agv/base_link/lidar'],
            parameters=[{'use_sim_time': True}],
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': ['slam_toolbox'],
                }
            ],
        ),

        Node(
            package='nav2_map_server',
            executable='map_saver_server',
            name='map_saver_server',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
        ),
    ])
