import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from xacro import process_file


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('agv_gazebo')
    pkg_description = get_package_share_directory('agv_description')

    world_file = os.path.join(pkg_gazebo, 'worlds', 'warehouse.sdf')
    world_path = LaunchConfiguration('world')

    model_path = os.path.join(pkg_gazebo, 'models')
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if gz_resource_path:
        gz_resource_path = model_path + ':' + gz_resource_path
    else:
        gz_resource_path = model_path

    urdf_file = os.path.join(pkg_description, 'urdf', 'agv.urdf.xacro')
    controllers_file = os.path.join(pkg_description, 'config', 'controllers.yaml')
    robot_desc = process_file(
        urdf_file,
        mappings={'controller_config': controllers_file},
    ).toxml()

    bridge_config = os.path.join(pkg_gazebo, 'config', 'gazebo_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='Path to the Gazebo world file',
        ),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py',
                )
            ),
            launch_arguments={'gz_args': [TextSubstitution(text='-r '), world_path]}.items(),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': robot_desc,
                    'use_sim_time': True,
                    'publish_robot_description': True,
                }
            ],
        ),

        Node(
            package='agv_gazebo',
            executable='robot_description_publisher.py',
            name='robot_description_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        ),


        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'agv',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.1',
            ],
            output='screen',
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            output='screen',
            parameters=[{'config_file': bridge_config}],
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager',
                '/controller_manager',
                '--param-file',
                controllers_file,
            ],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'diff_drive_controller',
                '--controller-manager',
                '/controller_manager',
                '--param-file',
                controllers_file,
            ],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'lifter_controller',
                '--controller-manager',
                '/controller_manager',
                '--param-file',
                controllers_file,
            ],
            output='screen',
        ),

        # Static transforms to bridge Gazebo sensor frame names
        # to ROS 2 URDF frame names.
        # Gazebo collapses fixed joints and renames frames to:
        #   model_name/parent_link/sensor_name
        # but ROS 2 TF tree uses URDF link names (lidar_link, camera_link).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_frame_bridge',
            output='screen',
            arguments=[
                '--frame-id', 'lidar_link',
                '--child-frame-id', 'agv/base_link/lidar',
            ],
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_frame_bridge',
            output='screen',
            arguments=[
                '--frame-id', 'camera_link',
                '--child-frame-id', 'agv/base_link/camera',
            ],
            parameters=[{'use_sim_time': True}],
        ),
    ])
