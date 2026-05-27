import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from xacro import process_file


def generate_launch_description():
    # Lấy đường dẫn chính xác đến package
    pkg_dir = get_package_share_directory('agv_description')

    # Đường dẫn đến file URDF
    urdf_file = os.path.join(pkg_dir, 'urdf', 'agv.urdf.xacro')

    print('Đang đọc file URDF từ:', urdf_file)  # In ra để kiểm tra

    # Xử lý file xacro
    doc = process_file(urdf_file)
    robot_desc = doc.toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
