from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory(
        'zed_aruco_detector'
    )

    params_file = os.path.join(
        package_share,
        'config',
        'aruco_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='zed_aruco_detector',
            executable='aruco',
            name='zed_aruco_depth_node',
            output='screen',
            parameters=[params_file],
        )
    ])
