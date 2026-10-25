
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='zed_wrapper',
            executable='zed_wrapper',
            name='zed_node',
            namespace='zed',
            output='screen',
            parameters=[{
                'general.camera_model': 'zed2i',
            }],
        )
    ])

