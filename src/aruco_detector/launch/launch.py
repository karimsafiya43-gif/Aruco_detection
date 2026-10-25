
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    # Launch the ZED 2i camera driver
    zed_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ),
        launch_arguments={
            'camera_model': 'zed2i'
        }.items()
    )

    # Launch our custom ArUco detector
    aruco_detector = Node(
        package='aruco_detector',
        executable='aruco_node',
        name='aruco_detector',
        output='screen',
        parameters=[{
            'image_topic': '/zed/zed_node/rgb/color/rect/image',
            'camera_info_topic': '/zed/zed_node/rgb/color/rect/camera_info',
            'marker_size': 0.107,
            'dictionary': 'DICT_7X7_250'
        }]
    )

    return LaunchDescription([
        zed_camera,
        aruco_detector
    ])
