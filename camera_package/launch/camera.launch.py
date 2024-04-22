import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # img_publisher를 위한 parameter 파일 경로
    img_param_dir = LaunchConfiguration(
        'img_param_dir',
        default=os.path.join(
            get_package_share_directory('camera_package'),
            'param',
            'size.yaml')
    )

    # camera_server를 위한 parameter 파일 경로
    server_param_dir = LaunchConfiguration(
        'server_param_dir',
        default=os.path.join(
            get_package_share_directory('camera_package'),
            'param',
            'storage.yaml')
    )

    video_param_dir = LaunchConfiguration(
        'server_param_dir',
        default=os.path.join(
            get_package_share_directory('camera_package'),
            'param',
            'video.yaml')
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'img_param_dir',
            default_value=img_param_dir
        ),
        DeclareLaunchArgument(
            'server_param_dir',
            default_value=server_param_dir
        ),
         DeclareLaunchArgument(
            'video_param_dir',
            default_value=video_param_dir
        ),

        #camera_server를 주석 처리하고, 대신에 capture_client 추가
        Node(
            package='camera_package',
            executable='camera_server',
            name='camera_server',
            parameters=[server_param_dir],
            output='screen'
        ),

        Node(
            package='camera_package',
            executable='img_publisher',
            name='img_publisher',
            parameters=[img_param_dir],
            output='screen'
        ),

         Node(
            package='camera_package',
            executable='video_server',
            name='video_server',
            parameters=[video_param_dir],
            output='screen'
        )
    ])
