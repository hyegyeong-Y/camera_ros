import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description(): 
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('camera_package'),
            'param',
            'filter.yaml')
        )
    
    # camera_server를 위한 parameter 파일 경로
    server_param_dir = LaunchConfiguration(
        'server_param_dir',
        default=os.path.join(
            get_package_share_directory('camera_package'),
            'param',
            'storage.yaml')
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir
        ),
        Node(
            package='camera_package',
            executable='canny',
            name='canny',
            parameters=[param_dir],
            output='screen'
        )

    ])
