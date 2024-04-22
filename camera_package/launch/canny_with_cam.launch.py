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

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Full path to the parameter file'
        ),
        Node(
            package='camera_package',
            executable='canny_with_cam',
            name='canny_with_cam',
            parameters=[param_dir],  # Pass the parameter file path
            output='screen'
        )
    ])