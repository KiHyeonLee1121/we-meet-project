"""Launch the configurable virtual distance sensor."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the virtual range sensor with its installed YAML profile."""
    package_share = get_package_share_directory('da_daka_control')
    parameter_file = os.path.join(
        package_share,
        'config',
        'virtual_distance_sensor.yaml',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'mode',
                default_value='constant',
                description='Virtual sensor mode',
            ),
            Node(
                package='da_daka_control',
                executable='virtual_distance_sensor',
                name='virtual_distance_sensor',
                output='screen',
                parameters=[parameter_file, {'mode': LaunchConfiguration('mode')}],
            )
        ]
    )
