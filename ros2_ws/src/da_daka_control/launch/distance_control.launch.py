"""Launch the distance filter and the disabled-by-default controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build the distance-processing launch description."""
    package_share = get_package_share_directory('da_daka_control')
    filter_config = os.path.join(package_share, 'config', 'distance_filter.yaml')
    controller_config = os.path.join(
        package_share,
        'config',
        'distance_controller.yaml',
    )

    return LaunchDescription(
        [
            Node(
                package='da_daka_control',
                executable='distance_filter',
                name='distance_filter',
                output='screen',
                parameters=[filter_config],
            ),
            Node(
                package='da_daka_control',
                executable='distance_controller',
                name='distance_controller',
                output='screen',
                parameters=[controller_config],
            ),
        ]
    )
