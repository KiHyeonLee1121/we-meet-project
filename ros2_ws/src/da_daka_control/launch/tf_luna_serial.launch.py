"""Launch the physical TF-Luna serial driver."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the TF-Luna serial range sensor node."""
    package_share = get_package_share_directory('da_daka_control')
    parameter_file = os.path.join(
        package_share,
        'config',
        'tf_luna_serial.yaml',
    )

    return LaunchDescription(
        [
            Node(
                package='da_daka_control',
                executable='tf_luna_serial',
                name='tf_luna_serial',
                output='screen',
                parameters=[parameter_file],
            )
        ]
    )
