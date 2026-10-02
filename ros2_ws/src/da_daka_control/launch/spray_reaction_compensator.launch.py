"""Launch spray-reaction feedforward alone for propellers-off diagnostics."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Build the fail-closed standalone feedforward launch description."""
    package_share = get_package_share_directory('da_daka_control')
    config = os.path.join(
        package_share,
        'config',
        'spray_reaction_compensator.yaml',
    )
    output_enabled = LaunchConfiguration('output_enabled')
    return LaunchDescription(
        [
            DeclareLaunchArgument('output_enabled', default_value='false'),
            Node(
                package='da_daka_control',
                executable='spray_reaction_compensator',
                name='spray_reaction_compensator',
                output='screen',
                parameters=[
                    config,
                    {
                        'output_enabled': ParameterValue(
                            output_enabled,
                            value_type=bool,
                        ),
                    },
                ],
            ),
        ]
    )
