"""Launch the airborne spray-reaction test with its valve and feedforward."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Build the fail-closed airborne spray-test launch description."""
    package_share = get_package_share_directory('da_daka_control')

    def config(name: str) -> str:
        return os.path.join(package_share, 'config', name)

    spray_output_enabled = LaunchConfiguration('spray_output_enabled')
    spray_backend = LaunchConfiguration('spray_backend')
    feedforward_output_enabled = LaunchConfiguration(
        'feedforward_output_enabled'
    )
    hover_height_m = LaunchConfiguration('hover_height_m')
    use_feedforward = LaunchConfiguration('use_feedforward')

    return LaunchDescription(
        [
            # Both physical gates stay closed unless explicitly opened, so a
            # bare launch commands nothing through AUX5.
            DeclareLaunchArgument(
                'spray_output_enabled', default_value='false'
            ),
            DeclareLaunchArgument('spray_backend', default_value='mock'),
            DeclareLaunchArgument(
                'feedforward_output_enabled', default_value='false'
            ),
            DeclareLaunchArgument('hover_height_m', default_value='3.0'),
            DeclareLaunchArgument('use_feedforward', default_value='true'),
            Node(
                package='da_daka_control',
                executable='spray_controller',
                name='spray_controller',
                output='screen',
                parameters=[
                    config('spray_controller.yaml'),
                    {
                        'backend': ParameterValue(
                            spray_backend, value_type=str
                        ),
                        'output_enabled': ParameterValue(
                            spray_output_enabled, value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package='da_daka_control',
                executable='spray_reaction_compensator',
                name='spray_reaction_compensator',
                output='screen',
                parameters=[
                    config('spray_reaction_compensator.yaml'),
                    {
                        'output_enabled': ParameterValue(
                            feedforward_output_enabled, value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package='da_daka_control',
                executable='spray_hover_test',
                name='spray_hover_test',
                output='screen',
                parameters=[
                    config('spray_hover_test.yaml'),
                    {
                        'hover_height_m': ParameterValue(
                            hover_height_m, value_type=float
                        ),
                        'use_feedforward': ParameterValue(
                            use_feedforward, value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
