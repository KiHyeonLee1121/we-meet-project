"""Launch the locked, ground-only solenoid pulse service."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Build the solenoid bench launch description, locked by default."""
    package_share = get_package_share_directory('da_daka_control')
    parameter_file = os.path.join(
        package_share,
        'config',
        'solenoid_bench.yaml',
    )
    approved = LaunchConfiguration('bench_test_approved')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'bench_test_approved',
                default_value='false',
                description=(
                    'Unlock only after propeller removal and wet-side checks'
                ),
            ),
            Node(
                package='da_daka_control',
                executable='solenoid_bench',
                name='solenoid_bench',
                output='screen',
                parameters=[
                    parameter_file,
                    {
                        'bench_test_approved': ParameterValue(
                            approved,
                            value_type=bool,
                        )
                    },
                ],
            ),
        ]
    )
