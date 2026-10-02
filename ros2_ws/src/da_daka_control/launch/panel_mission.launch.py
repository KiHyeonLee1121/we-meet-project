"""Launch the independent panel movement mission and safety nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build an inert-on-start panel mission launch description."""
    package_share = get_package_share_directory('da_daka_control')
    controller_config = os.path.join(
        package_share,
        'config',
        'distance_controller.yaml',
    )
    mission_config = os.path.join(
        package_share,
        'config',
        'panel_mission.yaml',
    )
    guard_config = os.path.join(
        package_share,
        'config',
        'altitude_guard.yaml',
    )

    return LaunchDescription(
        [
            Node(
                package='da_daka_control',
                executable='distance_controller',
                name='distance_controller',
                output='screen',
                parameters=[
                    controller_config,
                    {'takeoff_reference': 'local_z'},
                ],
            ),
            Node(
                package='da_daka_control',
                executable='panel_mission',
                name='panel_mission',
                output='screen',
                parameters=[mission_config],
            ),
            Node(
                package='da_daka_control',
                executable='altitude_guard',
                name='altitude_guard',
                output='screen',
                parameters=[
                    guard_config,
                    {'maximum_climb_m': 2.5},
                ],
            ),
        ]
    )
