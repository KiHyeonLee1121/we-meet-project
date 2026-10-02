"""Launch filtering, distance control, and the mission manager."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Build the complete distance-mission launch description."""
    package_share = get_package_share_directory('da_daka_control')
    filter_config = os.path.join(
        package_share,
        'config',
        'distance_filter.yaml',
    )
    controller_config = os.path.join(
        package_share,
        'config',
        'distance_controller.yaml',
    )
    mission_config = os.path.join(
        package_share,
        'config',
        'distance_mission.yaml',
    )
    altitude_guard_config = os.path.join(
        package_share,
        'config',
        'altitude_guard.yaml',
    )
    validation_approved = LaunchConfiguration('validation_approved')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'validation_approved', default_value='false'
            ),
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
                # This test mission uses TF-Luna for takeoff height, target
                # arrival, and distance hold. Panel missions keep Local Z.
                parameters=[
                    controller_config,
                    {'takeoff_reference': 'lidar'},
                ],
            ),
            Node(
                package='da_daka_control',
                executable='mission_manager',
                name='mission_manager',
                output='screen',
                parameters=[
                    mission_config,
                    {
                        'validation_approved': ParameterValue(
                            validation_approved, value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package='da_daka_control',
                executable='altitude_guard',
                name='altitude_guard',
                output='screen',
                parameters=[altitude_guard_config],
            ),
        ]
    )
