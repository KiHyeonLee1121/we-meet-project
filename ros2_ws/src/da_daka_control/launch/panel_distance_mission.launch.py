"""Launch the route + per-panel distance-hold mission and safety nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Build an inert-on-start panel distance mission launch description."""
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
        'panel_distance_mission.yaml',
    )
    guard_config = os.path.join(
        package_share,
        'config',
        'altitude_guard.yaml',
    )
    configuration_approved = LaunchConfiguration('configuration_approved')
    takeoff_height_m = LaunchConfiguration('takeoff_height_m')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'configuration_approved',
                default_value='false',
                description=(
                    'Unlock mission start only after route and clearance approval'
                ),
            ),
            DeclareLaunchArgument(
                'takeoff_height_m',
                default_value='3.0',
                description='Shared LiDAR takeoff and route height in meters',
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
                parameters=[
                    controller_config,
                    {
                        'takeoff_reference': 'lidar',
                        'lidar_takeoff_target_distance_m': ParameterValue(
                            takeoff_height_m,
                            value_type=float,
                        ),
                        # The earlier 2 m flight test showed sustained
                        # overshoot with
                        # Match the flight controller profile measured from
                        # the stable QGC/PX4 takeoff on 2026-08-22:
                        # MPC_Z_P=1, MPC_TKO_SPEED=1.5, MPC_ACC_UP_MAX=4.
                        'local_takeoff_tolerance_m': 0.30,
                        'local_takeoff_kp': 1.0,
                        'local_takeoff_max_speed_mps': 1.50,
                        'local_takeoff_slow_zone_m': 0.50,
                        'local_takeoff_max_accel_mps2': 4.0,
                        # 2026-08-10/11 night session (log_84-86): TF-Luna
                        # sits pinned near its min range for ~13-14s of
                        # ground effect before real climb starts, then
                        # settles within +-0.2-0.3 m of target, not the
                        # tight 0.08-0.10 m this hold check demanded -- so
                        # TAKEOFF_HOLD timed out twice without ever latching
                        # target_reached. This is a coarse "cleared the
                        # ground" check, not the final approach, so it does
                        # not need target_stable_tolerance_m-grade precision.
                        'local_takeoff_stable_duration_s': 1.0,
                        # Hold the pre-arm nose direction during both the
                        # LiDAR takeoff and each 1 m distance-control phase.
                        # 0.35 rad/s is a 20 deg/s correction limit.
                        'hold_yaw_enabled': True,
                        'yaw_target_timeout_s': 0.3,
                        'yaw_hold_kp': 1.0,
                        'yaw_hold_max_rate_rad_s': 0.35,
                        # Panel-only 1 m approach profile. The PID keeps the
                        # verified zero integral and filtered D damping, while
                        # the modest P increase shortens the near-target
                        # approach. Stability retains the strict +/-0.10 m and
                        # <=0.05 m/s limits, but tolerates up to 10 percent
                        # isolated LiDAR-rate noise over a 2.9 s window and
                        # also requires fresh, slow MAVROS vertical velocity.
                        'kp': 0.70,
                        'ki': 0.0,
                        'kd': 0.15,
                        'target_stable_duration_s': 2.9,
                        'target_stable_required_ratio': 0.90,
                        'target_stable_require_local_velocity': True,
                        'target_stable_max_vehicle_speed_mps': 0.05,
                    },
                ],
            ),
            Node(
                package='da_daka_control',
                executable='panel_distance_mission',
                name='panel_distance_mission',
                output='screen',
                parameters=[
                    mission_config,
                    {
                        'configuration_approved': ParameterValue(
                            configuration_approved,
                            value_type=bool,
                        ),
                        'takeoff_height_m': ParameterValue(
                            takeoff_height_m,
                            value_type=float,
                        ),
                    },
                ],
            ),
            Node(
                package='da_daka_control',
                executable='altitude_guard',
                name='altitude_guard',
                output='screen',
                parameters=[
                    guard_config,
                    # Keep a 1 m emergency margin above the 3 m patrol height.
                    {'maximum_climb_m': 4.0},
                ],
            ),
        ]
    )
