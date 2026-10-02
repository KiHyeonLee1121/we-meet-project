"""Launch the complete Pi-owned random-panel cleaning mission."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Build an inert-on-start full cleaning stack."""
    share = get_package_share_directory('da_daka_control')

    def config(name):
        return os.path.join(share, 'config', name)
    configuration_approved = LaunchConfiguration('configuration_approved')
    calibration_approved = LaunchConfiguration('calibration_approved')
    spray_output_enabled = LaunchConfiguration('spray_output_enabled')
    spray_backend = LaunchConfiguration('spray_backend')
    spray_reaction_enabled = LaunchConfiguration('spray_reaction_enabled')
    nozzle_forward_m = LaunchConfiguration('camera_to_nozzle_forward_m')
    nozzle_left_m = LaunchConfiguration('camera_to_nozzle_left_m')
    camera_height_above_lidar_m = LaunchConfiguration(
        'camera_height_above_lidar_m'
    )
    laptop_ip = LaunchConfiguration('laptop_ip')
    video_stream_enabled = LaunchConfiguration('video_stream_enabled')
    camera_shutter_us = LaunchConfiguration('camera_shutter_us')
    camera_gain = LaunchConfiguration('camera_gain')
    localization_test_mode = LaunchConfiguration('localization_test_mode')
    required_survey_panels = LaunchConfiguration('required_survey_panels')
    maximum_survey_panels = LaunchConfiguration('maximum_survey_panels')
    require_live_spray = LaunchConfiguration('require_live_spray')
    localization_position_tolerance_m = LaunchConfiguration(
        'localization_position_tolerance_m'
    )
    localization_scan_width_m = LaunchConfiguration('localization_scan_width_m')
    localization_scan_depth_m = LaunchConfiguration('localization_scan_depth_m')
    localization_scan_yaw_aligned = LaunchConfiguration(
        'localization_scan_yaw_aligned'
    )
    localization_scan_forward_offset_m = LaunchConfiguration(
        'localization_scan_forward_offset_m'
    )
    localization_scan_lateral_offset_m = LaunchConfiguration(
        'localization_scan_lateral_offset_m'
    )
    mission_node_name = LaunchConfiguration('mission_node_name')
    survey_timeout_s = LaunchConfiguration('survey_timeout_s')
    cruise_speed_mps = LaunchConfiguration('cruise_speed_mps')
    ignored_unhealthy_sensor_mask = LaunchConfiguration(
        'ignored_unhealthy_sensor_mask'
    )
    state_topic = LaunchConfiguration('state_topic')
    result_topic = LaunchConfiguration('result_topic')
    readiness_topic = LaunchConfiguration('readiness_topic')
    ai_mode_topic = LaunchConfiguration('ai_mode_topic')
    panel_id_topic = LaunchConfiguration('panel_id_topic')
    start_service = LaunchConfiguration('start_service')
    abort_service = LaunchConfiguration('abort_service')

    return LaunchDescription(
        [
            DeclareLaunchArgument('configuration_approved', default_value='false'),
            DeclareLaunchArgument('calibration_approved', default_value='false'),
            DeclareLaunchArgument('localization_test_mode', default_value='false'),
            DeclareLaunchArgument('required_survey_panels', default_value='0'),
            DeclareLaunchArgument('maximum_survey_panels', default_value='32'),
            DeclareLaunchArgument('require_live_spray', default_value='true'),
            DeclareLaunchArgument(
                'localization_position_tolerance_m', default_value='0.25'
            ),
            DeclareLaunchArgument('localization_scan_width_m', default_value='3.0'),
            DeclareLaunchArgument('localization_scan_depth_m', default_value='2.0'),
            DeclareLaunchArgument(
                'localization_scan_yaw_aligned', default_value='false'
            ),
            DeclareLaunchArgument(
                'localization_scan_forward_offset_m', default_value='0.0'
            ),
            DeclareLaunchArgument(
                'localization_scan_lateral_offset_m', default_value='0.0'
            ),
            DeclareLaunchArgument(
                'mission_node_name', default_value='autonomous_cleaning_mission'
            ),
            DeclareLaunchArgument(
                'survey_timeout_s', default_value='55.0'
            ),
            DeclareLaunchArgument(
                'cruise_speed_mps', default_value='0.65'
            ),
            DeclareLaunchArgument(
                'ignored_unhealthy_sensor_mask',
                default_value='81920',
                description=(
                    'Permanent field mask 0x14000, including absent RC '
                    'receiver bit 0x10000'
                ),
            ),
            DeclareLaunchArgument(
                'state_topic', default_value='/autonomous_cleaning/state'
            ),
            DeclareLaunchArgument(
                'result_topic', default_value='/autonomous_cleaning/result'
            ),
            DeclareLaunchArgument(
                'readiness_topic', default_value='/autonomous_cleaning/readiness'
            ),
            DeclareLaunchArgument(
                'ai_mode_topic', default_value='/ai/requested_mode'
            ),
            DeclareLaunchArgument(
                'panel_id_topic',
                default_value='/autonomous_cleaning/current_panel_id',
            ),
            DeclareLaunchArgument(
                'start_service', default_value='/autonomous_cleaning/start'
            ),
            DeclareLaunchArgument(
                'abort_service', default_value='/autonomous_cleaning/abort'
            ),
            DeclareLaunchArgument('spray_output_enabled', default_value='false'),
            DeclareLaunchArgument('spray_backend', default_value='mock'),
            DeclareLaunchArgument(
                'spray_reaction_enabled', default_value='false'
            ),
            DeclareLaunchArgument(
                'camera_to_nozzle_forward_m', default_value='-0.07'
            ),
            DeclareLaunchArgument(
                'camera_to_nozzle_left_m', default_value='-0.05'
            ),
            DeclareLaunchArgument(
                'camera_height_above_lidar_m', default_value='-0.16'
            ),
            DeclareLaunchArgument('laptop_ip', default_value='127.0.0.1'),
            DeclareLaunchArgument('video_stream_enabled', default_value='false'),
            DeclareLaunchArgument('camera_shutter_us', default_value='50000'),
            DeclareLaunchArgument('camera_gain', default_value='32.0'),
            Node(
                package='da_daka_control',
                executable='tf_luna_serial',
                name='tf_luna_serial',
                output='screen',
                parameters=[config('tf_luna_serial.yaml')],
            ),
            Node(
                package='da_daka_control',
                executable='distance_filter',
                name='distance_filter',
                output='screen',
                parameters=[config('distance_filter.yaml')],
            ),
            Node(
                package='da_daka_control',
                executable='distance_controller',
                name='distance_controller',
                output='screen',
                parameters=[
                    config('distance_controller.yaml'),
                    {
                        'command_topic': '/distance_control/cmd_vel_internal',
                        'takeoff_reference': 'lidar',
                        'lidar_takeoff_target_distance_m': 3.0,
                        'local_takeoff_tolerance_m': 0.30,
                        # Match the flight controller profile measured from
                        # the stable QGC/PX4 takeoff on 2026-08-22.
                        'local_takeoff_kp': 1.0,
                        'local_takeoff_max_speed_mps': 1.50,
                        'local_takeoff_slow_zone_m': 0.50,
                        'local_takeoff_max_accel_mps2': 4.0,
                        'local_takeoff_stable_duration_s': 1.0,
                        'target_distance_m': 1.0,
                        'kp': 0.70,
                        'ki': 0.0,
                        'kd': 0.15,
                        'target_stable_duration_s': 2.9,
                        'target_stable_required_ratio': 0.90,
                        'target_stable_require_local_velocity': True,
                        'target_stable_max_vehicle_speed_mps': 0.05,
                        'hold_yaw_enabled': True,
                        'spray_ff_enabled': ParameterValue(
                            spray_reaction_enabled, value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package='da_daka_control',
                executable='perception_receiver',
                name='perception_receiver',
                output='screen',
                parameters=[
                    config('perception_receiver.yaml'),
                    {'allowed_remote_ip': laptop_ip},
                ],
            ),
            Node(
                package='da_daka_control',
                executable='spray_reaction_compensator',
                name='spray_reaction_compensator',
                output='screen',
                parameters=[
                    config('spray_reaction_compensator_integrated.yaml'),
                    {
                        'output_enabled': ParameterValue(
                            spray_reaction_enabled, value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package='da_daka_control',
                executable='video_streamer',
                name='video_streamer',
                output='screen',
                parameters=[
                    config('video_streamer.yaml'),
                    {
                        'laptop_ip': laptop_ip,
                        'enabled_on_startup': ParameterValue(
                            video_stream_enabled, value_type=bool
                        ),
                        'shutter_us': ParameterValue(
                            camera_shutter_us, value_type=int
                        ),
                        'gain': ParameterValue(camera_gain, value_type=float),
                    },
                ],
            ),
            Node(
                package='da_daka_control',
                executable='perception_control_sender',
                name='perception_control_sender',
                output='screen',
                parameters=[{'laptop_ip': laptop_ip}],
            ),
            Node(
                package='da_daka_control',
                executable='panel_survey',
                name='panel_survey',
                output='screen',
                parameters=[config('panel_survey.yaml')],
            ),
            Node(
                package='da_daka_control',
                executable='nozzle_visual_servo',
                name='nozzle_visual_servo',
                output='screen',
                parameters=[
                    config('nozzle_visual_servo.yaml'),
                    {
                        'camera_to_nozzle_forward_m': ParameterValue(
                            nozzle_forward_m, value_type=float
                        ),
                        'camera_to_nozzle_left_m': ParameterValue(
                            nozzle_left_m, value_type=float
                        ),
                        'camera_height_above_lidar_m': ParameterValue(
                            camera_height_above_lidar_m, value_type=float
                        ),
                    },
                ],
            ),
            Node(
                package='da_daka_control',
                executable='spray_controller',
                name='spray_controller',
                output='screen',
                parameters=[
                    config('spray_controller.yaml'),
                    {
                        'backend': spray_backend,
                        'output_enabled': ParameterValue(
                            spray_output_enabled, value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package='da_daka_control',
                executable='autonomous_cleaning_mission',
                output='screen',
                parameters=[
                    config('autonomous_cleaning.yaml'),
                    {
                        'configuration_approved': ParameterValue(
                            configuration_approved, value_type=bool
                        ),
                        'calibration_approved': ParameterValue(
                            calibration_approved, value_type=bool
                        ),
                        'localization_test_mode': ParameterValue(
                            localization_test_mode, value_type=bool
                        ),
                        'required_survey_panels': ParameterValue(
                            required_survey_panels, value_type=int
                        ),
                        'maximum_survey_panels': ParameterValue(
                            maximum_survey_panels, value_type=int
                        ),
                        'require_live_spray': ParameterValue(
                            require_live_spray, value_type=bool
                        ),
                        'localization_position_tolerance_m': ParameterValue(
                            localization_position_tolerance_m,
                            value_type=float,
                        ),
                        'localization_scan_width_m': ParameterValue(
                            localization_scan_width_m, value_type=float
                        ),
                        'localization_scan_depth_m': ParameterValue(
                            localization_scan_depth_m, value_type=float
                        ),
                        'localization_scan_yaw_aligned': ParameterValue(
                            localization_scan_yaw_aligned,
                            value_type=bool,
                        ),
                        'localization_scan_forward_offset_m': (
                            ParameterValue(
                                localization_scan_forward_offset_m,
                                value_type=float,
                            )
                        ),
                        'localization_scan_lateral_offset_m': (
                            ParameterValue(
                                localization_scan_lateral_offset_m,
                                value_type=float,
                            )
                        ),
                        # Keep the localization-test node override independent
                        # of the node name used by the YAML parameter section.
                        'survey_timeout_s': ParameterValue(
                            survey_timeout_s, value_type=float
                        ),
                        'cruise_speed_mps': ParameterValue(
                            cruise_speed_mps, value_type=float
                        ),
                        # Pass the permanent hardware-configuration exception
                        # directly so alternate mission node names cannot lose
                        # the YAML-scoped setting.
                        'ignored_unhealthy_sensor_mask': ParameterValue(
                            ignored_unhealthy_sensor_mask, value_type=int
                        ),
                        'state_topic': state_topic,
                        'result_topic': result_topic,
                        'readiness_topic': readiness_topic,
                        'ai_mode_topic': ai_mode_topic,
                        'panel_id_topic': panel_id_topic,
                        'start_service': start_service,
                        'abort_service': abort_service,
                    },
                ],
                name=mission_node_name,
            ),
            Node(
                package='da_daka_control',
                executable='altitude_guard',
                name='altitude_guard',
                output='screen',
                parameters=[
                    config('altitude_guard.yaml'),
                    {'maximum_climb_m': 4.0},
                ],
            ),
        ]
    )
