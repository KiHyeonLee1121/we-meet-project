"""Launch the reconstructed, no-spray flight 141 profile."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Build a 3 m survey, 1 m localization and return-to-land profile."""
    share = Path(get_package_share_directory('we_meet_prototype141'))
    parent = share / 'launch' / '_stack141.launch.py'

    arguments = (
        'configuration_approved',
        'calibration_approved',
        'laptop_ip',
        'video_stream_enabled',
        'camera_shutter_us',
        'camera_gain',
        'required_survey_panels',
        'maximum_survey_panels',
        'lidar_port',
        'prototype_profile',
        'log_directory',
    )
    declarations = [
        DeclareLaunchArgument('lidar_port', default_value='/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0'),
        DeclareLaunchArgument('prototype_profile', default_value=str(share / 'config' / 'flight141.yaml')),
        DeclareLaunchArgument('log_directory', default_value='~/flight141-logs'),
        DeclareLaunchArgument(
            'configuration_approved',
            default_value='false',
            description='Explicit approval for the two-panel localization area',
        ),
        DeclareLaunchArgument(
            'calibration_approved',
            default_value='false',
            description='Explicit approval for calibrated camera and LiDAR offsets',
        ),
        DeclareLaunchArgument('laptop_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('video_stream_enabled', default_value='false'),
        DeclareLaunchArgument('camera_shutter_us', default_value='50000'),
        DeclareLaunchArgument('camera_gain', default_value='32.0'),
        # Flight 141 accepted four clusters from two physical panels.
        DeclareLaunchArgument(
            'maximum_survey_panels',
            default_value='8',
            description='Upper bound on panels the survey may map',
        ),
        DeclareLaunchArgument(
            'required_survey_panels',
            default_value='0',
            description=(
                'Exact panel count the survey must produce, or 0 to accept '
                'any count up to maximum_survey_panels'
            ),
        ),
    ]
    overrides = {
        name: LaunchConfiguration(name) for name in arguments
    }
    overrides.update({
        'localization_test_mode': 'true',
        'require_live_spray': 'false',
        'localization_scan_width_m': '3.0',
        'localization_scan_depth_m': '2.0',
        # Raw ULog 141: rectangle centred at the Home-follow launch,
        # yaw aligned, without the snapshot's +1 m forward offset.
        'localization_scan_yaw_aligned': 'true',
        'localization_scan_forward_offset_m': '0.0',
        'localization_scan_lateral_offset_m': '0.0',
        # Keep the attached 141 test-profile speed, not a v1/v2 approach.
        'cruise_speed_mps': '0.50',
        'survey_timeout_s': '65.0',
        'spray_output_enabled': 'false',
        'spray_backend': 'mock',
        'spray_reaction_enabled': 'false',
        'mission_node_name': 'panel_localization_test',
        'state_topic': '/panel_localization_test/state',
        'result_topic': '/panel_localization_test/result',
        'readiness_topic': '/panel_localization_test/readiness',
        'panel_id_topic': '/panel_localization_test/current_panel_id',
        'start_service': '/panel_localization_test/start',
        'abort_service': '/panel_localization_test/abort',
    })
    return LaunchDescription(
        declarations + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(parent)),
                launch_arguments=(
                    (name, value) for name, value in overrides.items()
                ),
            )
        ]
    )
