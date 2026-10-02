"""Launch the no-spray two-panel localization test profile."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Build a 3 m survey, 1 m localization and return-to-land profile."""
    share = Path(get_package_share_directory('da_daka_control'))
    parent = share / 'launch' / 'autonomous_cleaning.launch.py'

    arguments = (
        'configuration_approved',
        'calibration_approved',
        'laptop_ip',
        'video_stream_enabled',
        'camera_shutter_us',
        'camera_gain',
        'required_survey_panels',
        'maximum_survey_panels',
    )
    declarations = [
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
        # Was pinned to 2, which aborted the whole run whenever the survey
        # resolved only one panel - the 2026-08-24 night flights all ended
        # there with a panel already mapped and never exercised the 1 m
        # localization leg. 0 disables the exact-count check, so any survey
        # outcome from 1 up to maximum_survey_panels proceeds to approach
        # each panel it did find. Set this back to 2 for the strict
        # two-panel acceptance test.
        # Was pinned to 2, which aborted the run whenever the survey resolved
        # a third cluster - flight 6 on 2026-08-24 died on "3 > 2" with three
        # solid targets (29/61/46 observations, 0.78-0.88 confidence). The
        # count is no longer a reason to abort: any survey with at least one
        # panel proceeds and the route planner visits them nearest-first.
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
        # The world-aligned rectangle spent two of its four legs behind
        # the aircraft on 2026-08-23. The panels sit ahead, so align the
        # scan to the launch heading and push its centre forward.
        'localization_scan_yaw_aligned': 'true',
        'localization_scan_forward_offset_m': '1.0',
        'localization_scan_lateral_offset_m': '0.0',
        # ULog 116/117: 0.65 m/s produced 0.67-0.75 m p95 moving
        # tracking lag. 0.50 m/s adds about 5.4 s to the 11.6 m route.
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
