from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    defaults = {'profile': 'velocity_trial.yaml', 'dry_run': 'true',
                'flight_settings_verified': 'false', 'lidar_geometry_verified': 'false',
                'lidar_topic': '/distance/filtered', 'estimator_topic': '/mavros/estimator_status',
                'mavros_namespace': '/mavros', 'log_directory': '~/flight_logs/test_flying_v1'}
    booleans = {'dry_run', 'flight_settings_verified', 'lidar_geometry_verified'}
    parameters = {key: ParameterValue(LaunchConfiguration(key), value_type=bool) if key in booleans
                  else LaunchConfiguration(key) for key in defaults if key != 'profile'}
    parameters['config_file'] = PathJoinSubstitution([FindPackageShare('we_meet_flight_v1'), 'config', LaunchConfiguration('profile')])
    return LaunchDescription([*[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
                              Node(package='we_meet_flight_v1', executable='flight_trial', output='screen', parameters=[parameters])])
