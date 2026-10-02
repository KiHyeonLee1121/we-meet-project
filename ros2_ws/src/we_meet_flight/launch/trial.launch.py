from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='panel_approach.yaml'),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('flight_settings_verified', default_value='false'),
        DeclareLaunchArgument('lidar_geometry_verified', default_value='false'),
        DeclareLaunchArgument('camera_axes_verified', default_value='false'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('estimator_topic', default_value='/mavros/estimator_status'),
        DeclareLaunchArgument('lidar_topic', default_value='/distance/filtered'),
        DeclareLaunchArgument('fc_mavlink_topic', default_value='/mavros/mavlink_source'),
        DeclareLaunchArgument('fc_system_id', default_value='1'),
        DeclareLaunchArgument('fc_component_id', default_value='1'),
        DeclareLaunchArgument('mavros_namespace', default_value='/mavros'),
        DeclareLaunchArgument('log_directory', default_value='~/flight_logs/test_flying_v2'),
        Node(package='we_meet_flight', executable='flight_trial', output='screen', parameters=[{
            'config_file': PathJoinSubstitution([FindPackageShare('we_meet_flight'), 'config', LaunchConfiguration('profile')]),
            'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
            'flight_settings_verified': ParameterValue(LaunchConfiguration('flight_settings_verified'), value_type=bool),
            'lidar_geometry_verified': ParameterValue(LaunchConfiguration('lidar_geometry_verified'), value_type=bool),
            'camera_axes_verified': ParameterValue(LaunchConfiguration('camera_axes_verified'), value_type=bool),
            'image_topic': LaunchConfiguration('image_topic'), 'lidar_topic': LaunchConfiguration('lidar_topic'),
            'mavros_namespace': LaunchConfiguration('mavros_namespace'),
            'estimator_topic': LaunchConfiguration('estimator_topic'),
            'fc_mavlink_topic': LaunchConfiguration('fc_mavlink_topic'),
            'fc_system_id': ParameterValue(LaunchConfiguration('fc_system_id'), value_type=int),
            'fc_component_id': ParameterValue(LaunchConfiguration('fc_component_id'), value_type=int),
            'log_directory': LaunchConfiguration('log_directory')}])])
