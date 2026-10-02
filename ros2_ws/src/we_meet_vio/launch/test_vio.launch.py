"""No FC writes by default. A flight additionally requires /vio_flight/start."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    def enabled(name):
        return value(name).lower()=="true"
    directory=Path(value("config_dir")).expanduser().resolve()
    from we_meet_vio.settings import load,calibration
    config=load(directory/"settings.yaml")
    calibration(directory/"calibration.json")
    estimator=directory/"openvins"/"estimator_config.yaml"
    if enabled("openvins") and not estimator.is_file():
        raise RuntimeError("Run tools/configure_calibration.py with measured Kalibr inputs")
    parameters={"settings_path":str(directory/"settings.yaml"),
                "calibration_path":str(directory/"calibration.json"),"dry_run":enabled("dry_run")}
    actions=[]
    if enabled("start_mavros"):
        share=Path(get_package_share_directory("mavros"))
        actions.append(Node(package="mavros",executable="mavros_node",
                            namespace=config["mavros_namespace"].strip("/"),output="screen",
                            parameters=[str(share/"launch"/"px4_pluginlists.yaml"),
                                        str(share/"launch"/"px4_config.yaml"),
                                        str(directory/"mavros_overrides.yaml"),
                                        {"fcu_url":value("fcu_url"),"gcs_url":value("gcs_url"),
                                         "fcu_protocol":"v2.0","tgt_system":int(value("tgt_system")),
                                         "tgt_component":int(value("tgt_component"))}]))
    if enabled("camera"):
        actions.append(Node(package="we_meet_vio",executable="pi_camera",parameters=[parameters],output="screen"))
    if enabled("lidar"):
        actions.append(Node(package="we_meet_vio",executable="tf_luna",parameters=[parameters],output="screen"))
    if enabled("openvins"):
        actions.append(Node(package="we_meet_vio",executable="imu_gate",parameters=[parameters],output="screen"))
        actions.append(Node(package="ov_msckf",executable="run_subscribe_msckf",namespace="ov_msckf",
                            parameters=[{"config_path":str(estimator),"use_stereo":False,"max_cameras":1,
                                         "save_total_state":False,"verbosity":"WARNING"}],output="screen"))
    bridge=dict(parameters)
    bridge["dry_run"]=enabled("dry_run") and not enabled("feed_fc_vision")
    actions += [Node(package="we_meet_vio",executable="vio_bridge",parameters=[bridge],output="screen"),
                Node(package="we_meet_vio",executable="panel_detector",parameters=[parameters],output="screen"),
                Node(package="we_meet_vio",executable="vio_flight",parameters=[parameters],output="screen")]
    return actions


def generate_launch_description():
    arguments={"config_dir":"","dry_run":"true","feed_fc_vision":"false",
               "camera":"true","lidar":"true","openvins":"true","start_mavros":"false",
               "fcu_url":"serial:///dev/ttyACM0:921600","gcs_url":"",
               "tgt_system":"1","tgt_component":"1"}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in arguments.items()]+[OpaqueFunction(function=setup)])
