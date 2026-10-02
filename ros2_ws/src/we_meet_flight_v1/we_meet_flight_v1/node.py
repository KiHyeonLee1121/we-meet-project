"""ROS 2 / MAVROS adapter. FC writes are disabled unless dry_run=false + start."""
from collections import deque
from dataclasses import asdict
import math
from pathlib import Path
import time
import traceback
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState, Imu, NavSatFix, Range
from mavros_msgs.msg import EstimatorStatus, ExtendedState, State
from mavros_msgs.srv import CommandBool, SetMode
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .config import load_config
from .control import quaternion_yaw, wrapped_angle_error, yaw_reset_innovation
from .journal import Journal
from .mission import Mission, Sensors


class FlightNode(Node):
    def __init__(self):
        super().__init__('test_flying_v1')
        default = str(Path(get_package_share_directory('we_meet_flight_v1')) / 'config/velocity_trial.yaml')
        params = {'config_file': default, 'dry_run': True, 'flight_settings_verified': False,
                  'lidar_geometry_verified': False, 'log_directory': '~/flight_logs/test_flying_v1',
                  'mavros_namespace': '/mavros', 'lidar_topic': '/distance/filtered',
                  'estimator_topic': '/mavros/estimator_status'}
        for name, value in params.items():
            self.declare_parameter(name, value)
        self.c = load_config(self.get_parameter('config_file').value)
        # Flight permission is captured at startup; it cannot be changed mid-run.
        self.dry = self.get_parameter('dry_run').value
        self.settings_verified = self.get_parameter('flight_settings_verified').value
        self.lidar_verified = self.get_parameter('lidar_geometry_verified').value
        self.roll = self.pitch = math.nan
        self.lidar_capable = False
        self.ns = self.get_parameter('mavros_namespace').value.rstrip('/')
        self.mission = Mission(self.c)
        self.s = Sensors()
        self.receipts, self.source_stamps = {}, {}
        self.yaw_samples = deque()
        self.range_samples = deque()
        self.previous_pose = None
        self.rates = (math.nan, math.nan, math.nan)
        self.telemetry = {}
        self.last_graph_check = 0.0
        self.last_tick_s = time.monotonic()
        self.frame_verified = False
        self.frame_future = None
        self.frame_checked_s = -math.inf
        self.graph_fault = ''
        self.pending_services = []
        def git_read(arguments):
            try:
                return subprocess.check_output(['git', *arguments], stderr=subprocess.DEVNULL,
                                               text=True, timeout=1).strip()
            except Exception:
                return None
        self.journal = Journal(self.get_parameter('log_directory').value, {
            'code_commit': git_read(['rev-parse', 'HEAD']),
            'code_worktree_changes': git_read(['status', '--porcelain']),
            'config': asdict(self.c), 'dry_run': self.dry,
            'flight_settings_verified': self.settings_verified,
            'lidar_geometry_verified': self.lidar_verified,
            'unix_time_s': time.time(), 'monotonic_s': time.monotonic(),
            'distance_semantics': 'integral of published forward command, NOT measured travel',
            'software': 'we_meet_flight_v1 0.1.0', 'mavros_namespace': self.ns})
        self.command_topic = self.ns + '/setpoint_velocity/cmd_vel'
        self.output_topic = '/test_flying_v1/debug/cmd_vel' if self.dry else self.command_topic
        self.publisher = self.create_publisher(TwistStamped, self.output_topic, qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '/test_flying_v1/status', 10)
        qos = qos_profile_sensor_data
        estimator_topic = self.get_parameter('estimator_topic').value
        if estimator_topic == '/mavros/estimator_status':
            estimator_topic = self.ns+'/estimator_status'
        self.create_subscription(EstimatorStatus, estimator_topic, self.on_estimator, qos)
        self.create_subscription(State, self.ns+'/state', self.on_state, qos)
        self.create_subscription(ExtendedState, self.ns+'/extended_state', self.on_landed, qos)
        self.create_subscription(PoseStamped, self.ns+'/local_position/pose', self.on_pose, qos)
        self.create_subscription(Imu, self.ns+'/imu/data', self.on_imu, qos)
        self.create_subscription(NavSatFix, self.ns+'/global_position/raw/fix', self.on_gps, qos)
        self.create_subscription(BatteryState, self.ns+'/battery', self.on_battery, qos)
        self.create_subscription(TwistStamped, self.ns+'/local_position/velocity_local', self.on_velocity, qos)
        self.create_subscription(Range, self.get_parameter('lidar_topic').value, self.on_range, qos)
        self.mode_client = self.create_client(SetMode, self.ns+'/set_mode')
        self.arm_client = self.create_client(CommandBool, self.ns+'/cmd/arming')
        self.frame_client = self.create_client(GetParameters, self.ns+'/setpoint_velocity/get_parameters')
        self.create_service(Trigger, '/test_flying_v1/start', self.on_start)
        self.create_service(Trigger, '/test_flying_v1/abort', self.on_abort)
        self.timer = self.create_timer(1.0/self.c.control_hz, self.tick)
        self.get_logger().info(f'dry_run={self.dry}; commands={self.output_topic}; log={self.journal.path}')

    def receipt(self, key, msg, maximum_age=None):
        now = time.monotonic()
        if hasattr(msg, 'header'):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            ros_now = self.get_clock().now().nanoseconds*1e-9
            limit = maximum_age or self.c.sensor_timeout_s
            previous = self.source_stamps.get(key, -math.inf)
            if stamp <= 0 or stamp <= previous or not -0.05 <= ros_now-stamp <= limit:
                return None
            self.source_stamps[key] = stamp
            # Age includes transport delay, not just time since callback arrival.
            self.receipts[key] = now-max(0.0, ros_now-stamp)
        else:
            self.receipts[key] = now
        return now

    def on_estimator(self, msg):
        if self.receipt('estimator', msg, self.c.state_timeout_s) is not None:
            self.s.estimator_valid = (msg.attitude_status_flag and msg.velocity_horiz_status_flag
                                      and msg.velocity_vert_status_flag and not msg.accel_error_status_flag
                                      and not msg.const_pos_mode_status_flag)
            self.telemetry['estimator_status'] = {name: getattr(msg, name) for name in (
                'attitude_status_flag', 'velocity_horiz_status_flag', 'velocity_vert_status_flag',
                'pos_horiz_rel_status_flag', 'pos_horiz_abs_status_flag', 'gps_glitch_status_flag')}

    def on_state(self, msg):
        if self.receipt('state', msg, self.c.state_timeout_s) is None:
            return
        self.s.connected, self.s.armed, self.s.mode = msg.connected, msg.armed, msg.mode
        self.s.failsafe_observed = msg.system_status >= 5
        self.telemetry['system_status'] = msg.system_status
        self.telemetry['failsafe_semantics'] = 'MAV_STATE critical/emergency indication, not complete PX4 failsafe flags'

    def on_landed(self, msg):
        if self.receipt('landed', msg, self.c.landed_timeout_s) is None:
            return
        self.s.landed = msg.landed_state == ExtendedState.LANDED_STATE_ON_GROUND

    def on_pose(self, msg):
        now = self.receipt('yaw', msg)
        if now is None:
            return
        try:
            q = msg.pose.orientation
            yaw = quaternion_yaw(q.x, q.y, q.z, q.w)
            self.roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
            self.pitch = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y-q.z*q.x))))
            stamp = self.source_stamps['yaw']
            if self.previous_pose:
                old_stamp, old_yaw = self.previous_pose
                dt = stamp-old_stamp
                # Euler yaw derivative from body FLU rates, accounting for roll/pitch.
                roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
                pitch = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y-q.z*q.x))))
                if (0 < dt <= self.c.maximum_tick_gap_s and abs(math.cos(pitch)) > 0.5
                        and now-self.receipts.get('imu', -math.inf) <= self.c.sensor_timeout_s
                        and all(math.isfinite(v) for v in self.rates)):
                    rate = (self.rates[1]*math.sin(roll)+self.rates[2]*math.cos(roll))/math.cos(pitch)
                    innovation = yaw_reset_innovation(old_yaw, yaw, rate, dt)
                    if abs(innovation) >= self.c.yaw_reset_min_rad and self.mission.state not in Mission.TERMINAL:
                        self.s.yaw_reset = True  # Latch. Do not shift yaw_ref and continue.
                        self.journal.append({'kind': 'yaw_reset', 'time_s': now, 'innovation_rad': innovation})
            self.previous_pose = stamp, yaw
            self.s.yaw = yaw
            self.yaw_samples.append((now, yaw))
            while self.yaw_samples and now-self.yaw_samples[0][0] > self.c.yaw_stable_s+0.15:
                self.yaw_samples.popleft()
            p = msg.pose.position
            self.telemetry['position_enu_log_only'] = [p.x, p.y, p.z]
        except ValueError:
            self.s.yaw = math.nan

    def on_imu(self, msg):
        if self.receipt('imu', msg) is not None:
            self.rates = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
            if not all(math.isfinite(v) for v in self.rates):
                self.receipts.pop('imu', None)

    def on_velocity(self, msg):
        if self.receipt('velocity', msg) is not None:
            self.telemetry['velocity_enu_log_only'] = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]

    def on_gps(self, msg):
        if self.receipt('gps', msg, 2.0) is not None:
            self.telemetry['gps_log_only'] = {
                'status': msg.status.status, 'service': msg.status.service,
                'latitude': msg.latitude, 'longitude': msg.longitude, 'altitude': msg.altitude,
                'covariance': list(msg.position_covariance)}

    def on_battery(self, msg):
        if self.receipt('battery', msg, self.c.battery_timeout_s) is not None:
            self.s.battery_remaining = msg.percentage
            self.telemetry['battery_voltage'] = msg.voltage

    def on_range(self, msg):
        now = self.receipt('range', msg)
        if now is None:
            return
        value = msg.range
        if not math.isfinite(value) or not msg.min_range <= value <= msg.max_range:
            self.s.height_m = math.nan
            self.range_samples.clear()
            return
        self.telemetry['lidar_raw_m'] = value
        self.lidar_capable = math.isfinite(msg.max_range) and msg.max_range >= self.c.target_height_m
        if (not self.lidar_capable or not math.isfinite(self.roll) or not math.isfinite(self.pitch)
                or max(abs(self.roll), abs(self.pitch)) > self.c.maximum_tilt_rad):
            self.s.height_m = math.nan
            self.range_samples.clear()
            return
        if not self.c.lidar_input_is_vertical_height:
            # Body-aligned downward beam, flat surface; offset positive below body origin.
            value = (value+self.c.lidar_body_down_offset_m)*math.cos(self.roll)*math.cos(self.pitch)
        self.s.height_m = value
        if self.range_samples and now-self.range_samples[-1][0] > self.c.sensor_timeout_s:
            self.range_samples.clear()
        self.range_samples.append((now, value))
        while len(self.range_samples) > 2 and now-self.range_samples[1][0] >= self.c.lidar_rate_window_s:
            self.range_samples.popleft()
        first = self.range_samples[0]
        self.s.height_rate_mps = ((value-first[1])/(now-first[0])
                                  if now-first[0] >= 0.9*self.c.lidar_rate_window_s else math.nan)

    def snapshot(self, now):
        for key, field in [('estimator', 'estimator_age'), ('state', 'state_age'), ('landed', 'landed_age'), ('yaw', 'yaw_age'),
                           ('imu', 'imu_age'), ('range', 'range_age'), ('battery', 'battery_age')]:
            setattr(self.s, field, now-self.receipts.get(key, -math.inf))
        return self.s

    def stable_yaw(self, now):
        if not self.yaw_samples or now-self.yaw_samples[-1][0] > self.c.sensor_timeout_s:
            raise ValueError('fresh yaw samples unavailable')
        if self.yaw_samples[-1][0]-self.yaw_samples[0][0] < self.c.yaw_stable_s:
            raise ValueError('wait for a full stable yaw window')
        if any(b[0]-a[0] > self.c.sensor_timeout_s for a, b in zip(self.yaw_samples, list(self.yaw_samples)[1:])):
            raise ValueError('yaw samples have a gap in the stability window')
        mean = math.atan2(sum(math.sin(y) for _, y in self.yaw_samples), sum(math.cos(y) for _, y in self.yaw_samples))
        if max(abs(wrapped_angle_error(mean, y)) for _, y in self.yaw_samples) > self.c.yaw_stable_deviation_rad:
            raise ValueError('yaw is not stable')
        return mean

    def on_start(self, request, response):
        now = time.monotonic()
        try:
            if not self.dry:
                if not self.lidar_verified or not self.lidar_capable:
                    raise ValueError('verify downward lidar geometry and 3m measurement capability')
                if not self.settings_verified:
                    raise ValueError('live flight requires verified flight settings')
                if self.get_parameter('use_sim_time').value:
                    raise ValueError('live trial requires real clock, not use_sim_time')
                if not self.frame_verified:
                    raise ValueError('MAVROS setpoint_velocity.mav_frame must be verified LOCAL_NED')
                if self.graph_fault:
                    raise ValueError(self.graph_fault)
                if not self.mode_client.service_is_ready() or not self.arm_client.service_is_ready():
                    raise ValueError('MAVROS mode/arming services unavailable')
            self.mission.start(now, self.snapshot(now), self.stable_yaw(now))
            response.success = True
            response.message = 'started; DRY RUN only' if self.dry else 'started live flight trial'
            self.journal.append({'kind': 'start', 'time_s': now, 'yaw_ref': self.mission.yaw_ref})
        except ValueError as exc:
            response.success, response.message = False, str(exc)
            self.journal.append({'kind': 'start_rejected', 'time_s': now, 'reason': str(exc)})
        return response

    def on_abort(self, request, response):
        now = time.monotonic()
        self.mission.abort(now, self.snapshot(now), 'operator abort service')
        response.success, response.message = True, self.mission.state
        return response

    def check_graph_and_frame(self, now):
        if self.dry or now-self.last_graph_check < 1.0:
            return
        self.last_graph_check = now
        if now-self.frame_checked_s > self.c.state_timeout_s:
            self.frame_verified = False
        # Position/raw/unstamped publishers can also fight this one velocity stream.
        topics = [self.command_topic, self.ns+'/setpoint_velocity/cmd_vel_unstamped',
                  self.ns+'/setpoint_position/local', self.ns+'/setpoint_raw/local',
                  self.ns+'/setpoint_raw/global', self.ns+'/setpoint_raw/attitude',
                  self.ns+'/setpoint_attitude/attitude', self.ns+'/setpoint_attitude/thrust',
                  self.ns+'/setpoint_attitude/cmd_vel', self.ns+'/actuator_control']
        conflicts = []
        for topic in topics:
            infos = self.get_publishers_info_by_topic(topic)
            if topic == self.command_topic and len(infos) != 1:
                conflicts.append(f'{topic}: expected exactly one publisher, got {len(infos)}')
            for info in infos:
                if not (topic == self.command_topic and info.node_name == self.get_name()
                        and info.node_namespace == self.get_namespace()):
                    conflicts.append(f'{topic} ({info.node_namespace}/{info.node_name})')
        self.graph_fault = 'other FC setpoint publishers: '+', '.join(conflicts) if conflicts else ''
        if self.frame_future is None and self.frame_client.service_is_ready():
            req = GetParameters.Request()
            req.names = ['mav_frame']
            self.frame_future = self.frame_client.call_async(req)
        if self.frame_future is not None and self.frame_future.done():
            try:
                values = self.frame_future.result().values
                self.frame_verified = len(values) == 1 and values[0].string_value == 'LOCAL_NED'
                self.frame_checked_s = now
            except Exception:
                self.frame_verified = False
            self.frame_future = None

    def request(self, action, now):
        if not action:
            return
        self.journal.append({'kind': 'service_request', 'time_s': now, 'action': action, 'dry_run': self.dry})
        if self.dry:
            return  # No simulated FC state injected into real telemetry.
        client = self.arm_client if action == 'ARM' else self.mode_client
        req = CommandBool.Request() if action == 'ARM' else SetMode.Request()
        if action == 'ARM':
            req.value = True
        else:
            req.custom_mode = 'AUTO.LAND' if action == 'LAND' else 'OFFBOARD'
        if not client.service_is_ready():
            self.service_failure(action, now, 'service unavailable')
            return
        try:
            future = client.call_async(req)
            self.pending_services.append((action, now, future))
        except Exception:
            self.journal.append({'kind': 'exception', 'time_s': now, 'traceback': traceback.format_exc()})
            self.service_failure(action, now, 'request raised exception')

    def service_failure(self, action, now, reason):
        if action == 'LAND' and self.mission.state == 'LANDING':
            self.mission.result = 'landing_unconfirmed'
            self.mission.transition('FAILED', now, f'LAND {reason}; release to FC OFFBOARD-loss procedure')
        else:
            self.mission.abort(now, self.s, f'{action} {reason}; no retry')

    def tick(self):
        now = time.monotonic()
        dt = now-self.last_tick_s
        self.last_tick_s = now
        s = self.snapshot(now)
        self.check_graph_and_frame(now)
        if not self.dry and self.mission.state not in Mission.TERMINAL and self.mission.state != 'LANDING':
            if self.graph_fault or not self.frame_verified:
                self.mission.abort(now, s, self.graph_fault or 'MAVROS velocity frame unverified')
                self.mission.landing_stream_allowed = False
        if self.journal.error:
            self.mission.abort(now, s, self.journal.error)
        for action, started, future in list(self.pending_services):
            if future.done():
                try:
                    response = future.result()
                    accepted = response.success if action == 'ARM' else response.mode_sent
                except Exception:
                    accepted = False
                self.journal.append({'kind': 'service_response', 'time_s': now, 'action': action, 'accepted': accepted})
                wait_state = {'ARM': 'ARM_WAIT', 'OFFBOARD': 'OFFBOARD_WAIT', 'LAND': 'LANDING'}[action]
                if not accepted and self.mission.state == wait_state:
                    self.service_failure(action, now, 'request rejected')
                self.pending_services.remove((action, started, future))
            elif now-started > self.c.service_timeout_s:
                wait_state = {'ARM': 'ARM_WAIT', 'OFFBOARD': 'OFFBOARD_WAIT', 'LAND': 'LANDING'}[action]
                if self.mission.state == wait_state:
                    self.service_failure(action, now, 'response timed out')
                self.pending_services.remove((action, started, future))
        command = self.mission.step(now, s)
        self.request(command.request, now)
        published = False
        published_at = None
        publish_interval = None
        integral = None
        if command.publish and self.mission.state not in Mission.TERMINAL:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = command.vx, command.vy, command.vz
            msg.twist.angular.z = command.yaw_rate
            try:
                self.publisher.publish(msg)
                published = True  # Local ROS publish only, no FC execution ACK.
                published_at = time.monotonic()
                publish_interval = (published_at-self.mission.last_publish
                                    if self.mission.last_publish is not None else None)
                integral = self.mission.record_publish(command, published_at)
            except Exception:
                self.mission.publish_failed = True
                self.mission.abort(now, s, 'publication/integral failed; no normal completion')
                self.mission.landing_stream_allowed = False
                self.journal.append({'kind': 'exception', 'time_s': now, 'traceback': traceback.format_exc()})
        events, self.mission.events = self.mission.events, []
        for event in events:
            self.journal.append({'kind': 'transition', **event})
            self.get_logger().info(f"{event['from']} -> {event['to']}: {event['reason']}")
        row = {'kind': 'control', 'time_s': now, 'ros_time_s': self.get_clock().now().nanoseconds*1e-9,
               'dt_s': dt, 'state': self.mission.state,
               'integrate_distance': command.integrate_distance,
               'published_at_s': published_at, 'publish_interval_s': publish_interval, 'integral_segment': integral,
               'unlimited_vertical_mps': self.mission.raw_vertical,
               'unlimited_yaw_rate_rad_s': self.c.yaw_kp*wrapped_angle_error(self.mission.yaw_ref, s.yaw)
               if self.mission.yaw_ref is not None and math.isfinite(s.yaw) else None,
               'zero_hold_completed': self.mission.zero_hold_completed,
               'maximum_tick_gap_s': self.c.maximum_tick_gap_s,
               'result': self.mission.result, 'reason': self.mission.reason,
               'command': asdict(command), 'frame': 'ROS_ENU -> MAVLink LOCAL_NED',
               'type_mask': 1479, 'active_fields': ['vx', 'vy', 'vz', 'yaw_rate'], 'published': published, 'output_topic': self.output_topic,
               'yaw_ref': self.mission.yaw_ref, 'commanded_distance_m': self.mission.commanded_distance_m,
               'yaw_error_rad': wrapped_angle_error(self.mission.yaw_ref, s.yaw)
               if self.mission.yaw_ref is not None and math.isfinite(s.yaw) else None,
               'hold_elapsed_s': now-self.mission.hold_since if self.mission.hold_since is not None else 0.0,
               'sensors': asdict(s),
               'height_error_m': self.c.target_height_m-s.height_m if math.isfinite(s.height_m) else None,
               'roll_pitch_rad': [self.roll, self.pitch],
               'telemetry': self.telemetry.copy(), 'telemetry_age_s': now-self.receipts.get('velocity', -math.inf),
               'source_stamps': self.source_stamps.copy(), 'frame_verified': self.frame_verified,
               'topic_age_s': {key: now-ts for key, ts in self.receipts.items()},
               'fc_original_timestamp_s': None}
        self.journal.append(row)
        self.status_pub.publish(String(data=f'{self.mission.state}: {self.mission.result}: {self.mission.reason}'))

    def destroy_node(self):
        self.journal.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FlightNode()
    try:
        rclpy.spin(node)
    except Exception:
        node.journal.append({'kind': 'exception', 'time_s': time.monotonic(), 'traceback': traceback.format_exc()})
        raise
    except KeyboardInterrupt:
        # Shutdown stops streaming; do not send late OFFBOARD/arm requests.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
