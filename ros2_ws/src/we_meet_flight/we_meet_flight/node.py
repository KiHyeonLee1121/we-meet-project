"""ROS 2 / MAVROS adapter. FC writes are disabled unless dry_run=false + start."""
from collections import deque
from dataclasses import asdict
import math
from pathlib import Path
import queue
import threading
import time
import traceback
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState, Image, Imu, Range
from mavros_msgs.msg import EstimatorStatus, ExtendedState, State
from mavros_msgs.srv import CommandBool, SetMode
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .config import load_config
from .control import quaternion_yaw, wrapped_angle_error, yaw_reset_innovation
from .image_io import image_to_bgr
from .journal import Journal
from .mission import Mission, Sensors
from .vision import PanelDetector, TargetTracker


class FlightNode(Node):
    def __init__(self):
        super().__init__('test_flying_v2')
        default = str(Path(get_package_share_directory('we_meet_flight')) / 'config/panel_approach.yaml')
        params = {'config_file': default, 'dry_run': True, 'flight_settings_verified': False,
                  'camera_axes_verified': False, 'lidar_geometry_verified': False, 'log_directory': '~/flight_logs/test_flying_v2',
                  'mavros_namespace': '/mavros', 'lidar_topic': '/distance/filtered',
                  'image_topic': '/camera/image_raw'}
        for name, value in params.items():
            self.declare_parameter(name, value)
        self.c = load_config(self.get_parameter('config_file').value)
        # Flight permission is captured at startup; it cannot be changed mid-run.
        self.dry = self.get_parameter('dry_run').value
        self.settings_verified = self.get_parameter('flight_settings_verified').value
        self.lidar_verified = self.get_parameter('lidar_geometry_verified').value
        self.roll = self.pitch = math.nan
        self.lidar_capable = False
        self.axes_verified = self.get_parameter('camera_axes_verified').value
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
            'camera_axes_verified': self.axes_verified,
            'unix_time_s': time.time(), 'monotonic_s': time.monotonic(),
            'distance_semantics': 'integral of published forward command, NOT measured travel',
            'software': 'we_meet_flight 0.1.0', 'mavros_namespace': self.ns})
        self.command_topic = self.ns + '/setpoint_velocity/cmd_vel'
        self.output_topic = '/test_flying_v2/debug/cmd_vel' if self.dry else self.command_topic
        self.publisher = self.create_publisher(TwistStamped, self.output_topic, qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '/test_flying_v2/status', 10)
        qos = qos_profile_sensor_data
        self.create_subscription(EstimatorStatus, self.ns+'/estimator_status', self.on_estimator, qos)
        self.create_subscription(State, self.ns+'/state', self.on_state, qos)
        self.create_subscription(ExtendedState, self.ns+'/extended_state', self.on_landed, qos)
        self.create_subscription(PoseStamped, self.ns+'/local_position/pose', self.on_pose, qos)
        self.create_subscription(Imu, self.ns+'/imu/data', self.on_imu, qos)
        self.create_subscription(BatteryState, self.ns+'/battery', self.on_battery, qos)
        self.create_subscription(TwistStamped, self.ns+'/local_position/velocity_local', self.on_velocity, qos)
        self.create_subscription(Range, self.get_parameter('lidar_topic').value, self.on_range, qos)
        self.image_queue = queue.Queue(maxsize=1)
        self.worker_stop = threading.Event()
        self.tracker = TargetTracker(self.c)
        self.tracker_lock = threading.Lock()
        if self.c.vision_enabled:
            self.create_subscription(Image, self.get_parameter('image_topic').value, self.on_image, qos)
            self.worker = threading.Thread(target=self.vision_worker, daemon=True)
            self.worker.start()
        self.mode_client = self.create_client(SetMode, self.ns+'/set_mode')
        self.arm_client = self.create_client(CommandBool, self.ns+'/cmd/arming')
        self.frame_client = self.create_client(GetParameters, self.ns+'/setpoint_velocity/get_parameters')
        self.create_service(Trigger, '/test_flying_v2/start', self.on_start)
        self.create_service(Trigger, '/test_flying_v2/abort', self.on_abort)
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

    def on_landed(self, msg):
        if self.receipt('landed', msg, 2.0) is None:
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

    def on_battery(self, msg):
        if self.receipt('battery', msg, 2.0) is not None:
            self.s.battery_remaining = msg.percentage
            self.telemetry['battery_voltage'] = msg.voltage

    def on_range(self, msg):
        now = self.receipt('range', msg)
        if now is None:
            return
        value = msg.range
        if not math.isfinite(value) or not msg.min_range <= value <= msg.max_range:
            self.s.range_m = math.nan
            self.range_samples.clear()
            return
        self.telemetry['lidar_raw_m'] = value
        self.lidar_capable = msg.max_range >= self.c.target_range_m
        if (not self.lidar_capable or not math.isfinite(self.roll) or not math.isfinite(self.pitch)
                or max(abs(self.roll), abs(self.pitch)) > self.c.maximum_tilt_rad):
            self.s.range_m = math.nan
            self.range_samples.clear()
            return
        if not self.c.lidar_input_is_vertical_height:
            # Body-aligned downward beam, flat surface; offset positive below body origin.
            value = (value+self.c.lidar_body_down_offset_m)*math.cos(self.roll)*math.cos(self.pitch)
        self.s.range_m = value
        self.range_samples.append((now, value))
        while len(self.range_samples) > 2 and now-self.range_samples[0][0] > 0.5:
            self.range_samples.popleft()
        first = self.range_samples[0]
        self.s.range_rate = (value-first[1])/(now-first[0]) if now-first[0] > 0.03 else math.nan

    def on_image(self, msg):
        now = self.receipt('camera_transport', msg, self.c.max_frame_age_s)
        if now is None:
            return
        item = (msg, self.receipts['camera_transport'], self.source_stamps['camera_transport'])
        try:
            self.image_queue.put_nowait(item)
        except queue.Full:
            try:
                self.image_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.image_queue.put_nowait(item)
            except queue.Full:
                pass

    def vision_worker(self):
        detector = PanelDetector(self.c)
        while not self.worker_stop.is_set():
            try:
                msg, received, stamp = self.image_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                if time.monotonic()-received > self.c.camera_timeout_s:
                    continue
                candidates = detector.detect(image_to_bgr(msg))
                with self.tracker_lock:
                    obs = self.tracker.update(candidates, received, stamp)
                    if obs is not None:
                        self.s.observation = obs
                        self.receipts['camera'] = received
                self.journal.append({'kind': 'vision', 'time_s': received, 'source_stamp_s': stamp,
                                     'observation': asdict(obs) if obs else None,
                                     'candidates': [asdict(d) for d in candidates]})
            except Exception as exc:
                self.journal.append({'kind': 'vision_error', 'time_s': received, 'error': str(exc)})

    def snapshot(self, now):
        for key, field in [('estimator', 'estimator_age'), ('state', 'state_age'), ('landed', 'landed_age'), ('yaw', 'yaw_age'),
                           ('imu', 'imu_age'), ('range', 'range_age'), ('battery', 'battery_age'), ('camera', 'camera_age')]:
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
                if not self.settings_verified or (self.c.vision_enabled and not self.axes_verified):
                    raise ValueError('live flight requires verified flight settings and camera axes')
                if not self.frame_verified:
                    raise ValueError('MAVROS setpoint_velocity.mav_frame must be verified LOCAL_NED')
                if self.graph_fault:
                    raise ValueError(self.graph_fault)
                if not self.mode_client.service_is_ready() or not self.arm_client.service_is_ready():
                    raise ValueError('MAVROS mode/arming services unavailable')
            self.mission.start(now, self.snapshot(now), self.stable_yaw(now))
            with self.tracker_lock:
                self.tracker = TargetTracker(self.c)
                self.s.observation = None
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
                  self.ns+'/setpoint_position/local', self.ns+'/setpoint_raw/local']
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
        future = client.call_async(req)
        self.pending_services.append((action, now, future))

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
                if not accepted:
                    self.service_failure(action, now, 'request rejected')
                self.pending_services.remove((action, started, future))
            elif now-started > self.c.service_timeout_s:
                self.service_failure(action, now, 'response timed out')
                self.pending_services.remove((action, started, future))
        command = self.mission.step(now, s)
        self.request(command.request, now)
        published = False
        if command.publish and self.mission.state not in Mission.TERMINAL:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = command.vx, command.vy, command.vz
            msg.twist.angular.z = command.yaw_rate
            try:
                self.publisher.publish(msg)
                published = True  # Local ROS publish only, no FC execution ACK.
            except Exception:
                self.mission.abort(now, s, 'local setpoint publication failed')
                self.journal.append({'kind': 'exception', 'time_s': now, 'traceback': traceback.format_exc()})
        self.mission.acknowledge_published(published)
        events, self.mission.events = self.mission.events, []
        for event in events:
            self.journal.append({'kind': 'transition', **event})
            self.get_logger().info(f"{event['from']} -> {event['to']}: {event['reason']}")
        row = {'kind': 'control', 'time_s': now, 'ros_time_s': self.get_clock().now().nanoseconds*1e-9,
               'dt_s': dt, 'state': self.mission.state,
               'integrate_distance': self.mission.integrate_previous,
               'maximum_tick_gap_s': self.c.maximum_tick_gap_s,
               'result': self.mission.result, 'reason': self.mission.reason,
               'command': asdict(command), 'frame': 'ROS_ENU -> MAVLink LOCAL_NED',
               'type_mask': 1479, 'active_fields': ['vx', 'vy', 'vz', 'yaw_rate'], 'published': published, 'output_topic': self.output_topic,
               'yaw_ref': self.mission.yaw_ref, 'commanded_distance_m': self.mission.commanded_distance_m,
               'yaw_error_rad': wrapped_angle_error(self.mission.yaw_ref, s.yaw)
               if self.mission.yaw_ref is not None and math.isfinite(s.yaw) else None,
               'hold_elapsed_s': now-self.mission.hold_since if self.mission.hold_since is not None else 0.0,
               'sensors': {k: v for k, v in asdict(s).items() if k != 'observation'},
               'observation': asdict(s.observation) if s.observation else None,
               'telemetry': self.telemetry.copy(), 'telemetry_age_s': now-self.receipts.get('velocity', -math.inf),
               'source_stamps': self.source_stamps.copy(), 'frame_verified': self.frame_verified}
        self.journal.append(row)
        self.status_pub.publish(String(data=f'{self.mission.state}: {self.mission.result}: {self.mission.reason}'))

    def destroy_node(self):
        self.worker_stop.set()
        if hasattr(self, 'worker'):
            self.worker.join(timeout=0.5)
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
        rclpy.shutdown()
