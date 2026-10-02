"""Historical 141 adapter around the unchanged attached ROS mission.

No v1/v2 flight core is imported. The source snapshot supplies vertical
controllers, setpoint progression, handovers, panel survey, and full FSM.
"""
import json
import math
from pathlib import Path
import time

from da_daka_control.autonomous_cleaning_mission_node import (
    AutonomousCleaningMissionNode,
)
from da_daka_control.autonomous_cleaning_fsm import CleaningMissionState
from mavros_msgs.msg import HomePosition
from sensor_msgs.msg import NavSatFix
from rclpy.qos import qos_profile_sensor_data
import rclpy
from rclpy.executors import ExternalShutdownException

from we_meet_prototype141.history import (
    LegacyHomeAnchor, home_crosscheck_error, historical_panel_order,
)


class JournalPublisher:
    """Observe the exact archived command after its publish without changing it."""
    def __init__(self, publisher, observer):
        self.publisher, self.observer = publisher, observer

    def publish(self, message):
        self.publisher.publish(message)
        self.observer(message)


class Flight141Mission(AutonomousCleaningMissionNode):
    def __init__(self):
        self._journal = None
        super().__init__()
        directory = Path(str(self.get_parameter('prototype_log_directory').value)).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        session = time.strftime('%Y%m%dT%H%M%S') + f'-{time.time_ns()}'
        self._journal = (directory / f'{session}.jsonl').open('w', buffering=65536)
        self._velocity_publisher = JournalPublisher(
            self._velocity_publisher, self._record_velocity)
        parameters = {name: self.get_parameter(name).value
                      for name in self.list_parameters([], depth=0).names}
        (directory / f'{session}.parameters.json').write_text(
            json.dumps(parameters, ensure_ascii=False, indent=2) + '\n')
        self._record('startup', snapshot='33dcf07', reconstructed_flight=141,
                     historical_home_follow=self._home_follow_enabled)

    def _declare_parameters(self):
        super()._declare_parameters()
        self.declare_parameter('historical_home_follow_enabled', True)
        self.declare_parameter('historical_home_crosscheck_tolerance_m', 1.5)
        self.declare_parameter('historical_home_maximum_shift_m', 10.0)
        self.declare_parameter('historical_global_timeout_s', 6.0)
        self.declare_parameter('historical_panel_order_ids', [3, 1, 2, 6])
        self.declare_parameter('prototype_log_directory', '~/flight141-logs')

    def _load_parameters(self):
        super()._load_parameters()
        self._home_follow_enabled = bool(self.get_parameter(
            'historical_home_follow_enabled').value)
        self._home_tolerance = float(self.get_parameter(
            'historical_home_crosscheck_tolerance_m').value)
        self._home_max_shift = float(self.get_parameter(
            'historical_home_maximum_shift_m').value)
        self._global_timeout = float(self.get_parameter(
            'historical_global_timeout_s').value)
        self._historical_panel_ids = tuple(int(i) for i in self.get_parameter(
            'historical_panel_order_ids').value)
        if (any(i <= 0 for i in self._historical_panel_ids)
                or len(set(self._historical_panel_ids)) != len(self._historical_panel_ids)):
            raise ValueError('historical panel priorities must be unique positive IDs')
        if any(not math.isfinite(v) or v <= 0 for v in (
                self._home_tolerance, self._home_max_shift, self._global_timeout)):
            raise ValueError('historical Home settings must be finite and positive')

    def _initialize_runtime_state(self):
        super()._initialize_runtime_state()
        self._home_xy = None
        self._home_latlon = None
        self._home_time = None
        self._gps_latlon = None
        self._gps_time = None
        self._request_xyz = None
        self._home_anchor = None

    def _create_subscriptions(self, latched_qos):
        super()._create_subscriptions(latched_qos)
        self.create_subscription(HomePosition, '/mavros/home_position/home',
                                 self._home_cb, latched_qos)
        self.create_subscription(NavSatFix, '/mavros/global_position/global',
                                 self._gps_cb, qos_profile_sensor_data)

    def _home_cb(self, message):
        p, g = message.position, message.geo
        values = (float(p.x), float(p.y), float(g.latitude), float(g.longitude))
        if not all(math.isfinite(v) for v in values):
            return
        self._home_xy, self._home_latlon = values[:2], values[2:]
        self._home_time = time.monotonic()

    def _gps_cb(self, message):
        values = (float(message.latitude), float(message.longitude))
        if int(message.status.status) < 0 or not all(math.isfinite(v) for v in values):
            return
        self._gps_latlon, self._gps_time = values, time.monotonic()

    def _reference_failure(self, now):
        if not self._home_follow_enabled:
            return None
        if any(v is None for v in (self._pose_xyz, self._home_xy,
                                    self._home_latlon, self._home_time,
                                    self._gps_latlon, self._gps_time)):
            return 'historical start requires local pose, GPS global and Home'
        # Home is a retained reference, not a periodic position measurement.
        # A latched unchanged Home must remain usable after launch preparation.
        if not 0 <= now - self._gps_time <= self._global_timeout:
            return 'GPS reference data stale'
        try:
            error = home_crosscheck_error(self._pose_xyz, self._home_xy,
                                         self._gps_latlon, self._home_latlon)
        except ValueError as exc:
            return str(exc)
        if error > self._home_tolerance:
            return f'GPS/Home local offset disagrees by {error:.3f} m'
        return None

    def _start_callback(self, request, response):
        if self._pose_xyz is None:
            response.success, response.message = False, 'local pose unavailable'
            return response
        failure = self._reference_failure(time.monotonic())
        if failure is not None:
            response.success, response.message = False, failure
            return response
        return super()._start_callback(request, response)

    def _precheck_failures(self, now_s):
        failures = super()._precheck_failures(now_s)
        failure = self._reference_failure(now_s)
        if failure:
            failures.append(failure)
        return failures

    def _reset_mission_bookkeeping(self):
        super()._reset_mission_bookkeeping()
        self._request_xyz = self._pose_xyz
        self._home_anchor = (
            LegacyHomeAnchor(self._request_xyz[:2], self._home_xy, self._home_max_shift)
            if self._home_follow_enabled else None
        )
        self._record('mission_request_reference', request_xyz=self._request_xyz,
                     start_home_xy=self._home_xy,
                     offset_xy=self._home_anchor.offset_xy if self._home_anchor else None)

    def _tick_precheck(self, now_s):
        failures = self._precheck_failures(now_s)
        if failures:
            self._fail('preflight failed: ' + '; '.join(failures), False)
            return
        if self._mode != self._loiter_mode:
            self._request_mode(self._loiter_mode)
            return
        stable_yaw = self._launch_yaw_stability.stable_yaw_rad
        if stable_yaw is None:
            return
        try:
            xy = self._home_anchor.target_xy(self._home_xy) if self._home_anchor else self._request_xyz[:2]
        except ValueError as exc:
            self._fail(str(exc), False)
            return
        self._launch_xyz = (*xy, self._request_xyz[2])
        self._launch_yaw_rad = stable_yaw
        self._record('prearm_reference', launch_xyz=self._launch_xyz,
                     launch_yaw_rad=self._launch_yaw_rad)
        previous = self._fsm.state
        self._fsm.precheck_complete()
        self._on_transition(previous)

    def _tick_plan_route(self, now_s):
        if not self._historical_panel_ids:
            return super()._tick_plan_route(now_s)
        if self._launch_xyz is None or self._pose_xyz is None:
            self._fail('route planning lacks launch/current pose', True)
            return
        try:
            order = historical_panel_order(
                self._pose_xyz[:2], [p.target for p in self._fsm.panels],
                self._launch_xyz[:2], self._historical_panel_ids)
            previous = self._fsm.state
            self._fsm.route_planned(order)
            self._on_transition(previous)
            self.get_logger().info(f'141 reconstructed historical route {order}')
            self._record('panel_route', order=order,
                         provenance='141 recorded ID priority; original algorithm unavailable')
        except ValueError as exc:
            self._fail(f'route planning failed: {exc}', True)

    def _refresh_historical_home_anchor(self):
        if self._home_anchor is None or self._launch_xyz is None or self._home_xy is None:
            return True
        try:
            xy = self._home_anchor.target_xy(self._home_xy)
        except ValueError as exc:
            self._fail(str(exc), self._armed)
            return False
        before = self._launch_xyz
        self._launch_xyz = (*xy, before[2])
        if math.hypot(xy[0]-before[0], xy[1]-before[1]) > 1e-6:
            self.get_logger().warning(
                f'Historical Home-follow: launch {before[:2]} -> {xy}; '
                'reproducing the 141 reference motion, not a local-origin reset')
            self._record('historical_home_shift', before_xy=before[:2], after_xy=xy,
                         home_xy=self._home_xy)
        return True

    def _tick(self):
        if self._fsm.active and not self._refresh_historical_home_anchor():
            return
        # Keep the archived pre-arm yaw unchanged. No reset abort or
        # compensation from v1/v2 or post-flight 1dc425c is applied.
        super()._tick()
        if self._fsm.active:
            self._record('tick', state=self._fsm.state.name, stage=self._stage,
                         mode=self._mode, armed=self._armed,
                         pose_xyz=self._pose_xyz, velocity_xyz=self._velocity_xyz,
                         yaw_rad=self._yaw_rad, launch_xyz=self._launch_xyz,
                         launch_yaw_rad=self._launch_yaw_rad, distance_m=self._distance_m,
                         scan_index=self._localization_scan_index,
                         position_command=self._command_xyz)

    def _publish_position_hold(self, xyz):
        super()._publish_position_hold(xyz)
        if xyz is not None and self._launch_yaw_rad is not None:
            self._record('position_setpoint', frame='ENU/map', xyz=xyz,
                         yaw_rad=self._launch_yaw_rad)

    def _record_velocity(self, message):
        v = message.twist.linear
        self._record('velocity_setpoint', frame=message.header.frame_id,
                     xyz_mps=(float(v.x), float(v.y), float(v.z)),
                     yaw_rate_rad_s=float(message.twist.angular.z))

    def _fail(self, reason, request_land):
        super()._fail(reason, request_land)
        self._record('abort', reason=reason, request_land=request_land,
                     mode=self._mode, armed=self._armed)
        if self._journal is not None:
            self._journal.flush()

    def _on_transition(self, previous, preserve_control=False):
        super()._on_transition(previous, preserve_control)
        self._record('state_transition', previous=previous.name,
                     state=self._fsm.state.name, reason=self._fsm.reason)
        if self._journal is not None:
            self._journal.flush()

    def _record(self, event, **values):
        if self._journal is not None:
            self._journal.write(json.dumps(dict(
                event=event, monotonic_s=time.monotonic(), wall_ns=time.time_ns(),
                **values), ensure_ascii=False, allow_nan=False) + '\n')

    def destroy_node(self):
        if self._journal is not None:
            self._journal.close()
            self._journal = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Flight141Mission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
