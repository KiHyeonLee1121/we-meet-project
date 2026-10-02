"""Camera-free velocity trial. XY position/GPS are intentionally NOT inputs."""
from dataclasses import dataclass
import math
from .control import body_to_enu, clamp, slew_xy, yaw_rate_command, wrapped_angle_error


@dataclass
class Sensors:
    connected: bool = False
    armed: bool = False
    mode: str = ''
    landed: bool = False
    state_age: float = math.inf
    landed_age: float = math.inf
    estimator_valid: bool = False
    estimator_age: float = math.inf
    failsafe_observed: bool = False
    yaw: float = math.nan
    yaw_age: float = math.inf
    imu_age: float = math.inf
    yaw_reset: bool = False
    height_m: float = math.nan
    height_rate_mps: float = math.nan
    range_age: float = math.inf
    battery_remaining: float = math.nan
    battery_age: float = math.inf


@dataclass(frozen=True)
class Command:
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0
    publish: bool = False
    request: str = ''
    integrate_distance: bool = False


class Mission:
    TERMINAL = {'IDLE', 'COMPLETE', 'RELEASED', 'FAILED'}
    ACTIVE = {'ASCEND', 'SETTLE', 'ADVANCE', 'BRAKE', 'ZERO_VELOCITY_HOLD'}

    def __init__(self, config):
        self.c = config
        self.state, self.result, self.reason = 'IDLE', 'pending', ''
        self.yaw_ref = None
        self.commanded_distance_m = 0.0
        self.speed = self.vz = 0.0
        self.xy = (0.0, 0.0)
        self.raw_vertical = None
        self.last_tick = None
        self.last_publish = None
        self.last_published_command = None
        self.last_integral = None
        self.started_s = self.entered_s = 0.0
        self.stable_since = self.hold_since = None
        self.zero_hold_completed = False
        self.owned = False
        self.initial_mode = ''
        self.events = []
        self.pending_land = False
        self.landing_stream_allowed = False
        self.publish_failed = False

    def transition(self, state, now, reason=''):
        self.events.append({'time_s': now, 'from': self.state, 'to': state, 'reason': reason})
        self.state, self.entered_s = state, now
        self.stable_since = None
        if reason:
            self.reason = reason

    def velocity_estimate_ok(self, s):
        return (s.connected and 0 <= s.state_age <= self.c.state_timeout_s and s.estimator_valid
                and 0 <= s.estimator_age <= self.c.state_timeout_s and not s.failsafe_observed
                and math.isfinite(s.yaw) and 0 <= s.yaw_age <= self.c.sensor_timeout_s
                and 0 <= s.imu_age <= self.c.sensor_timeout_s and not s.yaw_reset)

    def health_fault(self, s):
        if not s.connected or not 0 <= s.state_age <= self.c.state_timeout_s:
            return 'FC connection/state lost'
        if s.failsafe_observed:
            return 'FC emergency/critical system status observed'
        if not s.estimator_valid or not 0 <= s.estimator_age <= self.c.state_timeout_s:
            return 'attitude/velocity estimator invalid or stale'
        if not math.isfinite(s.yaw) or not 0 <= s.yaw_age <= self.c.sensor_timeout_s or not 0 <= s.imu_age <= self.c.sensor_timeout_s:
            return 'yaw/IMU invalid or stale'
        if s.yaw_reset:
            return 'heading frame jump; fixed yaw reference not shifted'
        if (not math.isfinite(s.height_m) or s.height_m <= 0 or not math.isfinite(s.height_rate_mps)
                or not 0 <= s.range_age <= self.c.sensor_timeout_s):
            return 'lidar height/rate invalid or stale'
        if (not math.isfinite(s.battery_remaining) or not 0 <= s.battery_age <= self.c.battery_timeout_s
                or s.battery_remaining < self.c.minimum_battery_remaining):
            return 'battery invalid/stale/low'
        return self.extra_health_fault(s)

    def extra_health_fault(self, s):
        return ''

    def phase_timeouts(self):
        return {'ASCEND': self.c.ascend_timeout_s, 'SETTLE': self.c.settle_timeout_s,
                'ADVANCE': self.c.advance_timeout_s, 'BRAKE': self.c.brake_timeout_s,
                'ZERO_VELOCITY_HOLD': self.c.hold_timeout_s}

    def advance_handoff(self, now, s):
        """An extension may acquire a target and enter the common BRAKE stage."""
        return False

    def brake_complete(self, now, s):
        estimated = self.distance_at(now)
        if abs(estimated-self.c.commanded_target_distance_m) > self.c.commanded_distance_tolerance_m:
            self.abort(now, s, 'command distance outside tolerance; no normal completion')
        else:
            self.transition('ZERO_VELOCITY_HOLD', now, 'expected 5m command integral completed; actual arrival unverified')

    def extended_xy_target(self, now, s, altitude_ok, heading_ok):
        raise RuntimeError('unknown flight stage: '+self.state)

    def after_publish(self, command, now):
        pass

    def start(self, now, s, stable_yaw_ref):
        if self.state != 'IDLE':
            raise ValueError('single-shot process; restart for another trial')
        fault = self.health_fault(s)
        if fault:
            raise ValueError(fault)
        if s.armed or not s.landed or not 0 <= s.landed_age <= self.c.landed_timeout_s or s.mode == 'OFFBOARD':
            raise ValueError('fresh landed/disarmed state outside OFFBOARD required')
        if not math.isfinite(stable_yaw_ref) or abs(wrapped_angle_error(stable_yaw_ref, s.yaw)) > self.c.yaw_stable_deviation_rad:
            raise ValueError('stable yaw reference unavailable')
        self.yaw_ref = stable_yaw_ref
        self.initial_mode = s.mode
        self.started_s = self.last_tick = now
        self.transition('PRESTREAM', now)

    def begin_land(self, now, s, reason, result):
        self.result = result
        self.speed = self.vz = 0.0
        self.xy = (0.0, 0.0)
        self.landing_stream_allowed = self.velocity_estimate_ok(s) and not self.publish_failed
        self.pending_land = True
        self.transition('LANDING', now, reason)

    def abort(self, now, s, reason):
        if self.state in self.TERMINAL or self.state == 'LANDING':
            return
        if (self.owned and s.connected and 0 <= s.state_age <= self.c.state_timeout_s
                and s.mode == 'OFFBOARD' and s.armed and not s.failsafe_observed):
            self.begin_land(now, s, reason, 'aborted')
        else:
            self.result = 'aborted'
            self.transition('RELEASED', now, reason)

    def record_publish(self, command, now):
        """Called AFTER successful local publish; integrate previous published command once."""
        segment = None
        if self.last_publish is not None:
            dt = now-self.last_publish
            old = self.last_published_command
            if old.integrate_distance:
                if not 0 < dt <= self.c.maximum_tick_gap_s:
                    raise ValueError('published command gap invalidates distance estimate')
                forward = old.vx*math.cos(self.yaw_ref)+old.vy*math.sin(self.yaw_ref)
                delta = forward*dt  # Signed projection; Z never contributes.
                self.commanded_distance_m += delta
                segment = {'start_s': self.last_publish, 'end_s': now, 'dt_s': dt,
                           'forward_mps': forward, 'distance_m': delta,
                           'cumulative_m': self.commanded_distance_m}
        self.last_publish, self.last_published_command = now, command
        self.last_integral = segment
        if self.state == 'ZERO_VELOCITY_HOLD' and command.vx == 0 and command.vy == 0 and self.hold_since is None:
            self.hold_since = now  # First actual local publication of zero XY, not estimated ground speed.
        self.after_publish(command, now)
        return segment

    def distance_at(self, now):
        """Predict current held-command integral without mutating/double-counting it."""
        d = self.commanded_distance_m
        if self.last_publish is not None and self.last_published_command.integrate_distance:
            dt = now-self.last_publish
            if 0 <= dt <= self.c.maximum_tick_gap_s:
                old = self.last_published_command
                d += (old.vx*math.cos(self.yaw_ref)+old.vy*math.sin(self.yaw_ref))*dt
        return d

    def landing_command(self, now, s):
        if not s.connected or not 0 <= s.state_age <= self.c.state_timeout_s:
            self.result = 'landing_unconfirmed'
            self.transition('FAILED', now, 'FC state lost; stop stream and defer to FC failsafe')
            return Command()
        if not s.armed and s.landed and 0 <= s.landed_age <= self.c.landed_timeout_s:
            self.transition('COMPLETE', now, 'landed and disarmed observed')
            return Command()
        if s.mode not in {'OFFBOARD', 'AUTO.LAND'} or s.failsafe_observed:
            self.result = 'control_released'
            self.transition('RELEASED', now, 'pilot/FC control takeover')
            return Command()
        if now-self.entered_s > self.c.landing_timeout_s:
            self.result = 'landing_unconfirmed'
            self.transition('FAILED', now, 'LAND timeout; no re-entry or airborne disarm')
            return Command()
        if s.mode == 'AUTO.LAND':
            self.pending_land = False
            return Command()  # Release immediately, including yaw.
        request = 'LAND' if self.pending_land else ''
        self.pending_land = False
        if not self.velocity_estimate_ok(s):
            self.landing_stream_allowed = False  # Never resume stream after estimator loss.
        yr = self.yaw_rate(s) if self.landing_stream_allowed else 0.0
        return Command(yaw_rate=yr, publish=self.landing_stream_allowed, request=request)

    def yaw_rate(self, s):
        return yaw_rate_command(target_rad=self.yaw_ref, current_rad=s.yaw,
                                kp=self.c.yaw_kp, maximum_rate_rad_s=self.c.yaw_max_rate_rad_s)

    def step(self, now, s):
        if self.state in self.TERMINAL:
            return Command()
        dt = now-self.last_tick
        self.last_tick = now
        if not math.isfinite(dt) or not 0 < dt <= self.c.maximum_tick_gap_s:
            self.abort(now, s, 'control timer gap; no distance catch-up')
            self.landing_stream_allowed = False
        if self.state == 'LANDING':
            return self.landing_command(now, s)
        if self.state in self.TERMINAL:
            return Command()
        if self.owned and (s.mode != 'OFFBOARD' or not s.armed):
            self.result = 'control_released'
            self.transition('RELEASED', now, 'pilot/FC mode change or disarm')
            return Command()
        fault = self.health_fault(s)
        if fault or now-self.started_s > self.c.mission_timeout_s:
            self.abort(now, s, fault or 'mission timeout')
            return self.landing_command(now, s) if self.state == 'LANDING' else Command()
        timeouts = self.phase_timeouts()
        if self.state in timeouts and now-self.entered_s > timeouts[self.state]:
            self.abort(now, s, self.state+' phase timeout')
            return self.landing_command(now, s) if self.state == 'LANDING' else Command()
        request = ''
        if self.state in {'PRESTREAM', 'OFFBOARD_WAIT'}:
            allowed_modes = {self.initial_mode} if self.state == 'PRESTREAM' else {self.initial_mode, 'OFFBOARD'}
            if s.armed or s.mode not in allowed_modes:
                self.abort(now, s, 'pilot/FC intervention before arm')
                return Command()
        if self.state == 'PRESTREAM' and now-self.entered_s >= self.c.prestream_s:
            if self.last_publish is None or now-self.last_publish > self.c.maximum_tick_gap_s:
                self.abort(now, s, 'setpoint prestream not established')
                return Command()
            self.transition('OFFBOARD_WAIT', now)
            request = 'OFFBOARD'
        elif self.state == 'OFFBOARD_WAIT':
            if s.mode == 'OFFBOARD':
                self.transition('ARM_WAIT', now)
                request = 'ARM'
            elif now-self.entered_s > self.c.service_timeout_s:
                self.abort(now, s, 'OFFBOARD not observed')
        elif self.state == 'ARM_WAIT':
            if s.mode != 'OFFBOARD':
                self.abort(now, s, 'pilot/FC intervention while awaiting ARM')
            elif s.armed:
                self.owned = True
                self.transition('ASCEND', now)
            elif now-self.entered_s > self.c.service_timeout_s:
                self.abort(now, s, 'armed state not observed')
        if self.state not in self.ACTIVE:
            return Command(yaw_rate=self.yaw_rate(s), publish=self.state in {'PRESTREAM', 'OFFBOARD_WAIT', 'ARM_WAIT'}, request=request)

        height_error = self.c.target_height_m-s.height_m
        self.raw_vertical = self.c.vertical_kp*height_error-self.c.vertical_kd*s.height_rate_mps
        desired_z = clamp(self.raw_vertical,
                          -self.c.vertical_speed_mps, self.c.vertical_speed_mps)
        self.vz = clamp(desired_z, self.vz-self.c.vertical_accel_mps2*dt, self.vz+self.c.vertical_accel_mps2*dt)
        altitude_ok = abs(height_error) <= self.c.altitude_tolerance_m and abs(s.height_rate_mps) <= self.c.altitude_rate_tolerance_mps
        heading_ok = abs(wrapped_angle_error(self.yaw_ref, s.yaw)) <= self.c.yaw_tolerance_rad
        desired_speed = 0.0
        extended_xy = None
        if self.state in {'ASCEND', 'SETTLE'}:
            if altitude_ok and heading_ok:
                if self.state == 'ASCEND':
                    self.transition('SETTLE', now)
                self.stable_since = now if self.stable_since is None else self.stable_since
                if now-self.stable_since >= self.c.settle_s:
                    self.commanded_distance_m = 0.0
                    self.transition('ADVANCE' if self.c.advance_enabled else 'ZERO_VELOCITY_HOLD', now)
            else:
                self.stable_since = None
        elif self.state == 'ADVANCE':
            if not self.advance_handoff(now, s):
                remaining = self.c.commanded_target_distance_m-self.distance_at(now)
                # Braking command area + half maximum dispatch interval as scheduling margin.
                stop_area = self.speed*self.speed/(2*self.c.horizontal_accel_mps2)
                margin = self.speed*self.c.maximum_tick_gap_s/2
                if remaining <= stop_area+margin:
                    self.transition('BRAKE', now, 'brake before expected 5m, including deceleration command area')
                else:
                    desired_speed = self.c.forward_speed_mps
        elif self.state == 'BRAKE':
            if self.speed <= self.c.horizontal_accel_mps2*dt+1e-12:
                self.brake_complete(now, s)
        elif self.state == 'ZERO_VELOCITY_HOLD':
            # No XY position/velocity is consulted and no horizontal correction is applied.
            if self.hold_since is not None and now-self.hold_since >= self.c.zero_velocity_hold_s:
                self.zero_hold_completed = True
                result = 'expected_5m_zero_command_5s' if self.c.advance_enabled else 'ascent_zero_command_5s'
                self.begin_land(now, s, 'horizontal zero-velocity command held for 5s', result)
                return self.landing_command(now, s)
        else:
            extended_xy = self.extended_xy_target(now, s, altitude_ok, heading_ok)
        if self.state == 'LANDING':
            return self.landing_command(now, s)
        if self.state in self.TERMINAL:
            return Command()
        self.speed = clamp(desired_speed, max(0.0, self.speed-self.c.horizontal_accel_mps2*dt),
                           self.speed+self.c.horizontal_accel_mps2*dt)
        if extended_xy is None:
            self.xy = body_to_enu(self.speed, 0.0, self.yaw_ref)
        else:
            self.xy = slew_xy(self.xy, extended_xy, self.c.horizontal_accel_mps2*dt)
        vx, vy = self.xy
        return Command(vx, vy, self.vz, self.yaw_rate(s), True,
                       integrate_distance=self.state in {'ADVANCE', 'BRAKE'})
