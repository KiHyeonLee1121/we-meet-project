"""Deterministic mission logic. No GPS/XY position is an input to this module."""
from dataclasses import dataclass
import math
from .control import body_to_enu, clamp, slew_xy, wrapped_angle_error, yaw_rate_command


@dataclass
class Sensors:
    estimator_valid: bool = False
    estimator_age: float = math.inf
    connected: bool = False
    armed: bool = False
    mode: str = ''
    landed: bool = False
    state_age: float = math.inf
    landed_age: float = math.inf
    yaw: float = math.nan
    yaw_age: float = math.inf
    imu_age: float = math.inf
    yaw_reset: bool = False
    range_m: float = math.nan
    range_rate: float = math.nan
    range_age: float = math.inf
    battery_remaining: float = math.nan
    battery_age: float = math.inf
    camera_age: float = math.inf
    observation: object = None


@dataclass(frozen=True)
class Command:
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0
    publish: bool = False
    request: str = ''


class Mission:
    ACTIVE = {'TAKEOFF', 'SETTLE', 'CRUISE', 'BRAKE', 'ALIGN', 'HOLD'}
    TERMINAL = {'IDLE', 'DONE', 'RELEASED', 'FAILED'}

    def __init__(self, config):
        self.c = config
        self.state = 'IDLE'
        self.reason = ''
        self.result = 'pending'
        self.yaw_ref = None
        self.commanded_distance_m = 0.0
        self.last_time = None
        self.last_command = Command()
        self.last_published = False
        self.integrate_previous = False
        self.xy = (0.0, 0.0)
        self.vz = 0.0
        self.started_s = self.entered_s = 0.0
        self.stable_since = None
        self.hold_since = None
        self.last_target_seen = None
        self.owned = False
        self.pending_request = ''
        self.initial_mode = ''
        self.events = []

    def transition(self, state, now, reason=''):
        self.events.append({'time_s': now, 'from': self.state, 'to': state, 'reason': reason})
        self.state, self.entered_s = state, now
        self.stable_since = self.hold_since = None
        if reason:
            self.reason = reason

    def health_fault(self, s, include_camera=True):
        if not s.connected or not 0 <= s.state_age <= self.c.state_timeout_s:
            return 'FC disconnected or state stale'
        if not s.estimator_valid or not 0 <= s.estimator_age <= self.c.state_timeout_s:
            return 'FC attitude/velocity estimate invalid or stale'
        if (not math.isfinite(s.yaw) or not 0 <= s.yaw_age <= self.c.sensor_timeout_s
                or not 0 <= s.imu_age <= self.c.sensor_timeout_s):
            return 'yaw/IMU invalid or stale'
        if s.yaw_reset:
            return 'estimated heading frame jump; yaw reference not reset'
        if (not math.isfinite(s.range_m) or s.range_m <= 0
                or not math.isfinite(s.range_rate) or not 0 <= s.range_age <= self.c.sensor_timeout_s):
            return 'lidar invalid or stale'
        if (not math.isfinite(s.battery_remaining) or not 0 <= s.battery_age <= 2.0
                or s.battery_remaining < self.c.minimum_battery_remaining):
            return 'battery invalid, stale, or low'
        if self.c.vision_enabled and include_camera and not 0 <= s.camera_age <= self.c.camera_timeout_s:
            return 'camera invalid or stale'
        return ''

    def start(self, now, s, stable_yaw_ref):
        if self.state != 'IDLE':
            raise ValueError('mission is single-shot; restart node for another flight')
        fault = self.health_fault(s)
        if fault:
            raise ValueError(fault)
        if s.armed or not s.landed or not 0 <= s.landed_age <= 2.0 or s.mode == 'OFFBOARD':
            raise ValueError('start requires fresh landed/disarmed state outside OFFBOARD')
        if not math.isfinite(stable_yaw_ref):
            raise ValueError('stable yaw unavailable')
        if abs(wrapped_angle_error(stable_yaw_ref, s.yaw)) > self.c.yaw_stable_deviation_rad:
            raise ValueError('yaw changed after stability window')
        self.yaw_ref = stable_yaw_ref
        self.initial_mode = s.mode
        self.started_s = self.last_time = now
        self.transition('PRESTREAM', now)

    def abort(self, now, s, reason):
        if self.state in self.TERMINAL or self.state == 'LANDING':
            return
        self.result = 'aborted'
        # Do not command modes after the pilot/FC has taken control.
        if (self.owned and s.connected and s.state_age <= self.c.state_timeout_s
                and s.mode == 'OFFBOARD' and s.armed):
            self.transition('LANDING', now, reason)
            self.pending_request = 'LAND'
        else:
            self.transition('RELEASED', now, reason)
        self.xy, self.vz = (0.0, 0.0), 0.0

    def acknowledge_published(self, published):
        self.last_published = published

    def _return(self, command, count_distance=False):
        self.last_command = command
        self.integrate_previous = count_distance
        self.last_published = False
        return command

    def step(self, now, s):
        if self.state in self.TERMINAL:
            return self._return(Command())
        dt = now - self.last_time
        self.last_time = now
        if not math.isfinite(dt) or dt <= 0 or dt > self.c.maximum_tick_gap_s:
            self.abort(now, s, 'control timer gap; distance estimate no longer valid')
            dt = 0.0
        elif self.integrate_previous and self.last_published:
            # Zero-order hold of the previous actually published final command.
            forward = (self.last_command.vx * math.cos(self.yaw_ref)
                       + self.last_command.vy * math.sin(self.yaw_ref))
            self.commanded_distance_m += forward * dt

        if self.state == 'LANDING':
            request, self.pending_request = self.pending_request, ''
            if not s.connected or s.state_age > self.c.state_timeout_s:
                self.result = 'landing_unconfirmed'
                self.transition('FAILED', now, 'FC state lost; release to OFFBOARD-loss procedure')
                return self._return(Command())
            if not s.armed and s.landed and s.landed_age <= 2.0:
                self.transition('DONE', now, 'landed and disarmed')
                return self._return(Command())
            if now-self.entered_s > self.c.landing_timeout_s:
                self.result = 'landing_unconfirmed'
                self.transition('FAILED', now, 'landing timeout; no re-entry or disarm command')
                return self._return(Command())
            if s.mode not in ('OFFBOARD', 'AUTO.LAND'):
                self.result = 'control_released'
                self.transition('RELEASED', now, 'pilot/FC changed mode during landing')
                return self._return(Command())
            # Until LAND is observed, send zero velocities. Never fight AUTO.LAND.
            yr = (yaw_rate_command(target_rad=self.yaw_ref, current_rad=s.yaw,
                                   kp=self.c.yaw_kp, maximum_rate_rad_s=self.c.yaw_max_rate_rad_s)
                  if math.isfinite(s.yaw) and s.yaw_age <= self.c.sensor_timeout_s and not s.yaw_reset else 0.0)
            estimate_ok = (s.estimator_valid and s.estimator_age <= self.c.state_timeout_s
                           and math.isfinite(s.yaw) and s.yaw_age <= self.c.sensor_timeout_s and not s.yaw_reset)
            return self._return(Command(yaw_rate=yr, publish=estimate_ok and s.mode == 'OFFBOARD', request=request))

        if self.state in self.TERMINAL:
            return self._return(Command())
        if self.owned and (s.mode != 'OFFBOARD' or not s.armed):
            self.result = 'control_released'
            self.transition('RELEASED', now, 'pilot/FC mode change or disarm')
            return self._return(Command())
        fault = self.health_fault(s)
        if fault or now-self.started_s > self.c.mission_timeout_s:
            self.abort(now, s, fault or 'mission timeout')
            return self.step(now + 1e-9, s)

        request = ''
        if ((self.state in {'PRESTREAM', 'OFFBOARD_WAIT'} and s.armed)
                or (self.state == 'PRESTREAM' and s.mode != self.initial_mode)
                or (self.state == 'OFFBOARD_WAIT' and s.mode not in {self.initial_mode, 'OFFBOARD'})):
            self.abort(now, s, 'pilot/FC intervened before arming')
            return self._return(Command())
        if self.state == 'PRESTREAM' and now-self.entered_s >= self.c.prestream_s:
            self.transition('OFFBOARD_WAIT', now)
            request = 'OFFBOARD'
        elif self.state == 'OFFBOARD_WAIT':
            if s.mode == 'OFFBOARD':
                self.transition('ARM_WAIT', now)
                request = 'ARM'
            elif now-self.entered_s > self.c.service_timeout_s:
                self.abort(now, s, 'OFFBOARD not accepted')
        elif self.state == 'ARM_WAIT':
            if s.mode != 'OFFBOARD':
                self.abort(now, s, 'mode changed while awaiting arm')
            elif s.armed:
                self.owned = True
                self.transition('TAKEOFF', now)
            elif now-self.entered_s > self.c.service_timeout_s:
                self.abort(now, s, 'arming not accepted')
        if self.state not in self.ACTIVE:
            yr = yaw_rate_command(target_rad=self.yaw_ref, current_rad=s.yaw,
                                  kp=self.c.yaw_kp, maximum_rate_rad_s=self.c.yaw_max_rate_rad_s)
            return self._return(Command(yaw_rate=yr, publish=self.state in {'PRESTREAM', 'OFFBOARD_WAIT', 'ARM_WAIT'},
                                        request=request))

        yaw_error = wrapped_angle_error(self.yaw_ref, s.yaw)
        altitude_ok = (abs(self.c.target_range_m-s.range_m) <= self.c.altitude_tolerance_m
                       and abs(s.range_rate) <= self.c.altitude_rate_tolerance_mps)
        heading_ok = abs(yaw_error) <= self.c.yaw_tolerance_rad
        vertical_target = clamp(self.c.vertical_kp*(self.c.target_range_m-s.range_m)
                                - self.c.vertical_kd*s.range_rate,
                                -self.c.vertical_speed_mps, self.c.vertical_speed_mps)
        self.vz = clamp(vertical_target, self.vz-self.c.vertical_accel_mps2*dt,
                        self.vz+self.c.vertical_accel_mps2*dt)
        desired_xy = (0.0, 0.0)
        obs = s.observation
        visible = bool(obs and obs.visible and 0 <= now-obs.received_s <= self.c.camera_timeout_s)
        if self.state in {'TAKEOFF', 'SETTLE'}:
            if altitude_ok and heading_ok:
                if self.state == 'TAKEOFF':
                    self.transition('SETTLE', now)
                self.stable_since = now if self.stable_since is None else self.stable_since
                if now-self.stable_since >= self.c.settle_s:
                    self.transition('CRUISE' if self.c.approach_enabled else 'HOLD', now)
            else:
                self.stable_since = None
        elif self.state == 'CRUISE':
            if self.c.vision_enabled and visible:
                self.last_target_seen = now
                self.transition('BRAKE', now, 'panel acquired; image control takes over')
            else:
                remaining = max(0.0, self.c.approach_distance_m-self.commanded_distance_m)
                # One-cycle margin plus stopping distance, including the ramp.
                speed = min(self.c.cruise_speed_mps,
                            math.sqrt(max(0.0, 2*self.c.horizontal_accel_mps2*remaining)))
                if remaining <= math.hypot(*self.xy)**2/(2*self.c.horizontal_accel_mps2) + math.hypot(*self.xy)*dt:
                    self.transition('BRAKE', now, 'command-distance stopping point')
                else:
                    desired_xy = body_to_enu(speed, 0.0, self.yaw_ref)
        elif self.state == 'BRAKE':
            if visible:
                self.last_target_seen = now
            if math.hypot(*self.xy) <= self.c.horizontal_accel_mps2*dt:
                if self.c.vision_enabled:
                    if visible:
                        self.transition('ALIGN', now)
                    else:
                        self.abort(now, s, '5m command budget reached or target lost without panel lock')
                else:
                    self.transition('HOLD', now)
        elif self.state in {'ALIGN', 'HOLD'}:
            if self.c.vision_enabled:
                if visible:
                    self.last_target_seen = now
                    centered = abs(obs.ex) <= self.c.center_half_width and abs(obs.ey) <= self.c.center_half_height
                    if not centered:
                        self.hold_since = None
                        if self.state == 'HOLD':
                            self.transition('ALIGN', now, 'panel left centre region')
                        ix = 0.0 if abs(obs.ex) <= self.c.center_half_width else obs.ex
                        iy = 0.0 if abs(obs.ey) <= self.c.center_half_height else obs.ey
                        matrix = self.c.image_to_body
                        bf = self.c.visual_gain_mps*(matrix[0][0]*ix+matrix[0][1]*iy)
                        bl = self.c.visual_gain_mps*(matrix[1][0]*ix+matrix[1][1]*iy)
                        scale = min(1.0, self.c.visual_max_speed_mps/max(1e-9, math.hypot(bf, bl)))
                        # Image errors are body-relative NOW, unlike the fixed approach heading.
                        desired_xy = body_to_enu(bf*scale, bl*scale, s.yaw)
                    elif self.state == 'ALIGN':
                        self.transition('HOLD', now)
                else:
                    centered = False
                    self.hold_since = None
                    if self.last_target_seen is None or now-self.last_target_seen >= self.c.target_loss_s:
                        self.abort(now, s, 'locked panel lost; stop and land')
            else:
                centered = True
            if self.state == 'HOLD':
                settled = centered and altitude_ok and heading_ok and math.hypot(*self.xy) <= 1e-9
                self.hold_since = (now if self.hold_since is None else self.hold_since) if settled else None
                if self.hold_since is not None and now-self.hold_since >= self.c.hover_s:
                    self.result = 'panel_centered_5s' if self.c.vision_enabled else 'command_distance_hover_5s'
                    self.transition('LANDING', now, self.result)
                    self.pending_request = 'LAND'
                    return self.step(now + 1e-9, s)

        if self.state == 'LANDING' or self.state in self.TERMINAL:
            return self.step(now + 1e-9, s)
        self.xy = slew_xy(self.xy, desired_xy, self.c.horizontal_accel_mps2*dt)
        yr = yaw_rate_command(target_rad=self.yaw_ref, current_rad=s.yaw,
                              kp=self.c.yaw_kp, maximum_rate_rad_s=self.c.yaw_max_rate_rad_s)
        return self._return(Command(*self.xy, self.vz, yr, True),
                            count_distance=self.state in {'CRUISE', 'BRAKE'})
