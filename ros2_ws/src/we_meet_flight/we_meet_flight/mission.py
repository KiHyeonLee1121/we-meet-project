"""Camera extension; takeoff, advance, braking, yaw, Z and LAND use core."""
from dataclasses import dataclass
import math
from we_meet_flight_core.control import body_to_enu
from we_meet_flight_core.mission import Command, Mission as CoreMission, Sensors as CoreSensors


@dataclass
class Sensors(CoreSensors):
    camera_age: float = math.inf
    observation: object = None


class Mission(CoreMission):
    ACTIVE = CoreMission.ACTIVE | {'ALIGN', 'VISUAL_HOLD'}

    def __init__(self, config):
        super().__init__(config)
        self.visual_braking = False
        self.last_target_seen = None
        self.hold_eligible = False

    def visible(self, s, now):
        obs = s.observation
        return bool(obs and obs.visible and math.isfinite(obs.ex) and math.isfinite(obs.ey)
                    and 0 <= now-obs.received_s <= self.c.camera_timeout_s)

    def extra_health_fault(self, s):
        if not self.c.vision_enabled:
            return ''
        if not 0 <= s.camera_age <= self.c.camera_timeout_s:
            return 'camera invalid or stale'
        now = self.last_tick
        if self.visual_braking and now is not None:
            if self.visible(s, now):
                self.last_target_seen = s.observation.received_s
            elif self.last_target_seen is None or now-self.last_target_seen >= self.c.target_loss_s:
                return 'locked panel lost; stop and land without forward resume'
        return ''

    def phase_timeouts(self):
        return {**super().phase_timeouts(), 'ALIGN': self.c.align_timeout_s,
                'VISUAL_HOLD': self.c.visual_hold_timeout_s}

    def advance_handoff(self, now, s):
        if self.c.vision_enabled and self.visible(s, now):
            self.visual_braking = True
            self.last_target_seen = s.observation.received_s
            self.transition('BRAKE', now, 'panel acquired; common velocity ramp brakes before image alignment')
            return True
        return False

    def brake_complete(self, now, s):
        if not self.c.vision_enabled:
            super().brake_complete(now, s)
        elif self.visual_braking and self.visible(s, now):
            self.transition('ALIGN', now, 'common braking complete; camera controls XY')
        else:
            self.abort(now, s, '5m command budget reached or target lost without panel lock')

    def transition(self, state, now, reason=''):
        if state in {'ALIGN', 'VISUAL_HOLD'}:
            self.hold_since = None
            self.hold_eligible = False
        super().transition(state, now, reason)

    def extended_xy_target(self, now, s, altitude_ok, heading_ok):
        visible = self.visible(s, now)
        centered = visible and abs(s.observation.ex) <= self.c.center_half_width and abs(s.observation.ey) <= self.c.center_half_height
        self.hold_eligible = centered and altitude_ok and heading_ok
        if not self.hold_eligible:
            self.hold_since = None
        if not visible:
            if self.state == 'VISUAL_HOLD':
                self.transition('ALIGN', now, 'panel unavailable; restart continuous hold')
            return (0.0, 0.0)
        if not centered:
            if self.state == 'VISUAL_HOLD':
                self.transition('ALIGN', now, 'panel left centre region')
            obs = s.observation
            ix = 0.0 if abs(obs.ex) <= self.c.center_half_width else obs.ex
            iy = 0.0 if abs(obs.ey) <= self.c.center_half_height else obs.ey
            matrix = self.c.image_to_body
            bf = self.c.visual_gain_mps*(matrix[0][0]*ix+matrix[0][1]*iy)
            bl = self.c.visual_gain_mps*(matrix[1][0]*ix+matrix[1][1]*iy)
            scale = min(1.0, self.c.visual_max_speed_mps/max(1e-9, math.hypot(bf, bl)))
            return body_to_enu(bf*scale, bl*scale, s.yaw)
        if self.state == 'ALIGN':
            self.transition('VISUAL_HOLD', now)
            self.hold_eligible = altitude_ok and heading_ok
        if self.hold_since is not None and now-self.hold_since >= self.c.zero_velocity_hold_s:
            self.zero_hold_completed = True
            self.begin_land(now, s, 'panel centred continuously with zero XY command for 5s', 'panel_centered_5s')
        return (0.0, 0.0)

    def after_publish(self, command, now):
        if self.state == 'VISUAL_HOLD':
            fresh = self.last_target_seen is not None and 0 <= now-self.last_target_seen <= self.c.camera_timeout_s
            if self.hold_eligible and fresh and command.vx == 0 and command.vy == 0:
                if self.hold_since is None:
                    self.hold_since = now
            else:
                self.hold_since = None
