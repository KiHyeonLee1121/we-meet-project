#!/usr/bin/env python3
"""Replay selected October 2 failure signals against independent safety guards.

Other sensor values are synthetic and healthy in each isolated case. This is
not the flown controller, a complete sensor/ROS replay, SITL, or flight proof.
"""
from dataclasses import replace
import json
import math
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ros2_ws/src/we_meet_flight_core'))
from we_meet_flight_core.config import Config
from we_meet_flight_core.mission import Mission, Sensors


def baseline():
    return Sensors(connected=True, mode='POSCTL', landed=True, state_age=0, landed_age=0,
                   estimator_valid=True, estimator_age=0, yaw=0, yaw_age=0, imu_age=0,
                   height_m=3, height_rate_mps=0, range_age=0, battery_remaining=.8, battery_age=0,
                   odometry_age=0, odometry_stamp_s=0, fc_reset_counter=2, position_enu=(0, 0, 0),
                   velocity_enu=(0, 0, 0), position_sigma_m=.1, velocity_sigma_mps=.05,
                   gps_age=0, gps_sigma_m=.2, yaw_rate_rps=0)


def isolated_mission():
    s = baseline()
    m = Mission(Config())
    m.start(0, s, 0)
    return m, s


def replay():
    data = json.loads((Path(__file__).resolve().parents[1]/'tests/fixtures/oct02_failure_signals.json').read_text())
    cases = {}
    s = baseline(); s.gps_sigma_m = math.sqrt(data['initial_gps_variance_m2'])
    try:
        Mission(Config()).start(0, s, 0)
        reason = ''
    except ValueError as exc:
        reason = str(exc)
    cases['gnss_preflight'] = {'detected': 'GNSS' in reason, 'sigma_m': s.gps_sigma_m, 'reason': reason}
    m, s = isolated_mission()
    reason = ''
    for t, error in data['yaw_time_error_rad']:
        s.yaw = -error
        s.odometry_stamp_s = t
        reason = m.flight_guard_fault(t, s)
        if reason: break
    cases['early_yaw_excursion'] = {'detected': 'sustained heading' in reason, 'source_seconds_after_ASCEND': t, 'reason': reason}
    m, s = isolated_mission()
    previous = data['fc_time_combined_reset_yaw_reset'][0][1]
    reason = ''
    for t, counter, yaw_counter in data['fc_time_combined_reset_yaw_reset']:
        s.fc_reset = counter != previous
        s.fc_reset_counter = counter
        reason = m.health_fault(s)
        if reason: break
        previous = counter
    cases['fc_reset'] = {'detected': 'reset counter' in reason, 'source_ulog_seconds': t, 'combined_counter': counter,
                         'yaw_counter_log_only': yaw_counter, 'reason': reason}
    m, s = isolated_mission()
    # Isolate consistency from the separate path-deviation detector.
    m.c = replace(m.c, horizontal_tracking_limit_m=100)
    rows = data['fc_time_east_north_ve_vn']
    m.launch_xy = tuple(rows[0][1:3]); reason = ''
    for t, east, north, ve, vn in rows:
        s.odometry_stamp_s = t
        s.position_enu, s.velocity_enu = (east, north, 0), (ve, vn, 0)
        reason = m.flight_guard_fault(t, s)
        if reason: break
    cases['position_velocity_inconsistency'] = {'detected': 'position change inconsistent' in reason,
                                               'source_ulog_seconds': t, 'reason': reason}
    m, s = isolated_mission(); history = []; reason = ''
    for t, height, vz in data['lidar_time_height_fc_vz_up']:
        history.append((t, height))
        while len(history)>2 and t-history[1][0] >= m.c.lidar_rate_window_s: history.pop(0)
        if t-history[0][0] < .9*m.c.lidar_rate_window_s: continue
        s.height_rate_mps = (height-history[0][1])/(t-history[0][0])
        s.velocity_enu = (0, 0, vz)
        s.odometry_stamp_s = t
        reason = m.flight_guard_fault(t, s)
        if reason: break
    cases['lidar_fc_vertical_disagreement'] = {'detected': 'vertical velocity disagreement' in reason,
                                              'source_seconds_after_first_ROS_sample': t, 'reason': reason}
    return {'cases': cases, 'ulog_sha256': data['ulog_sha256'], 'full_aircraft_or_flight_validation': False,
            'scope': 'original selected scalars; isolated guard replay with synthetic healthy remaining inputs'}


if __name__ == '__main__':
    result = replay()
    print(json.dumps(result, indent=2))
    if not all(case['detected'] for case in result['cases'].values()): raise SystemExit(1)
