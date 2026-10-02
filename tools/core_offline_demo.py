#!/usr/bin/env python3
"""Pure synthetic command-response plant. No ROS, ports, network, or flight."""
import argparse
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ros2_ws/src/we_meet_flight_core'))
from we_meet_flight_core.config import Config
from we_meet_flight_core.mission import Mission, Sensors


def run_demo(output=None, ascent_only=False, irregular=False):
    c = Config(advance_enabled=not ascent_only)
    m = Mission(c)
    s = Sensors(connected=True, mode='POSCTL', landed=True, state_age=0, landed_age=0,
                estimator_valid=True, estimator_age=0, yaw=0.3, yaw_age=0, imu_age=0,
                height_m=0.3, height_rate_mps=0, range_age=0, battery_remaining=1, battery_age=0)
    now, x, y, z = 0.0, 0.0, 0.0, 0.3
    previous = None
    m.start(now, s, s.yaw)
    rows = []
    periods = [0.03, 0.08, 0.05, 0.10, 0.04] if irregular else [0.05]
    for i in range(3500):
        dt = periods[i % len(periods)]
        now += dt
        if previous and previous.publish:
            x += previous.vx*dt
            y += previous.vy*dt
            z += previous.vz*dt
            s.yaw += previous.yaw_rate*dt
        if s.mode == 'AUTO.LAND':
            z = max(0.3, z-0.3*dt)
            if z <= 0.3:
                s.armed, s.landed = False, True
        old_height = s.height_m
        s.height_m, s.height_rate_mps = z, (z-old_height)/dt
        if s.armed and z > 0.35:
            s.landed = False
        cmd = m.step(now, s)
        segment = m.record_publish(cmd, now) if cmd.publish else None
        if cmd.request == 'OFFBOARD':
            s.mode = 'OFFBOARD'
        elif cmd.request == 'ARM':
            s.armed = True
        elif cmd.request == 'LAND':
            s.mode = 'AUTO.LAND'
        rows.append({'kind': 'control', 'time_s': now, 'published_at_s': now if cmd.publish else None,
                     'state': m.state, 'command': asdict(cmd), 'published': cmd.publish,
                     'integrate_distance': cmd.integrate_distance, 'integral_segment': segment,
                     'yaw_ref': m.yaw_ref, 'maximum_tick_gap_s': c.maximum_tick_gap_s,
                     'commanded_distance_m': m.commanded_distance_m, 'synthetic_xyz': [x, y, z],
                     'result': m.result})
        previous = cmd
        if m.state in {'COMPLETE', 'FAILED', 'RELEASED'}:
            break
    if output:
        Path(output).write_text('\n'.join(json.dumps(row) for row in rows)+'\n')
    return {'state': m.state, 'result': m.result, 'time_s': now,
            'commanded_distance_m': m.commanded_distance_m, 'zero_hold_completed': m.zero_hold_completed,
            'synthetic_ground_position_is_aircraft_validation': False,
            'transitions': [f"{e['from']}->{e['to']}" for e in m.events]}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output')
    parser.add_argument('--ascent-only', action='store_true')
    parser.add_argument('--irregular', action='store_true')
    args = parser.parse_args()
    result = run_demo(args.output, args.ascent_only, args.irregular)
    print(json.dumps(result, indent=2))
    if result['state'] != 'COMPLETE' or not result['zero_hold_completed']:
        raise SystemExit(1)
