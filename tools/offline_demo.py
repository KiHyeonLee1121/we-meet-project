#!/usr/bin/env python3
"""Synthetic plant + real OpenCV. No ROS, serial, network, arming or flight."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ros2_ws/src/we_meet_flight'))
import cv2
import numpy as np
from we_meet_flight.config import Config
from we_meet_flight.mission import Mission, Sensors
from we_meet_flight.vision import PanelDetector, TargetTracker


def run_demo(output, vision=True):
    c = Config(vision_enabled=vision)
    m = Mission(c)
    s = Sensors(estimator_valid=True, estimator_age=0, connected=True, armed=False, mode='POSCTL', landed=True,
                state_age=0, landed_age=0, yaw=0, yaw_age=0, imu_age=0,
                range_m=0.25, range_rate=0, range_age=0,
                battery_remaining=1, battery_age=0, camera_age=0)
    detector, tracker = PanelDetector(c), TargetTracker(c)
    now = 0.0
    x, y, z = 0.0, 0.0, 0.25
    m.start(now, s, 0.0)
    frames = 0
    rows = []
    for i in range(2400):
        now += 0.05
        if vision and i % 2 == 0:
            frame = np.full((480, 640, 3), 220, dtype=np.uint8)
            # A simple downward-camera planar projection; NOT aircraft dynamics.
            # Source report: footprint approximately 0.67 x 0.50 m per metre height.
            scale = 960/max(z, 0.25)
            cx, cy = 320-(0.4-y)*scale, 240-(5.0-x)*scale
            if z > 2.0:
                cv2.rectangle(frame, (round(cx-0.225*scale), round(cy-0.3*scale)),
                              (round(cx+0.225*scale), round(cy+0.3*scale)), (35, 45, 55), -1)
            frames += 1
            s.observation = tracker.update(detector.detect(frame), now, frames)
        command = m.step(now, s)
        m.acknowledge_published(command.publish)
        if command.request == 'OFFBOARD':
            s.mode = 'OFFBOARD'
        elif command.request == 'ARM':
            s.armed = True
        elif command.request == 'LAND':
            s.mode = 'AUTO.LAND'
        if command.publish:
            x += command.vx*0.05
            y += command.vy*0.05
            z += command.vz*0.05
        if s.mode == 'AUTO.LAND':
            z = max(0.25, z-0.3*0.05)
            if z <= 0.25:
                s.armed, s.landed = False, True
        elif s.armed and z > 0.3:
            s.landed = False
        previous_range = s.range_m
        s.range_m, s.range_rate = z, (z-previous_range)/0.05
        s.camera_age = now-s.observation.received_s if vision and s.observation else 0
        rows.append({'time_s': now, 'state': m.state, 'command': asdict(command),
                     'commanded_distance_m': m.commanded_distance_m,
                     'observation': asdict(s.observation) if s.observation else None,
                     'synthetic_ground_truth_xyz': [x, y, z], 'result': m.result})
        if m.state in {'DONE', 'FAILED', 'RELEASED'}:
            break
    if output:
        Path(output).write_text('\n'.join(json.dumps(row) for row in rows)+'\n')
    summary = {'result': m.result, 'state': m.state, 'seconds': round(now, 2),
               'commanded_distance_m': round(m.commanded_distance_m, 3),
               'synthetic_final_xy': [round(x, 3), round(y, 3)],
               'transitions': [f"{e['from']}->{e['to']}" for e in m.events]}
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output')
    parser.add_argument('--baseline', action='store_true')
    args = parser.parse_args()
    result = run_demo(args.output, not args.baseline)
    print(json.dumps(result, indent=2))
    if result['state'] != 'DONE' or result['result'] not in {'panel_centered_5s', 'command_distance_hover_5s'}:
        raise SystemExit(1)
