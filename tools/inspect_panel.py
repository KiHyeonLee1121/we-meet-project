#!/usr/bin/env python3
"""Tune the detector on a saved camera photo/video; never connects to the FC."""
import argparse
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ros2_ws/src/we_meet_flight'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'ros2_ws/src/we_meet_flight_core'))
import cv2
from we_meet_flight.config import load_config
from we_meet_flight.vision import PanelDetector


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', help='saved image or video path')
    parser.add_argument('--config', default=str(Path(__file__).resolve().parents[1]/'ros2_ws/src/we_meet_flight/config/panel_approach.yaml'))
    parser.add_argument('--output', required=True, help='overlay image .png or video .mp4')
    args = parser.parse_args()
    config = load_config(args.config)
    detector = PanelDetector(config)
    cap = None
    frame = cv2.imread(args.input)
    is_image = frame is not None
    if not is_image:
        cap = cv2.VideoCapture(args.input)
        ok, frame = cap.read()
        if not ok:
            raise SystemExit('input image/video could not be read')
    writer = None
    count = 0
    try:
        while frame is not None:
            h, w = frame.shape[:2]
            candidates = detector.detect(frame)
            cv2.rectangle(frame, (int(w*(0.5-config.center_half_width)), int(h*(0.5-config.center_half_height))),
                          (int(w*(0.5+config.center_half_width)), int(h*(0.5+config.center_half_height))), (0, 200, 0), 2)
            for d in candidates:
                for i in range(4):
                    cv2.line(frame, d.corners[i], d.corners[(i+1)%4], (0, 200, 255), 2)
                cv2.circle(frame, (int(d.cx*w), int(d.cy*h)), 5, (0, 0, 255), -1)
            count += len(candidates)
            if is_image:
                if not cv2.imwrite(args.output, frame):
                    raise RuntimeError('overlay image write failed')
                break
            if writer is None:
                fps = cap.get(cv2.CAP_PROP_FPS) or 15
                writer = cv2.VideoWriter(args.output, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                if not writer.isOpened():
                    raise RuntimeError('overlay video writer could not open')
            writer.write(frame)
            ok, frame = cap.read()
            if not ok:
                break
    finally:
        if cap:
            cap.release()
        if writer:
            writer.release()
    print(f'candidate observations: {count}; overlay: {args.output}')


if __name__ == '__main__':
    main()
