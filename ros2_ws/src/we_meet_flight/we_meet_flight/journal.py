"""Bounded async per-run evidence files. Never turn missing values into zero."""
import csv
import json
import math
from pathlib import Path
import queue
import threading
import time
import yaml


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


class Journal:
    def __init__(self, directory, metadata):
        root = Path(directory).expanduser()
        self.run_id = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()) + f'_{time.time_ns()}'
        self.directory = root / self.run_id
        self.directory.mkdir(parents=True, exist_ok=False)
        self.path = self.directory / 'all.jsonl'
        metadata = json_safe({'run_id': self.run_id, 'utc_start': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                              'px4_version_live': None, 'ulog_recording_live': None, **metadata})
        (self.directory/'run_metadata.json').write_text(json.dumps(metadata, indent=2, allow_nan=False))
        (self.directory/'config_snapshot.yaml').write_text(yaml.safe_dump(metadata.get('config', {})))
        (self.directory/'field_result.md').write_text(
            '# 현장 측정 (명령 적분 및 FC 추정과 별도)\n\n'
            f'- run_id: {self.run_id}\n- ULog 원본 파일명/해시:\n- 현장 시작 UTC:\n'
            '- 목표까지 줄자로 잰 거리:\n- 착륙 전방/좌우 오차:\n- 5초 대기 중 이동 범위:\n'
            '- 카메라 방향/패널 크기/배치:\n- 바람/관찰자/비고:\n')
        self.file = self.path.open('x', buffering=1)
        self.commands = (self.directory/'commands.jsonl').open('x', buffering=1)
        self.events = (self.directory/'events.jsonl').open('x', buffering=1)
        self.console = (self.directory/'console.log').open('x', buffering=1)
        self.csv_file = (self.directory/'telemetry.csv').open('x', buffering=1, newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['utc', 'monotonic_s', 'state', 'mode', 'armed', 'range_m', 'yaw',
                           'vx', 'vy', 'vz', 'yaw_rate', 'commanded_distance_m', 'sensors_json', 'telemetry_json'])
        self.queue = queue.Queue(maxsize=4096)
        self.error = ''
        self.closed = False
        self.last = None
        self.stage_times = {}
        self.max_speed = self.max_dt = 0.0
        self.file.write(json.dumps({'kind': 'metadata', **metadata}, allow_nan=False) + '\n')
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()

    def append(self, row):
        row = json_safe({'utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()), **row})
        try:
            self.queue.put_nowait(row)
        except queue.Full:
            self.error = 'log queue full'

    def _write(self):
        try:
            while True:
                row = self.queue.get()
                if row is None:
                    break
                line = json.dumps(row, allow_nan=False) + '\n'
                self.file.write(line)
                if row['kind'] == 'control':
                    self.last = row
                    cmd, s = row['command'], row['sensors']
                    self.commands.write(line)
                    self.csv.writerow([row['utc'], row['time_s'], row['state'], s['mode'], s['armed'],
                                       s['range_m'], s['yaw'], cmd['vx'], cmd['vy'], cmd['vz'], cmd['yaw_rate'],
                                       row['commanded_distance_m'], json.dumps(s), json.dumps(row['telemetry'])])
                    self.stage_times[row['state']] = self.stage_times.get(row['state'], 0)+row['dt_s']
                    self.max_dt = max(self.max_dt, row['dt_s'])
                    self.max_speed = max(self.max_speed, math.hypot(cmd['vx'], cmd['vy']))
                else:
                    self.events.write(line)
                    if row['kind'] in {'transition', 'exception', 'vision_error', 'start_rejected'}:
                        self.console.write(line)
        except Exception as exc:
            self.error = f'log write failed: {exc}'
        finally:
            last = self.last or {}
            summary = {'run_id': self.run_id, 'state': last.get('state', 'not_started'),
                       'result': last.get('result', 'not_started'), 'reason': last.get('reason'),
                       'commanded_distance_m': last.get('commanded_distance_m'),
                       'stage_times_s_approx': self.stage_times, 'max_command_speed_mps': self.max_speed,
                       'max_tick_dt_s': self.max_dt, 'log_error': self.error or None,
                       'five_second_center_hold_completed': last.get('result') == 'panel_centered_5s',
                       'estimated_displacement_is_ground_truth': False,
                       'landing_confirmed': last.get('state') == 'DONE'}
            try:
                (self.directory/'summary.json').write_text(json.dumps(json_safe(summary), indent=2, allow_nan=False))
            except Exception as exc:
                self.error = f'summary write failed: {exc}'
            for f in [self.file, self.commands, self.events, self.console, self.csv_file]:
                f.close()

    def close(self):
        if not self.closed:
            self.closed = True
            try:
                self.queue.put(None, timeout=0.5)
            except queue.Full:
                self.error = 'log queue did not drain on shutdown'
            self.thread.join(timeout=1.0)
