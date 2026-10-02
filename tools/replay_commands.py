#!/usr/bin/env python3
"""Reconstruct signed ZOH integral between actual local publish timestamps."""
import argparse
import json
import math
from pathlib import Path


def replay(path):
    total, previous, invalid = 0.0, None, 0
    last_recorded = None
    for line in Path(path).read_text().splitlines():
        row = json.loads(line)
        if row.get('kind') != 'control':
            continue
        last_recorded = row['commanded_distance_m']
        if not row['published'] or row.get('published_at_s') is None:
            continue
        if previous and previous['command'].get('integrate_distance'):
            dt = row['published_at_s']-previous['published_at_s']
            if 0 < dt <= row.get('maximum_tick_gap_s', 0.2):
                c, yaw = previous['command'], previous['yaw_ref']
                total += (c['vx']*math.cos(yaw)+c['vy']*math.sin(yaw))*dt
            else:
                invalid += 1
        previous = row
    return {'recomputed_commanded_distance_m': total, 'last_recorded_commanded_distance_m': last_recorded,
            'invalid_intervals': invalid, 'meaning': 'local command integral, not ground truth or FC execution ACK'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('commands')
    print(json.dumps(replay(parser.parse_args().commands), indent=2))
