#!/usr/bin/env python3
"""Recompute signed approach command integral from commands.jsonl. No FC IO."""
import argparse
import json
import math
from pathlib import Path


def replay(path):
    total, previous = 0.0, None
    for line in Path(path).read_text().splitlines():
        row = json.loads(line)
        if row.get('kind') != 'control':
            continue
        if previous and previous.get('integrate_distance') and previous['published']:
            dt = row['time_s']-previous['time_s']
            limit = previous.get('maximum_tick_gap_s', 0.2)
            if 0 < dt <= limit:
                c, yaw = previous['command'], previous['yaw_ref']
                total += (c['vx']*math.cos(yaw)+c['vy']*math.sin(yaw))*dt
        previous = row
    return {'recomputed_commanded_distance_m': total,
            'last_recorded_commanded_distance_m': previous['commanded_distance_m'] if previous else None,
            'meaning': 'local published command integral, not physical displacement or FC ACK'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('commands')
    args = parser.parse_args()
    print(json.dumps(replay(args.commands), indent=2))
