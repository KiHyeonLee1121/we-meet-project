#!/usr/bin/env python3
"""Run the shared camera-free synthetic trial."""
import argparse
import json
from core_offline_demo import run_demo

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
