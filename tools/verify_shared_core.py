#!/usr/bin/env python3
"""Verify checked-in core bytes; optionally compare two downloaded branches."""
import argparse
import hashlib
import json
from pathlib import Path


def core_files(root):
    directory = root/'ros2_ws/src/we_meet_flight_core'
    return {str(p.relative_to(directory)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(directory.rglob('*'))
            if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc'}


def verify(root, other=None):
    actual = core_files(root)
    expected = json.loads((root/'docs/shared_core_manifest.json').read_text())['files_sha256']
    if actual != expected:
        raise ValueError('shared core differs from manifest; review and sync both branches')
    if other is not None and actual != core_files(other):
        raise ValueError('downloaded branches contain different shared core files')
    identity = hashlib.sha256(json.dumps(actual, sort_keys=True).encode()).hexdigest()
    return {'core_version': '0.2.0', 'files': len(actual), 'sha256': identity,
            'other_branch_compared': other is not None}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--other', type=Path)
    args = parser.parse_args()
    print(json.dumps(verify(Path(__file__).resolve().parents[1], args.other), indent=2))
