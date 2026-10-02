#!/usr/bin/env python3
"""Fetch pinned real OpenVINS. No package installs, FC connection or flight operations."""
import json
import subprocess
from pathlib import Path
from patch_openvins import patch

ROOT=Path(__file__).resolve().parents[1]


def main():
    lock=json.loads((ROOT/"config/openvins.lock.json").read_text())
    directory=ROOT/"ros2_ws/src/open_vins"
    if not directory.exists():
        subprocess.run(["git","clone","--no-checkout",lock["repository"],str(directory)],check=True)
        subprocess.run(["git","-C",str(directory),"checkout","--detach",lock["commit"]],check=True)
    head=subprocess.check_output(["git","-C",str(directory),"rev-parse","HEAD"],text=True).strip()
    if head!=lock["commit"]:
        raise SystemExit("existing OpenVINS checkout is not pinned; preserve changes and inspect manually")
    dirty=subprocess.check_output(["git","-C",str(directory),"status","--porcelain"],text=True)
    if dirty:
        manifest=directory/"WE_MEET_PATCH.json"
        if not manifest.exists():
            raise SystemExit("existing checkout has local changes; refusing overwrite")
        import hashlib
        known=json.loads(manifest.read_text())
        for line in dirty.splitlines():
            relative=line[3:]
            if relative=="WE_MEET_PATCH.json":
                continue
            if relative not in known or hashlib.sha256((directory/relative).read_bytes()).hexdigest()!=known[relative]:
                raise SystemExit("unrecognised local OpenVINS change: "+relative)
    print("OpenVINS",head,"patched",len(patch(directory)),"files")


if __name__=="__main__":
    main()
