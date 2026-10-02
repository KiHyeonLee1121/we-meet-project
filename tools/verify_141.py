#!/usr/bin/env python3
"""Read-only checks against archived 141 facts; no ROS, sockets or flight output."""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
for name in ('da_daka_control', 'we_meet_prototype141'):
    sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / name))

from da_daka_control.survey_planner import rectangular_scan_waypoints
from da_daka_control.panel_mapping import PanelTarget
from we_meet_prototype141.history import LegacyHomeAnchor, historical_panel_order


def require(condition, description):
    if not condition:
        raise ValueError(description)


def verify_sources():
    manifest = json.loads((ROOT / 'evidence/archive_source_manifest.json').read_text())
    for entry in manifest['files']:
        actual = hashlib.sha256((ROOT / entry['path']).read_bytes()).hexdigest()
        require(actual == entry['source_sha256'], f"archive source changed: {entry['path']}")
    return len(manifest['files'])


def verify_geometry(ref):
    scan = ref['scan']
    generated = rectangular_scan_waypoints(
        *scan['launch_enu_xy'], scan['width_m'], scan['depth_m'],
        yaw_rad=math.radians(scan['yaw_enu_deg']),
        forward_offset_m=scan['forward_offset_m'],
        lateral_offset_m=scan['lateral_offset_m'])
    require(len(generated) == len(scan['waypoints_enu_xy']) == 5, 'five scan goals required')
    errors = [math.dist(a, b) for a, b in zip(generated, scan['waypoints_enu_xy'])]
    require(max(errors) < 1e-6, '141 scan geometry mismatch')
    request = ref['request_reference_from_field_document']
    anchor = LegacyHomeAnchor(request['enu_xy'], request['home_enu_xy'])
    home_error = math.dist(anchor.target_xy(ref['home_xy_events'][-1]['enu_xy']),
                           scan['launch_enu_xy'])
    require(home_error < .001, 'rounded field-document Home anchor mismatch')
    targets = [PanelTarget(**p) for p in ref['panels_from_field_document']]
    order = historical_panel_order(scan['launch_enu_xy'], targets, scan['launch_enu_xy'])
    require(order == (3, 1, 2, 6), 'recorded panel order mismatch')
    return {'waypoint_max_error_m': max(errors), 'rounded_home_error_m': home_error,
            'recorded_route': order}


def verify_ulog(path, ref):
    # Parser copied from the same supplied archive, with its BSD license.
    import numpy as np
    require(hashlib.sha256(path.read_bytes()).hexdigest() == ref['source_sha256'],
            'ULog SHA256 is not the supplied original flight 141')
    spec = importlib.util.spec_from_file_location('archive_pyulog', ROOT / 'tools/vendor/pyulog/core.py')
    core = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(core)
    ulog = core.ULog(str(path))
    status = ulog.get_dataset('vehicle_status').data
    arm = int(status['timestamp'][np.flatnonzero(status['arming_state'] == 2)[0]])
    trajectory = ulog.get_dataset('trajectory_setpoint').data
    time_s = (trajectory['timestamp'].astype(float) - arm) / 1e6
    scan = (time_s >= 22) & (time_s <= 72.125015)
    xy = np.column_stack((trajectory['position[1]'], trajectory['position[0]']))[scan]
    yaw = trajectory['yaw'][scan]
    finite = np.isfinite(xy).all(axis=1) & np.isfinite(yaw)
    xy, yaw = xy[finite], yaw[finite]
    require(len(xy) > 0, 'no finite scan setpoints')
    errors = [float(np.linalg.norm(xy - p, axis=1).min()) for p in ref['scan']['waypoints_enu_xy']]
    require(max(errors) < 1e-6, 'original FC scan goals differ from fixture')
    yaw_enu = np.pi / 2 - yaw
    expected = math.radians(ref['scan']['yaw_enu_deg'])
    yaw_error = np.arctan2(np.sin(yaw_enu - expected), np.cos(yaw_enu - expected))
    require(float(np.abs(yaw_error).max()) < 1e-6, 'original scan yaw is not fixed at fixture yaw')
    home = ulog.get_dataset('home_position').data
    for event in ref['home_xy_events']:
        index = int(np.argmin(np.abs(home['timestamp'].astype(float) - arm - event['after_arm_s']*1e6)))
        actual = (float(home['y'][index]), float(home['x'][index]))
        require(math.dist(actual, event['enu_xy']) < 1e-6, 'original Home XY mismatch')
    local = ulog.get_dataset('vehicle_local_position').data
    reset_indices = np.flatnonzero(local['heading_reset_counter'][1:] != local['heading_reset_counter'][:-1]) + 1
    expected_reset = ref['heading_reset']
    require(any(abs((int(local['timestamp'][i])-arm)/1e6-expected_reset['after_arm_s']) < .05
                and abs(math.degrees(float(local['delta_heading'][i]))-expected_reset['delta_ned_deg']) < 1e-5
                for i in reset_indices), 'original heading reset mismatch')
    # First low-altitude stationary approach matches the recorded first ID 3.
    approach = (time_s >= 96.5) & (time_s <= 97.5)
    approach_xy = np.column_stack((trajectory['position[1]'], trajectory['position[0]']))[approach]
    panel = next(p for p in ref['panels_from_field_document'] if p['panel_id'] == 3)
    panel_error = float(np.nanmin(np.linalg.norm(approach_xy - (panel['east_m'], panel['north_m']), axis=1)))
    require(panel_error < .001, 'original first panel approach mismatch')
    # Radius/period are independently visible in the recorded reacquisition.
    orbit = (time_s >= 100) & (time_s <= 116)
    orbit_t = time_s[orbit]
    orbit_xy = np.column_stack((trajectory['position[1]'], trajectory['position[0]']))[orbit]
    finite = np.isfinite(orbit_xy).all(axis=1)
    orbit_t, orbit_xy = orbit_t[finite], orbit_xy[finite]
    vectors = orbit_xy - (panel['east_m'], panel['north_m'])
    radius_error = float(np.abs(np.linalg.norm(vectors, axis=1) - .25).max())
    phase = np.unwrap(np.arctan2(vectors[:, 1], vectors[:, 0]))
    angular_rate = float(np.polyfit(orbit_t, phase, 1)[0])
    period = 2 * math.pi / angular_rate
    require(radius_error < .005 and abs(period-5.0) < .02, 'original reacquisition circle mismatch')
    require(ulog.msg_info_dict.get('ver_sw') == ref['firmware'], 'original firmware mismatch')
    parameters = json.loads((ROOT / 'evidence/px4_parameters_141.json').read_text())
    require(ulog.initial_parameters == parameters['initial_parameters'], 'original PX4 parameters mismatch')
    return {'source_sha256_verified': True, 'raw_waypoint_max_error_m': max(errors),
            'raw_yaw_max_error_rad': float(np.abs(yaw_error).max()),
            'first_panel_error_m': panel_error, 'reacquire_radius_max_error_m': radius_error,
            'reacquire_period_s': period, 'px4_parameter_count': len(ulog.initial_parameters)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ulog', type=Path, help='Optional original px4_log_00141.ulg; requires numpy')
    args = parser.parse_args()
    ref = json.loads((ROOT / 'evidence/flight141_reference.json').read_text())
    try:
        result = {'archive_sources_unchanged': verify_sources(), 'fixture': verify_geometry(ref)}
        if args.ulog:
            result['original_ulog'] = verify_ulog(args.ulog, ref)
        result['scope'] = 'offline archive/command verification; no ROS integration or real flight validation'
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except (ValueError, OSError, KeyError, IndexError, ImportError) as exc:
        print(f'FAIL: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
