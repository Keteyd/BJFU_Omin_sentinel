#!/usr/bin/env python3
"""Audit one S1/S2 speed-reference capture without fitting a model."""

import argparse
import csv
import json
import math
from pathlib import Path


LIMITS = dict(small_travel=20., big_rate=360., small_rate=180.,
              big_command=30., small_command=6.)


def circular_delta(value, origin, period):
    return (value-origin+period/2) % period-period/2


def rmse(values):
    return math.sqrt(sum(x*x for x in values)/len(values)) if values else None


def review(capture):
    capture = Path(capture)
    report = json.loads((capture/'report.json').read_text(encoding='utf-8'))
    with (capture/'samples.csv').open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError('capture has no samples')
    f = lambda row, key: float(row[key])
    first = rows[0]
    big_motion = [circular_delta(f(r, 'big_raw'), f(first, 'big_raw'), 8192)*360/8192 for r in rows]
    small_motion = [circular_delta(f(r, 'small_raw'), f(first, 'small_raw'), 8192)*360/8192 for r in rows]
    heading_motion = [circular_delta(f(r, 'yaw_deg'), f(first, 'yaw_deg'), 360) for r in rows]
    big_command = [f(r, 'big_mean_attempted_command_raw')/1000 for r in rows]
    small_command = [f(r, 'small_mean_attempted_command_raw')/1000 for r in rows]
    big_rate = [f(r, 'big_feedback_rpm')*6 for r in rows]
    small_rate = [f(r, 'gyro_z_rad_s')*180/math.pi for r in rows]
    active = [i for i, r in enumerate(rows) if int(r['phase']) == 3]
    big_ref = [f(rows[i], 'big_reference_offset_cdeg')/100 for i in active]
    small_ref = [f(rows[i], 'small_heading_reference_offset_cdeg')/100 for i in active]
    phase = next((name for name, value in (('S1', 0), ('S2', 1))
                  if (report.get('profile') or {}).get('reverse') == value), None)
    terminal = report.get('last_status') or {}
    structural_ok = bool(report.get('download_complete') and not report.get('quality_issues')
        and report.get('crc_errors') == 0 and terminal.get('phase') == 5
        and terminal.get('reason') == 0 and not report.get('firmware_stream_quality_latched')
        and report.get('big_stop_reason_name') in (None, 'none'))
    maximum = lambda values: max(map(abs, values))
    maxima = dict(big_motion_deg=maximum(big_motion), small_motion_deg=maximum(small_motion),
                  heading_motion_deg=maximum(heading_motion), big_rate_dps=maximum(big_rate),
                  small_inertial_rate_dps=maximum(small_rate),
                  big_command_software_units=maximum(big_command),
                  small_command_software_units=maximum(small_command))
    fractions = {key+'_fraction': maxima[name]/LIMITS[key] for key, name in (
        ('small_travel', 'small_motion_deg'), ('big_rate', 'big_rate_dps'),
        ('small_rate', 'small_inertial_rate_dps'), ('big_command', 'big_command_software_units'),
        ('small_command', 'small_command_software_units'))}
    return {
        'capture': str(capture.resolve()), 'phase_set': phase, 'records': len(rows),
        'structural_quality_passed': structural_ok,
        'maximum_abs': maxima, 'fraction_of_runtime_bound': fractions,
        'disabled_speed_profile_travel_stops': ['big_motion_deg', 'heading_motion_deg'],
        'active_speed_tracking': {
            'big_rmse_dps': rmse([big_rate[i]-r for i, r in zip(active, big_ref)]),
            'small_rmse_dps': rmse([small_rate[i]-r for i, r in zip(active, small_ref)]),
            'big_reference_peak_dps': maximum(big_ref),
            'small_reference_peak_dps': maximum(small_ref),
        },
        'decision': ('FIT_AND_FREEZE_MODEL_BEFORE_S2' if phase == 'S1' and structural_ok else
                     'INDEPENDENT_VALIDATION_READY_FOR_OFFLINE_REVIEW' if phase == 'S2' and structural_ok else
                     'STOP_AND_REVIEW_CAPTURE'),
        'hardware_takeover_allowed': False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = review(args.capture)
    text = json.dumps(result, indent=2, allow_nan=False)+'\n'
    if args.output: args.output.write_text(text, encoding='utf-8')
    print(text, end='')
    return 0 if result['structural_quality_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
