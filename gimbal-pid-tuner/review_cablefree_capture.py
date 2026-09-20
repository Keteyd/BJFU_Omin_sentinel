#!/usr/bin/env python3
"""Summarize one R1/F4/F5 capture and decide whether F5 may be attempted."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


BIG_TRAVEL, SMALL_TRAVEL, HEADING_TRAVEL = 25., 20., 20.
BIG_RATE, SMALL_RATE = 360., 180.
BIG_COMMAND, SMALL_COMMAND = 30., 6.


def circular_delta(value, origin, period):
    return (value-origin+period/2) % period-period/2


def review(capture):
    capture = Path(capture)
    report = json.loads((capture/'report.json').read_text(encoding='utf-8'))
    with (capture/'samples.csv').open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError('capture has no samples')
    numeric = lambda row, key: float(row[key])
    first = rows[0]
    big0, small0, heading0 = (numeric(first, 'big_raw'), numeric(first, 'small_raw'),
                              numeric(first, 'yaw_deg'))
    big_motion = [circular_delta(numeric(row, 'big_raw'), big0, 8192)*360/8192
                  for row in rows]
    small_motion = [circular_delta(numeric(row, 'small_raw'), small0, 8192)*360/8192
                    for row in rows]
    heading_motion = [circular_delta(numeric(row, 'yaw_deg'), heading0, 360)
                      for row in rows]
    big_command = [numeric(row, 'big_mean_attempted_command_raw')/1000 for row in rows]
    small_command = [numeric(row, 'small_mean_attempted_command_raw')/1000 for row in rows]
    big_rate = [abs(numeric(row, 'big_feedback_rpm')*6) for row in rows]
    small_rate = [abs(numeric(row, 'small_feedback_rpm')*6) for row in rows]
    maximum = lambda values: max(map(abs, values))
    terminal = report.get('last_status') or {}
    structural_ok = bool(
        report.get('download_complete')
        and not report.get('quality_issues')
        and report.get('crc_errors') == 0
        and terminal.get('phase') == 5 and terminal.get('reason') == 0
        and not report.get('firmware_stream_quality_latched')
        and report.get('big_stop_reason_name') in (None, 'none'))
    margins = {
        'big_travel_fraction': maximum(big_motion)/BIG_TRAVEL,
        'small_travel_fraction': maximum(small_motion)/SMALL_TRAVEL,
        'heading_travel_fraction': maximum(heading_motion)/HEADING_TRAVEL,
        'big_rate_fraction': maximum(big_rate)/BIG_RATE,
        'small_rate_fraction': maximum(small_rate)/SMALL_RATE,
        'big_command_fraction': maximum(big_command)/BIG_COMMAND,
        'small_command_fraction': maximum(small_command)/SMALL_COMMAND,
    }
    finite = all(math.isfinite(value) for values in
                 (big_motion, small_motion, heading_motion, big_command,
                  small_command, big_rate, small_rate) for value in values)
    # F5 is only 25% above F4.  Requiring every F4 quantity below 80% of its
    # runtime bound gives the linear extrapolation nominal room to reach F5.
    f5_allowed = bool(report.get('ladder_phase_name') == 'F4'
                      and structural_ok and finite and
                      all(value < .8 for value in margins.values()))
    return {
        'capture': str(capture.resolve()),
        'ladder_phase': report.get('ladder_phase_name'),
        'records': len(rows),
        'structural_quality_passed': structural_ok,
        'maximum_abs': {
            'big_motion_deg': maximum(big_motion),
            'small_motion_deg': maximum(small_motion),
            'heading_motion_deg': maximum(heading_motion),
            'big_rate_dps_from_feedback_rpm': maximum(big_rate),
            'small_rate_dps_from_feedback_rpm': maximum(small_rate),
            'big_command_software_units': maximum(big_command),
            'small_command_software_units': maximum(small_command),
        },
        'fraction_of_runtime_bound': margins,
        'f5_allowed_after_F4': f5_allowed,
        'decision': ('F5_OPTIONAL_ALLOWED' if f5_allowed else
                     'STOP_BEFORE_F5_AND_REVIEW'),
        'hardware_takeover_allowed': False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = review(args.capture)
    text = json.dumps(result, indent=2, allow_nan=False)+'\n'
    if args.output:
        args.output.write_text(text, encoding='utf-8')
    print(text, end='')
    return 0 if result['structural_quality_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
