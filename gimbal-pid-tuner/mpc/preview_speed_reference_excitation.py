#!/usr/bin/env python3
"""Reproduce and audit the frozen S1/S2 speed-reference waveform."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from yaw_speed_trace_protocol import PLAN_SHA256, speed_wave


DEFAULT_PLAN = Path(__file__).with_name('speed_reference_excitation_plan.json')


def preview(plan_path=DEFAULT_PLAN, output=None):
    raw = plan_path.read_bytes()
    if hashlib.sha256(raw).hexdigest().upper() != PLAN_SHA256:
        raise ValueError('frozen speed-reference plan hash mismatch')
    plan = json.loads(raw)
    dt = plan['sample_period_ms']/1000
    rows = []
    report = {'plan_sha256': PLAN_SHA256, 'sets': {}}
    for label, phase_set in (('S1', 0), ('S2', 1)):
        values = [[], []]
        for ms in range(0, plan['duration_ms']+1, plan['sample_period_ms']):
            refs = [0., 0.] if not 3000 <= ms < 31000 else [
                speed_wave(ms-3000, axis, phase_set) for axis in (0, 1)]
            rows.append((label, ms, *refs))
            for axis in (0, 1): values[axis].append(refs[axis])
        integrals = [sum(v)*dt for v in values]
        peaks = [max(map(abs, v)) for v in values]
        rms = [(sum(x*x for x in v)/len(v))**.5 for v in values]
        report['sets'][label] = {
            'peak_dps': peaks, 'rms_dps': rms,
            'reference_net_travel_deg': integrals,
            'positive_samples': [sum(x > .5 for x in v) for v in values],
            'negative_samples': [sum(x < -.5 for x in v) for v in values],
        }
        if peaks[0] > 30.0001 or peaks[1] > 60.0001 or any(abs(x) > .02 for x in integrals):
            raise ValueError('speed waveform violates frozen bounds')
    if output:
        output.mkdir(parents=True, exist_ok=False)
        (output/'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        with (output/'references.csv').open('x', newline='', encoding='utf-8') as f:
            writer = csv.writer(f); writer.writerow(('phase_set', 'time_ms', 'big_ref_dps', 'small_ref_dps'))
            writer.writerows(rows)
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--plan', type=Path, default=DEFAULT_PLAN)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    print(json.dumps(preview(args.plan, args.output), indent=2))


if __name__ == '__main__':
    main()
