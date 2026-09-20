"""Audit the cable-free R1/F4/F5 amplitude ladder without hardware access."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from preview_dual_reference_cde import reference, time_grid


HERE = Path(__file__).resolve().parent
DEFAULT_PLAN = HERE/'cable_free_amplitude_ladder_plan.json'
SOURCE_PLAN = HERE/'dual_reference_excitation_plan_cde.json'
PHASES = ('R1', 'F4', 'F5')


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest().upper()


def calculate(plan, source_plan, source_path=SOURCE_PLAN):
    if (plan.get('schema_version') != 1
            or plan.get('status') != 'FROZEN_CABLE_FREE_AMPLITUDE_LADDER'
            or tuple(plan.get('phase_sets', ())) != PHASES
            or plan.get('source_phase') != 'E'):
        raise ValueError('invalid cable-free ladder plan identity')
    source_hash = sha256(source_path)
    if source_hash != plan['source_reference_plan_sha256']:
        raise ValueError('source C/D/E reference plan hash changed')
    time_s = time_grid(source_plan)
    base = reference(source_plan, 'E', time_s)
    dt = plan['sample_period_ms']/1000
    result = {
        'status': 'CABLE_FREE_LADDER_PREVIEW_PASSED',
        'hardware_takeover_allowed': False,
        'source_reference_plan_sha256': source_hash,
        'phases': {},
    }
    keys = (
        'big_peak_deg', 'heading_peak_deg', 'planar_implied_small_joint_peak_deg',
        'big_rate_peak_dps', 'heading_rate_peak_dps',
        'planar_implied_small_joint_rate_peak_dps',
        'big_acceleration_peak_dps2', 'heading_acceleration_peak_dps2',
        'planar_implied_small_joint_acceleration_peak_dps2')
    for phase in PHASES:
        scale = plan['phase_sets'][phase]['scale']
        values = base*scale
        rate = np.gradient(values, dt, axis=0, edge_order=2)
        acceleration = np.gradient(rate, dt, axis=0, edge_order=2)
        small_joint = values[:, 1]-values[:, 0]
        small_rate = rate[:, 1]-rate[:, 0]
        small_acceleration = acceleration[:, 1]-acceleration[:, 0]
        actual = dict(zip(keys, map(float, (
            np.max(np.abs(values[:, 0])), np.max(np.abs(values[:, 1])),
            np.max(np.abs(small_joint)), np.max(np.abs(rate[:, 0])),
            np.max(np.abs(rate[:, 1])), np.max(np.abs(small_rate)),
            np.max(np.abs(acceleration[:, 0])), np.max(np.abs(acceleration[:, 1])),
            np.max(np.abs(small_acceleration))))))
        declared = plan['reference_preview'][phase]
        if any(abs(actual[key]-declared[key]) > 2e-6 for key in keys):
            raise ValueError('declared preview differs for '+phase)
        result['phases'][phase] = {'scale': scale, **actual}
    guards = plan['unchanged_runtime_guards']
    f5 = result['phases']['F5']
    if (f5['big_peak_deg'] >= guards['big_relative_travel_abort_deg']
            or f5['heading_peak_deg'] >= guards['heading_relative_travel_abort_deg']
            or f5['planar_implied_small_joint_peak_deg'] >=
            guards['small_relative_travel_abort_deg']):
        raise ValueError('F5 reference has no nominal travel reserve')
    rejected = plan['rejected_10x_preview']
    if (rejected['big_peak_deg'] <= guards['big_relative_travel_abort_deg']
            or rejected['planar_implied_small_joint_peak_deg'] <=
            guards['small_relative_travel_abort_deg']):
        raise ValueError('10x rejection is not supported by travel limits')
    return result, time_s, base


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, default=DEFAULT_PLAN)
    parser.add_argument('--source-plan', type=Path, default=SOURCE_PLAN)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output must not exist')
    plan = json.loads(args.plan.read_text(encoding='utf-8'))
    source = json.loads(args.source_plan.read_text(encoding='utf-8'))
    report, time_s, base = calculate(plan, source, args.source_plan)
    args.output.mkdir(parents=True)
    (args.output/'report.json').write_text(
        json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    for phase in PHASES:
        scale = plan['phase_sets'][phase]['scale']
        np.savetxt(args.output/(phase+'.csv'), np.column_stack((time_s, base*scale)),
                   delimiter=',', comments='', fmt='%.9g',
                   header='time_s,big_reference_deg,heading_reference_deg')
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
