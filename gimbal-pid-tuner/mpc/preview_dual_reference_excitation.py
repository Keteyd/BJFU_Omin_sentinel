"""Preview the proposed dual-reference closed-loop identification signal.

Offline only: this module has no serial or hardware-control imports.
"""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from identify_closed_loop_offline import fir_features


DEFAULT_PLAN = Path(__file__).with_name('dual_reference_excitation_plan.json')
DEFAULT_MODEL = Path(__file__).resolve().parents[2] / 'NoMachineTemp' / 'closed-loop-model-20260910-v5' / 'report.json'


def validate(plan):
    if (plan.get('schema_version') != 1 or plan.get('hardware_execution_authorized') is not False
            or plan.get('status') != 'DRAFT_OFFLINE_ONLY_NOT_FIRMWARE_CONFIGURATION'):
        raise ValueError('preview requires an unauthorized draft plan')
    period = plan['sample_period_ms']
    duration = plan['duration_ms']
    spans = plan['baseline_ms'] + plan['excitation_ms'] + plan['settle_ms']
    if (type(period) is not int or period <= 0 or type(duration) is not int
            or duration <= 0 or duration != spans or duration % period):
        raise ValueError('invalid sampling schedule')
    if plan['window'] != 'sin_squared_over_excitation':
        raise ValueError('unsupported excitation window')
    phase_sets = plan['phase_sets_rad']
    if set(phase_sets) != {'A', 'B'}:
        raise ValueError('independent phase sets A and B are required')
    for axis in ('big', 'small'):
        item = plan['axes'][axis]
        harmonics = item['harmonics_per_14s']
        amplitudes = item['component_amplitudes_deg']
        if (not harmonics or len(harmonics) != len(amplitudes)
                or any(type(h) is not int or h <= 0 for h in harmonics)
                or len(set(harmonics)) != len(harmonics)
                or any(not math.isfinite(a) or a <= 0 for a in amplitudes)
                or not math.isclose(sum(amplitudes), item['peak_budget_deg'], abs_tol=1e-12)):
            raise ValueError('invalid frequency or amplitude budget for '+axis)
        for phase in phase_sets.values():
            values = phase[axis]
            if len(values) != len(harmonics) or not all(math.isfinite(v) for v in values):
                raise ValueError('invalid phases for '+axis)
    if set(plan['axes']['big']['harmonics_per_14s']) & set(plan['axes']['small']['harmonics_per_14s']):
        raise ValueError('axis frequency sets must be disjoint')
    if (plan['existing_record_bytes'] != 152 or plan['existing_frame_bytes'] != 163
            or plan['proposed_trace_version'] <= 3):
        raise ValueError('unexpected trace layout/version proposal')


def time_grid(plan):
    return np.arange(0, plan['duration_ms']+plan['sample_period_ms'],
                     plan['sample_period_ms'], dtype=float)/1000


def reference(plan, phase_set, time_s):
    """Return N-by-2 [big, small] scheduled offsets in degrees."""
    validate(plan)
    if phase_set not in plan['phase_sets_rad']:
        raise ValueError('unknown phase set')
    t = np.asarray(time_s, dtype=float)
    if t.ndim != 1 or not np.isfinite(t).all() or np.any(t < 0) or np.any(t > plan['duration_ms']/1000):
        raise ValueError('time outside profile')
    start = plan['baseline_ms']/1000
    drive = plan['excitation_ms']/1000
    u = t-start
    active = (u > 0) & (u < drive)
    window = np.zeros_like(t)
    window[active] = np.sin(np.pi*u[active]/drive)**2
    result = np.zeros((len(t), 2))
    for column, axis in enumerate(('big', 'small')):
        item = plan['axes'][axis]
        phases = plan['phase_sets_rad'][phase_set][axis]
        for harmonic, amplitude, phase in zip(item['harmonics_per_14s'],
                                               item['component_amplitudes_deg'], phases):
            result[active, column] += amplitude*np.sin(2*np.pi*harmonic*u[active]/drive+phase)
        result[:, column] *= window
    return result


def derivatives(values, dt):
    return np.gradient(values, dt, axis=0, edge_order=2), np.gradient(
        np.gradient(values, dt, axis=0, edge_order=2), dt, axis=0, edge_order=2)


def _axis_stats(values, rate, acceleration, budget):
    threshold = max(0.05, budget*.02)
    return {
        'minimum_deg': float(values.min()),
        'maximum_deg': float(values.max()),
        'peak_abs_deg': float(np.max(np.abs(values))),
        'peak_budget_deg': float(budget),
        'maximum_abs_rate_dps': float(np.max(np.abs(rate))),
        'maximum_abs_acceleration_dps2': float(np.max(np.abs(acceleration))),
        'positive_fraction_above_threshold': float(np.mean(values > threshold)),
        'negative_fraction_above_threshold': float(np.mean(values < -threshold)),
        'near_zero_fraction': float(np.mean(np.abs(values) <= threshold)),
        'zero_crossings': int(np.sum(values[1:]*values[:-1] < 0)),
    }


def audit(plan, model=None):
    validate(plan)
    t = time_grid(plan)
    dt = plan['sample_period_ms']/1000
    begin = plan['baseline_ms']//plan['sample_period_ms']
    end = (plan['baseline_ms']+plan['excitation_ms'])//plan['sample_period_ms']+1
    result = {
        'status': 'DRAFT_REFERENCE_PREVIEW_ONLY',
        'hardware_execution_authorized': False,
        'firmware_ready': False,
        'field_execution_allowed': False,
        'sample_period_ms': plan['sample_period_ms'],
        'records_per_trial': len(t),
        'record_bytes': plan['existing_record_bytes'],
        'frame_bytes': plan['existing_frame_bytes'],
        'sample_wire_bytes_per_second': 1000/plan['sample_period_ms']*plan['existing_frame_bytes'],
        'sample_uart_8n1_utilization': 10*1000/plan['sample_period_ms']*plan['existing_frame_bytes']/plan['uart_baud'],
        'phase_sets': {},
        'record_layout_proposal': plan['proposed_record_reuse'],
        'blockers': plan['required_before_execution'],
        'warnings': [
            'The reference preview is not a measured-motion or collision simulation.',
            'The FIR projection describes only the previously measured PID closed loop and is extrapolated in frequency.',
            'Feedback current remains an uncalibrated current proxy, not motor torque.',
            'MPC hardware takeover remains disabled.'
        ]
    }
    coefficient = memory = delay = None
    if model is not None:
        closed = model['closed_loop_reference_model']
        coefficient = np.asarray(closed['coefficient'], dtype=float)
        memory, delay = closed['memory_samples'], closed['delay_samples']
    phase_predictions = {}
    for phase_set in ('A', 'B'):
        refs = reference(plan, phase_set, t)
        rate, acceleration = derivatives(refs, dt)
        active_refs = refs[begin:end]
        phase_report = {
            'reference_correlation_during_excitation': float(np.corrcoef(active_refs.T)[0, 1]),
            'axes': {axis: _axis_stats(refs[:, i], rate[:, i], acceleration[:, i],
                                       plan['axes'][axis]['peak_budget_deg'])
                     for i, axis in enumerate(('big', 'small'))}
        }
        if coefficient is not None:
            prediction = fir_features(refs, memory, delay)@coefficient
            phase_predictions[phase_set] = prediction
            phase_report['existing_closed_loop_fir_projection'] = {
                name: {'minimum_deg': float(prediction[:, i].min()),
                       'maximum_deg': float(prediction[:, i].max()),
                       'peak_abs_deg': float(np.max(np.abs(prediction[:, i])))}
                for i, name in enumerate(('big_joint_deg', 'small_joint_deg', 'heading_deg'))
            }
        result['phase_sets'][phase_set] = phase_report
    refs_a = reference(plan, 'A', t)[begin:end]
    refs_b = reference(plan, 'B', t)[begin:end]
    result['repeat_similarity'] = {
        axis: float(np.corrcoef(refs_a[:, i], refs_b[:, i])[0, 1])
        for i, axis in enumerate(('big', 'small'))
    }
    result['_generated'] = {'time': t, 'predictions': phase_predictions}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, default=DEFAULT_PLAN)
    parser.add_argument('--closed-loop-report', type=Path, default=DEFAULT_MODEL)
    parser.add_argument('--without-model', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    plan = json.loads(args.plan.read_text(encoding='utf-8'))
    model = None if args.without_model else json.loads(args.closed_loop_report.read_text(encoding='utf-8'))
    report = audit(plan, model)
    generated = report.pop('_generated')
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'plan.json').write_text(json.dumps(plan, indent=2, allow_nan=False), encoding='utf-8')
    (args.output/'review.json').write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
    t = generated['time']
    for phase_set in ('A', 'B'):
        refs = reference(plan, phase_set, t)
        rate, acceleration = derivatives(refs, plan['sample_period_ms']/1000)
        prediction = generated['predictions'].get(phase_set)
        with (args.output/f'phase_{phase_set}.csv').open('w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            header = ['time_s', 'big_reference_deg', 'small_reference_deg',
                      'big_reference_rate_dps', 'small_reference_rate_dps',
                      'big_reference_accel_dps2', 'small_reference_accel_dps2']
            if prediction is not None:
                header += ['projected_big_joint_deg', 'projected_small_joint_deg', 'projected_heading_deg']
            writer.writerow(header)
            for i in range(len(t)):
                row = [t[i], *refs[i], *rate[i], *acceleration[i]]
                if prediction is not None:
                    row += list(prediction[i])
                writer.writerow(row)
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
