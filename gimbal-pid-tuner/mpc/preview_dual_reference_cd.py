"""Preview the proposed C/D reversal-plus-multisine dual-reference signal.

Offline only.  The JSON is a design artifact, not a firmware configuration or
permission to move hardware.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


DEFAULT_PLAN = Path(__file__).with_name('dual_reference_excitation_plan_cd.json')
PHASES = ('C', 'D')
AXES = ('big', 'small')


def validate(plan):
    if (plan.get('schema_version') != 1
            or plan.get('status') != 'DRAFT_OFFLINE_ONLY_NOT_FIRMWARE_CONFIGURATION'
            or plan.get('hardware_execution_authorized') is not False):
        raise ValueError('C/D preview requires an unauthorized offline draft')
    period, duration = plan['sample_period_ms'], plan['duration_ms']
    segments = plan['segments_ms']
    if (type(period) is not int or period <= 0 or type(duration) is not int
            or duration <= 0 or duration % period
            or sum(segments.values()) != duration):
        raise ValueError('invalid C/D sample or segment duration')
    probe = plan['reversal_probe']
    if (probe['transition'] != 'raised_cosine_from_previous_level'
            or probe['level_period_ms'] % period
            or probe['transition_ms'] % period
            or not 0 < probe['transition_ms'] < probe['level_period_ms']
            or segments['reversal_probe'] % probe['level_period_ms']):
        raise ValueError('invalid reversal probe timing')
    count = segments['reversal_probe']//probe['level_period_ms']
    for phase in PHASES:
        if set(probe['levels'][phase]) != {'big_deg', 'small_deg'}:
            raise ValueError('invalid reversal axes')
        for axis in AXES:
            levels = probe['levels'][phase][axis+'_deg']
            if len(levels) != count or levels[0] or levels[-1]:
                raise ValueError('reversal levels must span the segment and start/end at zero')
    multisine = plan['multisine']
    if (multisine['window'] != 'sin_squared_over_multisine_segment'
            or set(multisine['phase_sets_rad']) != set(PHASES)):
        raise ValueError('invalid multisine window or phases')
    harmonic_sets = []
    for axis in AXES:
        item = multisine['axes'][axis]
        harmonics = item['harmonics_per_22s']
        amplitudes = item['component_amplitudes_deg']
        harmonic_sets.append(set(harmonics))
        if (len(harmonics) != len(amplitudes) or not harmonics
                or any(type(value) is not int or value <= 0 for value in harmonics)
                or len(set(harmonics)) != len(harmonics)
                or any(value <= 0 for value in amplitudes)
                or sum(amplitudes) > item['peak_budget_deg']):
            raise ValueError('invalid multisine axis '+axis)
        for phase in PHASES:
            if len(multisine['phase_sets_rad'][phase][axis]) != len(harmonics):
                raise ValueError('phase count differs from harmonic count')
    if harmonic_sets[0] & harmonic_sets[1]:
        raise ValueError('axis harmonics must be disjoint')
    wire = plan['wire_assumptions']
    if (wire['nominal_records'] != duration//period+1
            or wire['trace_version_must_exceed'] < 4
            or wire['frame_bytes'] != 163 or wire['record_bytes'] != 152):
        raise ValueError('invalid wire assumptions')
    gates = plan['offline_acceptance_gates']
    if (gates['hardware_takeover_allowed'] is not False
            or gates['D_role'].find('no coefficient') < 0):
        raise ValueError('D must remain independent and hardware-disabled')


def time_grid(plan):
    validate(plan)
    return np.arange(0, plan['duration_ms']+plan['sample_period_ms'],
                     plan['sample_period_ms'], dtype=float)/1000


def _raised_probe(plan, phase, t):
    segments = plan['segments_ms']
    start = segments['baseline']/1000
    duration = segments['reversal_probe']/1000
    local = t-start
    active = (local >= 0) & (local < duration)
    result = np.zeros((len(t), 2))
    level_period = plan['reversal_probe']['level_period_ms']/1000
    transition = plan['reversal_probe']['transition_ms']/1000
    index = np.floor(local[active]/level_period).astype(int)
    within = local[active]-index*level_period
    blend = np.ones_like(within)
    ramping = within < transition
    blend[ramping] = .5-.5*np.cos(np.pi*within[ramping]/transition)
    for column, axis in enumerate(AXES):
        levels = np.asarray(plan['reversal_probe']['levels'][phase][axis+'_deg'])
        previous = levels[np.maximum(index-1, 0)]
        current = levels[index]
        result[active, column] = previous+(current-previous)*blend
    return result


def reference(plan, phase, time_s):
    validate(plan)
    if phase not in PHASES:
        raise ValueError('unknown phase set')
    t = np.asarray(time_s, dtype=float)
    if (t.ndim != 1 or not np.isfinite(t).all() or np.any(t < 0)
            or np.any(t > plan['duration_ms']/1000)):
        raise ValueError('time outside C/D profile')
    result = _raised_probe(plan, phase, t)
    segments = plan['segments_ms']
    start = (segments['baseline']+segments['reversal_probe'])/1000
    duration = segments['multisine']/1000
    local = t-start
    active = (local > 0) & (local < duration)
    window = np.zeros_like(t)
    window[active] = np.sin(np.pi*local[active]/duration)**2
    for column, axis in enumerate(AXES):
        item = plan['multisine']['axes'][axis]
        phases = plan['multisine']['phase_sets_rad'][phase][axis]
        values = np.zeros_like(t)
        for harmonic, amplitude, angle in zip(
                item['harmonics_per_22s'], item['component_amplitudes_deg'], phases):
            values[active] += amplitude*np.sin(
                2*np.pi*harmonic*local[active]/duration+angle
            )
        result[:, column] += values*window
    return result


def _derivatives(values, dt):
    velocity = np.gradient(values, dt, axis=0, edge_order=2)
    return velocity, np.gradient(velocity, dt, axis=0, edge_order=2)


def audit(plan):
    validate(plan)
    t = time_grid(plan)
    dt = plan['sample_period_ms']/1000
    segments = plan['segments_ms']
    multisine_start = (segments['baseline']+segments['reversal_probe'])//plan['sample_period_ms']
    multisine_end = multisine_start+segments['multisine']//plan['sample_period_ms']+1
    active_start = segments['baseline']//plan['sample_period_ms']
    active_end = (plan['duration_ms']-segments['settle'])//plan['sample_period_ms']+1
    result = {
        'status': 'FROZEN_C_D_REFERENCE_PREVIEW',
        'hardware_execution_authorized': False,
        'firmware_ready': True,
        'field_execution_allowed': False,
        'sample_period_ms': plan['sample_period_ms'],
        'duration_ms': plan['duration_ms'],
        'nominal_records': len(t),
        'wire_bytes': len(t)*plan['wire_assumptions']['frame_bytes'],
        'sample_uart_8n1_utilization': (
            10*1000/plan['sample_period_ms']*plan['wire_assumptions']['frame_bytes']
            /plan['wire_assumptions']['uart_baud']
        ),
        'development_structure': plan['development_structure_frozen_before_C'],
        'phase_sets': {},
        'required_before_execution': plan['required_before_execution'],
        'warnings': [
            'Reference preview is not a measured motion, effort or collision simulation.',
            'A/B were used to choose this design and cannot validate its final model.',
            'Firmware and host implementation passed offline release checks; deploy and flash before field use.',
            'MPC hardware takeover remains disabled.',
        ],
    }
    generated = {}
    for phase in PHASES:
        refs = reference(plan, phase, t)
        velocity, acceleration = _derivatives(refs, dt)
        active = refs[active_start:active_end]
        multi = refs[multisine_start:multisine_end]
        result['phase_sets'][phase] = {
            'active_cross_axis_correlation': float(np.corrcoef(active.T)[0, 1]),
            'multisine_cross_axis_correlation': float(np.corrcoef(multi.T)[0, 1]),
            'axes': {
                axis: {
                    'minimum_deg': float(refs[:, column].min()),
                    'maximum_deg': float(refs[:, column].max()),
                    'peak_abs_deg': float(np.max(np.abs(refs[:, column]))),
                    'positive_fraction': float(np.mean(active[:, column] > .02)),
                    'negative_fraction': float(np.mean(active[:, column] < -.02)),
                    'near_zero_fraction': float(np.mean(np.abs(active[:, column]) <= .02)),
                    'maximum_abs_rate_dps': float(np.max(np.abs(velocity[:, column]))),
                    'maximum_abs_acceleration_dps2': float(np.max(np.abs(acceleration[:, column]))),
                }
                for column, axis in enumerate(AXES)
            },
        }
        generated[phase] = (refs, velocity, acceleration)
    result['same_axis_C_D_correlation'] = {
        axis: float(np.corrcoef(generated['C'][0][active_start:active_end, column],
                                generated['D'][0][active_start:active_end, column])[0, 1])
        for column, axis in enumerate(AXES)
    }
    limits = plan['preview_limits']
    checks = []
    for phase in PHASES:
        item = result['phase_sets'][phase]
        checks.append(abs(item['active_cross_axis_correlation'])
                      <= limits['maximum_abs_cross_axis_correlation'])
        for axis in AXES:
            stats = item['axes'][axis]
            checks += [stats['maximum_abs_rate_dps'] <= limits['maximum_reference_rate_dps'],
                       stats['maximum_abs_acceleration_dps2']
                       <= limits['maximum_reference_acceleration_dps2'],
                       stats['peak_abs_deg']
                       <= plan['multisine']['axes'][axis]['peak_budget_deg']]
    checks += [abs(value) <= limits['maximum_abs_same_axis_C_D_correlation']
               for value in result['same_axis_C_D_correlation'].values()]
    result['preview_limits_passed'] = bool(all(checks))
    result['_generated'] = generated
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, default=DEFAULT_PLAN)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output must not exist')
    plan = json.loads(args.plan.read_text(encoding='utf-8'))
    report = audit(plan)
    generated = report.pop('_generated')
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'plan.json').write_text(
        json.dumps(plan, indent=2, allow_nan=False)+'\n', encoding='utf-8'
    )
    (args.output/'review.json').write_text(
        json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8'
    )
    t = time_grid(plan)
    for phase in PHASES:
        refs, velocity, acceleration = generated[phase]
        np.savetxt(
            args.output/('phase_'+phase+'.csv'),
            np.column_stack((t, refs, velocity, acceleration)), delimiter=',',
            comments='', fmt='%.9g',
            header=('time_s,big_reference_deg,small_reference_deg,'
                    'big_reference_rate_dps,small_reference_rate_dps,'
                    'big_reference_acceleration_dps2,small_reference_acceleration_dps2'),
        )
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report['preview_limits_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
