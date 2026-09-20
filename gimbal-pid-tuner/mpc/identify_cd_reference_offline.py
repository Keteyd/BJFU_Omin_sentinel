"""Audit trace-v5 C/D captures and freeze the predeclared model on C only.

C fits the fixed 12/4/0 ridge-1 closed-loop ARX coefficients.  D is evaluated
later without changing coefficients, centering, scaling, structure or gates.
This module is offline only and never imports a serial interface.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from yaw_cd_trace_protocol import BUILD, Decoder, PLAN_SHA256
from identify_can_trace_offline import (causal_resample, joint_delta,
                                        resample_integrated_input)
from identify_closed_loop_offline import first_order_signal
from identify_dual_reference_offline import (_arx_feature, _arx_simulate,
                                              _arx_spectral_radius)
from preview_dual_reference_cd import DEFAULT_PLAN, reference


DT_US = 4000
DT = DT_US / 1e6
BASELINE = 750
ACTIVE_END = 7750
NA, NB, DELAY, RIDGE = 12, 4, 0, 1.0
AXES = ('big', 'small')
OUTPUTS = ('big_joint_deg', 'small_joint_deg', 'heading_deg')
QUANTUM_DEG = 360 / 8192


def _metrics(prediction, measured):
    error = prediction - measured
    return {
        'rmse_deg': np.sqrt(np.mean(error**2, axis=0)).tolist(),
        'maximum_abs_error_deg': np.max(np.abs(error), axis=0).tolist(),
        'mean_error_deg': np.mean(error, axis=0).tolist(),
    }


def _hold_metrics(measured, start, end):
    held = np.repeat(measured[start-1:start], end-start, axis=0)
    return _metrics(held, measured[start:end])


def _raw_metrics(prediction, measured):
    error = prediction-measured
    return {
        'rmse_raw': float(np.sqrt(np.mean(error**2))),
        'maximum_abs_error_raw': float(np.max(np.abs(error))),
        'mean_error_raw': float(np.mean(error)),
    }


def fit_frozen_arx(reference_input, measured):
    """Fit exactly the structure selected from A/B; perform no grid search."""
    start, end = BASELINE, ACTIVE_END
    output_center = measured[start:end].mean(axis=0)
    output_scale = np.maximum(measured[start:end].std(axis=0), QUANTUM_DEG)
    reference_center = reference_input[start:end].mean(axis=0)
    reference_scale = np.maximum(reference_input[start:end].std(axis=0), .01)
    output = (measured-output_center)/output_scale
    refs = (reference_input-reference_center)/reference_scale
    history = max(NA, DELAY+NB-1)
    first = max(start, history)
    design = np.asarray([
        _arx_feature(output, refs, index, NA, NB, DELAY)
        for index in range(first, end)
    ])
    target = output[first:end]
    penalty = np.eye(design.shape[1])
    penalty[-1, -1] = 0
    coefficient = np.linalg.solve(
        design.T@design+len(design)*RIDGE*penalty,
        design.T@target,
    )
    one_step = design@coefficient*output_scale+output_center
    one_step_metrics = _metrics(one_step, measured[first:end])
    radius = _arx_spectral_radius(coefficient, measured.shape[1], NA)
    simulated = _arx_simulate(output, refs, coefficient, start, len(measured),
                              NA, NB, DELAY)
    finite = simulated is not None
    result = {
        'status': ('frozen_C_candidate_pending_independent_D'
                   if finite and np.isfinite(radius) and radius < 1
                   else 'rejected_nonfinite_or_unstable_C_fit'),
        'selection_source': 'A/B development analysis; no C structure search',
        'structure': {
            'output_lags': NA,
            'reference_lags': NB,
            'reference_delay_samples': DELAY,
            'reference_delay_ms': DELAY*DT_US/1000,
        },
        'ridge': RIDGE,
        'autonomous_spectral_radius': float(radius),
        'output_center_deg': output_center.tolist(),
        'output_scale_deg': output_scale.tolist(),
        'reference_center_deg': reference_center.tolist(),
        'reference_scale_deg': reference_scale.tolist(),
        'normalized_coefficient': coefficient.tolist(),
        'one_step_fit': one_step_metrics,
    }
    if finite:
        prediction = simulated*output_scale+output_center
        result['free_run'] = {}
        for name, seg_start, seg_end in (
                ('full_excitation_in_sample', start, end),
                ('settle_after_excitation', end, len(measured)),
                ('continuous_from_excitation_start', start, len(measured))):
            model = _metrics(prediction[seg_start:seg_end], measured[seg_start:seg_end])
            hold = _hold_metrics(measured, seg_start, seg_end)
            model_rmse = np.asarray(model['rmse_deg'])
            hold_rmse = np.asarray(hold['rmse_deg'])
            result['free_run'][name] = {
                'model': model,
                'hold': hold,
                'improvement_fraction':
                    (1-model_rmse/np.maximum(hold_rmse, 1e-12)).tolist(),
            }
    return result


def fit_command_current(command, current):
    """Freeze C-only effective command/current delay and gain diagnostics."""
    train = slice(BASELINE, 5000)
    select = slice(5000, ACTIVE_END)
    active = slice(BASELINE, ACTIVE_END)
    result = {}
    for column, axis in enumerate(AXES):
        target = current[:, column]
        scale = max(float(np.std(target[train])), 1.)
        candidates = []
        for delay in range(4):
            for tau_s in (0., .004, .008, .016, .032):
                filtered = first_order_signal(command[:, column]*1000, delay, tau_s)
                design = np.column_stack((filtered[train], np.ones(len(filtered[train]))))
                gain, intercept = np.linalg.lstsq(design, target[train], rcond=None)[0]
                error = gain*filtered[select]+intercept-target[select]
                candidates.append((float(np.sqrt(np.mean(error**2))/scale),
                                   delay, tau_s, filtered))
        score, delay, tau_s, filtered = min(candidates, key=lambda item: item[0])
        design = np.column_stack((filtered[active], np.ones(len(filtered[active]))))
        gain, intercept = np.linalg.lstsq(design, target[active], rcond=None)[0]
        prediction = gain*filtered+intercept
        result[axis] = {
            'status': 'C_only_frozen_diagnostic_offset_and_load_merged',
            'delay_samples': delay,
            'effective_delay_ms': delay*DT_US/1000,
            'tau_ms': tau_s*1000,
            'gain_current_raw_per_command_raw': float(gain),
            'intercept_raw_including_sensor_offset_and_load': float(intercept),
            'selection_normalized_rmse': score,
            'active_fit': _raw_metrics(prediction[active], target[active]),
        }
    return result


def load_capture(directory, plan_path=DEFAULT_PLAN):
    directory = Path(directory)
    saved = json.loads((directory/'report.json').read_text(encoding='utf-8'))
    raw = (directory/'raw.bin').read_bytes()
    decoder = Decoder(saved['trial_id'])
    for offset in range(0, len(raw), 4096):
        decoder.feed(raw[offset:offset+4096])
    verified = decoder.report()
    if (saved.get('error') or not decoder.complete or decoder.crc_errors
            or decoder.discarded_bytes or verified['quality_issues']
            or decoder.metadata['phase'] != 5 or decoder.metadata['reason']
            or decoder.profile['profile'] != 4 or decoder.profile['build'] != BUILD
            or decoder.profile['version'] != 5
            or not verified['capture_timing']['full_duration_timing_accepted']):
        raise ValueError('capture does not pass trace-v5 C/D gates: '
                         +str(verified['quality_issues']))
    if decoder.metadata != saved['metadata'] or decoder.profile != saved['profile']:
        raise ValueError('raw metadata/profile differs from saved report')
    if verified['frozen_reference_plan_sha256'] != PLAN_SHA256:
        raise ValueError('host decoder references a different frozen C/D plan')

    rows = decoder.rows
    samples = len(rows)
    trace_us = np.asarray([row['trace_us'] for row in rows], dtype=np.int64)
    interval_us = np.asarray([row['interval_us'] for row in rows], dtype=np.int64)
    areas = np.asarray([[row['big_integral_raw_us'], row['small_integral_raw_us']]
                        for row in rows], dtype=float)
    grid_us, command = resample_integrated_input(trace_us, areas, samples=samples)
    source_t = trace_us/1e6
    grid_t = (grid_us-grid_us[0])/1e6

    measured_source = np.column_stack((
        joint_delta([row['big_raw'] for row in rows]),
        joint_delta([row['small_raw'] for row in rows]),
        np.rad2deg(np.unwrap(np.deg2rad([row['yaw_deg'] for row in rows]))),
    ))
    measured_source[:, 2] -= measured_source[0, 2]
    measured = np.column_stack([
        np.interp(grid_us, trace_us, measured_source[:, column])
        for column in range(3)
    ])
    recorded_source = np.asarray([
        [row['big_reference_offset_cdeg']/100,
         row['small_heading_reference_offset_cdeg']/100]
        for row in rows
    ])
    recorded = np.column_stack([
        np.interp(grid_us, trace_us, recorded_source[:, column])
        for column in range(2)
    ])
    phase_set = 'D' if decoder.profile['reverse'] else 'C'
    plan_path = Path(plan_path)
    plan_raw = plan_path.read_bytes()
    if hashlib.sha256(plan_raw).hexdigest().upper() != PLAN_SHA256:
        raise ValueError('C/D plan SHA256 differs from the firmware release')
    plan = json.loads(plan_raw)
    expected_source = reference(
        plan, phase_set, np.minimum(source_t, plan['duration_ms']/1000)
    )
    reference_error = recorded_source-expected_source

    currents, ages = [], []
    for axis in AXES:
        sampled, age = causal_resample(
            [row[axis+'_feedback_us'] for row in rows],
            [row[axis+'_current'] for row in rows], grid_us)
        currents.append(sampled)
        ages.append(age)
    current = np.column_stack(currents)
    feedback_age = np.column_stack(ages)
    active = slice(BASELINE, ACTIVE_END)
    centered = recorded[active]-recorded[active].mean(axis=0)
    singular = np.linalg.svd(
        centered/np.maximum(centered.std(axis=0), 1e-12), compute_uv=False)
    source_area = np.sum(areas[1:], axis=0)/1000
    grid_area = np.sum(command[:-1]*DT_US, axis=0)

    audit = {
        'source': str(directory.resolve()),
        'raw_sha256': hashlib.sha256(raw).hexdigest().upper(),
        'saved_report_sha256': hashlib.sha256(
            (directory/'report.json').read_bytes()).hexdigest().upper(),
        'trial_id': decoder.metadata['id'],
        'firmware_build': '0x%08X' % decoder.metadata['build'],
        'trace_version': 5,
        'phase_set': phase_set,
        'frozen_reference_plan_sha256': PLAN_SHA256,
        'verified_quality_issues': verified['quality_issues'],
        'capture_timing': verified['capture_timing'],
        'reference_round_trip': {
            'maximum_abs_error_deg': float(np.max(np.abs(reference_error))),
            'rms_error_deg': np.sqrt(np.mean(reference_error**2, axis=0)).tolist(),
            'limit_deg': .011,
        },
        'reference_excitation': {
            'rank': int(np.linalg.matrix_rank(centered)),
            'standardized_singular_values': singular.tolist(),
            'correlation': float(np.corrcoef(centered.T)[0, 1]),
            'axes': {
                axis: {
                    'minimum_deg': float(recorded[:, column].min()),
                    'maximum_deg': float(recorded[:, column].max()),
                    'std_deg_during_excitation': float(recorded[active, column].std()),
                } for column, axis in enumerate(AXES)
            },
        },
        'measured_motion': {
            output: {
                'minimum_delta_deg': float(measured[:, column].min()),
                'maximum_delta_deg': float(measured[:, column].max()),
                'peak_abs_delta_deg': float(np.max(np.abs(measured[:, column]))),
            } for column, output in enumerate(OUTPUTS)
        },
        'attempted_command': {
            axis: {
                'minimum_software_units': float(command[:, column].min()),
                'maximum_software_units': float(command[:, column].max()),
                'std_software_units': float(command[:, column].std()),
                'source_area_software_command_us_after_first': float(source_area[column]),
                'uniform_grid_area_software_command_us': float(grid_area[column]),
            } for column, axis in enumerate(AXES)
        },
        'feedback_current': {
            axis: {
                'baseline_3s_median_raw': float(np.median(current[:BASELINE, column])),
                'minimum_raw': float(current[:, column].min()),
                'maximum_raw': float(current[:, column].max()),
                'resampled_age_us': {
                    'minimum': int(feedback_age[:, column].min()),
                    'maximum': int(feedback_age[:, column].max()),
                    'mean': float(feedback_age[:, column].mean()),
                },
            } for column, axis in enumerate(AXES)
        },
        'can_summary': verified['can_summary'],
        'interval_us_percentiles': {
            str(p): float(np.percentile(interval_us[1:], p))
            for p in (0, 1, 50, 99, 100)
        },
    }
    capture_accepted = bool(
        audit['reference_round_trip']['maximum_abs_error_deg'] <= .011
        and audit['reference_excitation']['rank'] == 2
        and abs(audit['reference_excitation']['correlation']) <= .1
        and not verified['quality_issues']
    )
    result = {
        'capture_status': ('accepted' if capture_accepted else 'rejected'),
        'model_status': 'not_identified',
        'hardware_takeover_allowed': False,
        'audit': audit,
    }
    if phase_set == 'C' and capture_accepted:
        model = fit_frozen_arx(recorded, measured)
        result['frozen_C_closed_loop_model'] = model
        result['frozen_C_command_to_current'] = fit_command_current(command, current)
        if model['status'].startswith('frozen_C_candidate'):
            result['model_status'] = 'frozen_C_candidate_pending_independent_D'
        else:
            result['capture_status'] = 'accepted_but_C_model_rejected'
    elif phase_set == 'D':
        result['model_status'] = 'D_requires_separate_frozen_C_model_evaluator'
    result['blockers'] = [
        'independent_D_validation_not_completed',
        'closed_loop_reference_map_is_not_an_open_loop_MPC_plant',
        'feedback_current_is_not_calibrated_torque',
    ]
    result['conclusion'] = (
        'C may be released for the one-time D capture only if capture_status is accepted '
        'and model_status is frozen_C_candidate_pending_independent_D.'
    )
    return result, {
        'time_s': grid_t,
        'reference': recorded,
        'measured': measured,
        'command': command,
        'current': current,
        'feedback_age': feedback_age,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--plan', type=Path, default=DEFAULT_PLAN)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output must not exist')
    report, data = load_capture(args.capture, args.plan)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'report.json').write_text(
        json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    np.savetxt(
        args.output/'resampled.csv',
        np.column_stack((data['time_s'], data['reference'], data['measured'],
                         data['command'], data['current'], data['feedback_age'])),
        delimiter=',', comments='', fmt='%.9g',
        header=('trace_time_s,reference_big_deg,reference_small_deg,'
                'measured_big_joint_delta_deg,measured_small_joint_delta_deg,'
                'measured_heading_delta_deg,attempted_big_command,'
                'attempted_small_command,big_current_raw,small_current_raw,'
                'big_feedback_age_us,small_feedback_age_us'))
    print(json.dumps({
        'capture_status': report['capture_status'],
        'model_status': report['model_status'],
        'hardware_takeover_allowed': False,
        'output': str(args.output),
    }, indent=2))
    return 0 if report['capture_status'] == 'accepted' else 2


if __name__ == '__main__':
    raise SystemExit(main())
