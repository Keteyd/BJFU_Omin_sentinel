"""Validate a frozen phase-A closed-loop model on an independent phase-B trace.

The phase-B samples are decoded, audited and resampled, but never used to fit
or select coefficients.  This program is offline only and never authorizes or
commands hardware.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from yaw_dual_trace_protocol import BUILD, Decoder
from identify_can_trace_offline import (DT_US, SAMPLES, causal_resample,
                                        joint_delta, resample_integrated_input)
from identify_closed_loop_offline import first_order_signal
from identify_dual_reference_offline import (AXES, OUTPUTS, _arx_simulate,
                                              _vector_metrics)
from preview_dual_reference_excitation import DEFAULT_PLAN, reference


EXCITATION = (500, 4000)
FULL_AFTER_BASELINE = (500, SAMPLES)
PRIMARY_IMPROVEMENT_GATE = .30
START_DEPENDENT_CONFIGURATION = {
    'amplitude_deg', 'big_anchor_deg', 'small_heading_anchor_deg',
    'small_joint_start_deg', 'pitch_target_rad', 'imu_roll_start_deg',
    'imu_pitch_start_deg',
}


def _read_verified_capture(directory, expected_phase, plan_path):
    directory = Path(directory)
    saved = json.loads((directory/'report.json').read_text(encoding='utf-8'))
    setup = json.loads((directory/'setup.json').read_text(encoding='utf-8'))
    raw = (directory/'raw.bin').read_bytes()
    decoder = Decoder(saved['trial_id'])
    for offset in range(0, len(raw), 4096):
        decoder.feed(raw[offset:offset+4096])
    verified = decoder.report()
    phase_set = 'B' if decoder.profile['reverse'] else 'A'
    failures = []
    if saved.get('error'):
        failures.append('saved_report_error')
    if not decoder.complete:
        failures.append('decoder_incomplete')
    if decoder.crc_errors:
        failures.append('crc_errors')
    if decoder.discarded_bytes:
        failures.append('discarded_bytes')
    if verified['quality_issues']:
        failures.append('quality_issues:'+','.join(verified['quality_issues']))
    if decoder.metadata['phase'] != 5 or decoder.metadata['reason']:
        failures.append('terminal_status_not_ok')
    if decoder.profile['profile'] != 3 or decoder.profile['build'] != BUILD:
        failures.append('wrong_profile_or_build')
    if not verified['capture_timing']['full_duration_timing_accepted']:
        failures.append('full_duration_timing_rejected')
    if phase_set != expected_phase:
        failures.append('expected_phase_'+expected_phase+'_got_'+phase_set)
    if decoder.metadata != saved['metadata'] or decoder.profile != saved['profile']:
        failures.append('raw_metadata_or_profile_differs_from_saved_report')
    if failures:
        raise ValueError(str(directory)+': '+'; '.join(failures))

    rows = decoder.rows
    trace_us = np.asarray([row['trace_us'] for row in rows], dtype=np.int64)
    grid_us, command = resample_integrated_input(
        trace_us,
        np.asarray([[row['big_integral_raw_us'], row['small_integral_raw_us']]
                    for row in rows], dtype=float),
    )
    source_t, grid_t = trace_us/1e6, grid_us/1e6
    measured_source = np.column_stack((
        joint_delta([row['big_raw'] for row in rows]),
        joint_delta([row['small_raw'] for row in rows]),
        np.rad2deg(np.unwrap(np.deg2rad([row['yaw_deg'] for row in rows]))
                   - np.deg2rad(rows[0]['yaw_deg'])),
    ))
    measured = np.column_stack([
        np.interp(grid_t, source_t, measured_source[:, column])
        for column in range(3)
    ])
    recorded_source = np.asarray([
        [row['big_reference_offset_cdeg']/100,
         row['small_heading_reference_offset_cdeg']/100]
        for row in rows
    ])
    recorded_reference = np.column_stack([
        np.interp(grid_t, source_t, recorded_source[:, column])
        for column in range(2)
    ])
    plan = json.loads(Path(plan_path).read_text(encoding='utf-8'))
    expected = reference(
        plan, phase_set, np.minimum(source_t, plan['duration_ms']/1000)
    )
    reference_error = recorded_source-expected
    current_columns, feedback_age_columns = [], []
    for axis in AXES:
        sampled, age = causal_resample(
            [row[axis+'_feedback_us'] for row in rows],
            [row[axis+'_current'] for row in rows], grid_us,
        )
        current_columns.append(sampled)
        feedback_age_columns.append(age)
    current = np.column_stack(current_columns)
    feedback_age = np.column_stack(feedback_age_columns)
    active = recorded_reference[EXCITATION[0]:EXCITATION[1]]
    centered = active-active.mean(axis=0)
    audit = {
        'source': str(directory.resolve()),
        'raw_sha256': hashlib.sha256(raw).hexdigest(),
        'trial_id': decoder.metadata['id'],
        'firmware_build': '0x%08X' % decoder.metadata['build'],
        'phase_set': phase_set,
        'records': len(rows),
        'verified_quality_issues': verified['quality_issues'],
        'capture_timing': verified['capture_timing'],
        'reference_round_trip': {
            'maximum_abs_error_deg': float(np.max(np.abs(reference_error))),
            'rms_error_deg': np.sqrt(np.mean(reference_error**2, axis=0)).tolist(),
        },
        'reference_excitation': {
            'rank': int(np.linalg.matrix_rank(centered)),
            'correlation': float(np.corrcoef(centered.T)[0, 1]),
        },
        'measured_motion': {
            output: {
                'minimum_delta_deg': float(measured[:, column].min()),
                'maximum_delta_deg': float(measured[:, column].max()),
                'peak_abs_delta_deg': float(np.max(np.abs(measured[:, column]))),
            }
            for column, output in enumerate(OUTPUTS)
        },
        'attempted_command_during_excitation': {
            axis: {
                'minimum_software_units': float(command[EXCITATION[0]:EXCITATION[1], column].min()),
                'maximum_software_units': float(command[EXCITATION[0]:EXCITATION[1], column].max()),
                'std_software_units': float(command[EXCITATION[0]:EXCITATION[1], column].std()),
                'near_zero_fraction_abs_below_0_05': float(np.mean(
                    np.abs(command[EXCITATION[0]:EXCITATION[1], column]) < .05
                )),
            }
            for column, axis in enumerate(AXES)
        },
        'can_summary': verified['can_summary'],
    }
    return {
        'saved': saved,
        'setup': setup,
        'audit': audit,
        'time_s': grid_t,
        'reference': recorded_reference,
        'measured': measured,
        'command': command,
        'current': current,
        'feedback_age': feedback_age,
    }


def _configuration_comparison(a, b):
    a_config = a['saved']['metadata']['configuration']
    b_config = b['saved']['metadata']['configuration']
    fixed_keys = sorted((set(a_config) | set(b_config))-START_DEPENDENT_CONFIGURATION)
    differences = {
        key: {'phase_A': a_config.get(key), 'phase_B': b_config.get(key)}
        for key in fixed_keys if a_config.get(key) != b_config.get(key)
    }
    start_values = {
        key: {'phase_A': a_config.get(key), 'phase_B': b_config.get(key)}
        for key in sorted(START_DEPENDENT_CONFIGURATION)
    }
    return {
        'fixed_configuration_matches': not differences,
        'fixed_differences': differences,
        'start_dependent_values': start_values,
        'setup_json_matches': (a['setup'] == b['setup']),
    }


def _evaluate_state_model(frozen, data, source_phase='A', target_phase='B'):
    structure = frozen['structure']
    na = structure['output_lags']
    nb = structure['reference_lags']
    delay = structure['reference_delay_samples']
    coefficient = np.asarray(frozen['normalized_coefficient'], dtype=float)
    output_center = np.asarray(frozen['output_center_deg'], dtype=float)
    output_scale = np.asarray(frozen['output_scale_deg'], dtype=float)
    reference_center = np.asarray(frozen['reference_center_deg'], dtype=float)
    reference_scale = np.asarray(frozen['reference_scale_deg'], dtype=float)
    output = (data['measured']-output_center)/output_scale
    reference_scaled = (data['reference']-reference_center)/reference_scale
    segments = {}
    for name, (start, end) in {
            'primary_excitation_free_run': EXCITATION,
            'full_after_baseline_free_run': FULL_AFTER_BASELINE,
    }.items():
        prediction = _arx_simulate(
            output, reference_scaled, coefficient, start, end, na, nb, delay
        )
        if prediction is None:
            segments[name] = {'finite': False}
            continue
        prediction_deg = prediction[start:end]*output_scale+output_center
        target = data['measured'][start:end]
        hold = np.repeat(data['measured'][start-1:start], end-start, axis=0)
        model_metrics = _vector_metrics(prediction_deg, target, output_scale)
        hold_metrics = _vector_metrics(hold, target, output_scale)
        model_rmse = np.asarray(model_metrics['rmse_deg'])
        hold_rmse = np.asarray(hold_metrics['rmse_deg'])
        improvements = 1-model_rmse/np.maximum(hold_rmse, 1e-12)
        segments[name] = {
            'finite': True,
            'model': model_metrics,
            'hold': hold_metrics,
            'improvement_vs_hold_fraction_by_output': dict(zip(OUTPUTS, improvements.tolist())),
        }
    primary = segments['primary_excitation_free_run']
    joint_improvements = [
        primary.get('improvement_vs_hold_fraction_by_output', {}).get(name, -np.inf)
        for name in OUTPUTS[:2]
    ]
    return {
        'coefficient_source_phase': source_phase,
        'validation_target_phase': target_phase,
        'coefficients_frozen_before_target_phase_evaluation': True,
        'target_phase_used_for_fitting_or_selection': False,
        'structure': structure,
        'autonomous_spectral_radius_from_source_phase': frozen['autonomous_spectral_radius'],
        'segments': segments,
        'primary_gate': {
            'metric': 'per-joint RMSE improvement over constant hold during 2-16 s excitation',
            'minimum_required_fraction': PRIMARY_IMPROVEMENT_GATE,
            'joint_outputs': list(OUTPUTS[:2]),
            'passed': bool(primary.get('finite') and all(
                item >= PRIMARY_IMPROVEMENT_GATE for item in joint_improvements
            )),
        },
    }


def _evaluate_current_models(frozen_models, b):
    result = {}
    for column, axis in enumerate(AXES):
        model = frozen_models[axis]
        filtered = first_order_signal(
            b['command'][:, column]*1000,
            model['delay_samples'], model['tau_ms']/1000,
        )
        prediction = (model['gain_current_raw_per_command_raw']*filtered
                      + model['intercept_raw_including_sensor_offset_and_load'])
        target = b['current'][:, column]
        segments = {}
        for name, (start, end) in {
                'baseline': (0, 500), 'excitation': EXCITATION,
                'settle': (4000, SAMPLES), 'full': (0, SAMPLES),
        }.items():
            error = prediction[start:end]-target[start:end]
            centered = target[start:end]-target[start:end].mean()
            segments[name] = {
                'rmse_raw': float(np.sqrt(np.mean(error**2))),
                'mean_error_raw': float(np.mean(error)),
                'r2_vs_segment_mean': float(
                    1-np.sum(error**2)/max(np.sum(centered**2), 1e-12)
                ),
            }
        result[axis] = {
            'parameters_frozen_from_phase_A': True,
            'delay_samples': model['delay_samples'],
            'effective_delay_ms': model['effective_delay_ms'],
            'tau_ms': model['tau_ms'],
            'gain_current_raw_per_command_raw': model['gain_current_raw_per_command_raw'],
            'phase_A_intercept_raw': model['intercept_raw_including_sensor_offset_and_load'],
            'segments': segments,
            'warning': ('The intercept merges sensor offset, gravity/load and PID holding bias; '
                        'its transfer is diagnostic because no paired zero-output BENCH exists.'),
        }
    return result


def _symmetric_comparison(a_report, b_report, a, b):
    b_hash_matches = b_report['audit']['raw_sha256'] == b['audit']['raw_sha256']
    if not b_hash_matches:
        raise ValueError('phase-B fit report does not identify the supplied phase-B raw.bin')
    reverse = _evaluate_state_model(
        b_report['closed_loop_state_model'], a, source_phase='B', target_phase='A'
    )
    a_state, b_state = (a_report['closed_loop_state_model'],
                        b_report['closed_loop_state_model'])
    structure_matches = a_state['structure'] == b_state['structure']
    current = {}
    current_gate = True
    for axis in AXES:
        am = a_report['command_to_feedback_current'][axis]
        bm = b_report['command_to_feedback_current'][axis]
        gain_ratio = bm['gain_current_raw_per_command_raw']/am['gain_current_raw_per_command_raw']
        delay_difference = bm['delay_samples']-am['delay_samples']
        item_passed = bool(gain_ratio > 0 and .1 <= abs(gain_ratio) <= 10
                           and abs(delay_difference) <= 1)
        current_gate &= item_passed
        current[axis] = {
            'phase_A_gain': am['gain_current_raw_per_command_raw'],
            'phase_B_gain': bm['gain_current_raw_per_command_raw'],
            'phase_B_over_A_gain_ratio': gain_ratio,
            'phase_A_delay_samples': am['delay_samples'],
            'phase_B_delay_samples': bm['delay_samples'],
            'delay_difference_samples': delay_difference,
            'phase_A_tau_ms': am['tau_ms'],
            'phase_B_tau_ms': bm['tau_ms'],
            'sign_order_and_one_sample_delay_gate_passed': item_passed,
        }
    return {
        'phase_B_fit_raw_hash_matches': b_hash_matches,
        'phase_B_model_on_phase_A': reverse,
        'state_structure_matches': structure_matches,
        'phase_A_structure': a_state['structure'],
        'phase_B_structure': b_state['structure'],
        'current_parameter_comparison': current,
        'parameter_stability_gate_passed': bool(structure_matches and current_gate),
        'reverse_predictive_gate_passed': reverse['primary_gate']['passed'],
    }


def validate(a_capture, b_capture, frozen_report_path, plan_path=DEFAULT_PLAN,
             phase_b_fit_report_path=None):
    frozen_report_path = Path(frozen_report_path)
    frozen_report = json.loads(frozen_report_path.read_text(encoding='utf-8'))
    a = _read_verified_capture(a_capture, 'A', plan_path)
    b = _read_verified_capture(b_capture, 'B', plan_path)
    frozen_hash_matches = (
        frozen_report['audit']['raw_sha256'] == a['audit']['raw_sha256']
    )
    if not frozen_hash_matches:
        raise ValueError('frozen model report does not identify the supplied phase-A raw.bin')
    configuration = _configuration_comparison(a, b)
    state = _evaluate_state_model(frozen_report['closed_loop_state_model'], b)
    current = _evaluate_current_models(frozen_report['command_to_feedback_current'], b)
    capture_gate = bool(
        b['audit']['reference_round_trip']['maximum_abs_error_deg'] <= .011
        and b['audit']['reference_excitation']['rank'] == 2
        and configuration['fixed_configuration_matches']
        and configuration['setup_json_matches']
    )
    predictive_gate = state['primary_gate']['passed']
    symmetric = None
    if phase_b_fit_report_path is not None:
        phase_b_fit_report_path = Path(phase_b_fit_report_path)
        b_fit = json.loads(phase_b_fit_report_path.read_text(encoding='utf-8'))
        symmetric = _symmetric_comparison(frozen_report, b_fit, a, b)
    symmetric_gate = bool(
        symmetric and symmetric['reverse_predictive_gate_passed']
        and symmetric['parameter_stability_gate_passed']
    )
    accepted = capture_gate and predictive_gate and symmetric_gate
    blockers = [
        'residual_periodicity_and_uncertainty_audit_not_run',
        'closed_loop_reference_map_is_not_an_open_loop_MPC_plant',
        'feedback_current_not_calibrated_to_joint_torque',
        'zero_output_current_offset_not_separable_without_optional_BENCH',
    ]
    if symmetric is None:
        blockers.insert(0, 'symmetric_phase_B_fit_phase_A_validation_not_run')
    elif not symmetric_gate:
        blockers.insert(0, 'symmetric_prediction_or_parameter_stability_gate_failed')
    if not capture_gate:
        blockers.insert(0, 'phase_B_capture_or_configuration_gate_failed')
    if not predictive_gate:
        blockers.insert(0, 'frozen_phase_A_model_failed_phase_B_predictive_gate')
    return {
        'validation_status': (
            'dual_phase_closed_loop_model_passed_all_predeclared_gates'
            if accepted else 'closed_loop_model_rejected_by_dual_phase_validation'
        ),
        'hardware_takeover_allowed': False,
        'frozen_model': {
            'source': str(frozen_report_path.resolve()),
            'report_sha256': hashlib.sha256(frozen_report_path.read_bytes()).hexdigest(),
            'phase_A_raw_hash_matches': frozen_hash_matches,
            'phase_B_used_for_fitting_or_selection': False,
        },
        'phase_A_audit': a['audit'],
        'phase_B_audit': b['audit'],
        'configuration_comparison': configuration,
        'frozen_phase_A_state_model_on_phase_B': state,
        'frozen_phase_A_command_current_models_on_phase_B': current,
        'symmetric_check': symmetric,
        'capture_gate_passed': capture_gate,
        'predictive_gate_passed': predictive_gate,
        'blockers': blockers,
        'conclusion': (
            'Both directions pass the predeclared prediction and parameter-stability '
            'gates. The result is still a diagnostic PID closed-loop map only.'
            if accepted else
            'The closed-loop model is rejected because it does not pass every '
            'predeclared dual-phase gate. Do not promote it to shadow use.'
        ),
    }, b


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase_a_capture', type=Path)
    parser.add_argument('phase_b_capture', type=Path)
    parser.add_argument('--frozen-a-report', required=True, type=Path)
    parser.add_argument('--phase-b-fit-report', type=Path)
    parser.add_argument('--plan', type=Path, default=DEFAULT_PLAN)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output must not exist')
    report, b = validate(
        args.phase_a_capture, args.phase_b_capture, args.frozen_a_report, args.plan,
        args.phase_b_fit_report,
    )
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'report.json').write_text(
        json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8'
    )
    np.savetxt(
        args.output/'phase_b_resampled.csv',
        np.column_stack((b['time_s'], b['reference'], b['measured'], b['command'],
                         b['current'], b['feedback_age'])),
        delimiter=',', comments='', fmt='%.9g',
        header=('trace_time_s,reference_big_deg,reference_small_deg,'
                'measured_big_joint_delta_deg,measured_small_joint_delta_deg,'
                'measured_heading_delta_deg,attempted_big_command,'
                'attempted_small_command,big_current_raw,small_current_raw,'
                'big_feedback_age_us,small_feedback_age_us'),
    )
    print(json.dumps({
        'validation_status': report['validation_status'],
        'capture_gate_passed': report['capture_gate_passed'],
        'predictive_gate_passed': report['predictive_gate_passed'],
        'hardware_takeover_allowed': False,
        'output': str(args.output),
    }, indent=2))
    return 0 if report['validation_status'].endswith('passed_all_predeclared_gates') else 2


if __name__ == '__main__':
    raise SystemExit(main())
