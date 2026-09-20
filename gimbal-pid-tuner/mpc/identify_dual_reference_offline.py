"""Audit and fit one trace-v4 dual-reference capture entirely offline.

The result describes the existing PID closed loop.  It never authorizes or
commands hardware, and phase-set B remains an independent validation dataset.
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
from identify_closed_loop_offline import (TRAIN, TUNE, closed_loop_fir,
                                           first_order_signal)
from preview_dual_reference_excitation import DEFAULT_PLAN, reference


AXES = ('big', 'small')
OUTPUTS = ('big_joint_deg', 'small_joint_deg', 'heading_deg')


def _segment_error(prediction, target):
    error = prediction-target
    centered = target-target.mean()
    return {
        'rmse_raw': float(np.sqrt(np.mean(error**2))),
        'mean_error_raw': float(np.mean(error)),
        'r2_vs_segment_mean': float(
            1-np.sum(error**2)/max(np.sum(centered**2), 1e-12)
        ),
    }


def command_current_fit(command, current):
    """Estimate effective command/current timing with offset and load merged."""
    result = {}
    for column, axis in enumerate(AXES):
        options = []
        target = current[:, column]
        scale = max(float(np.std(target[:TRAIN])), 1.)
        for delay in range(4):
            for tau_s in (0., .004, .008, .016, .032):
                filtered = first_order_signal(command[:, column]*1000, delay, tau_s)
                design = np.column_stack((filtered[:TRAIN], np.ones(TRAIN)))
                gain, intercept = np.linalg.lstsq(design, target[:TRAIN], rcond=None)[0]
                error = gain*filtered[TRAIN:TUNE]+intercept-target[TRAIN:TUNE]
                options.append((float(np.sqrt(np.mean(error**2))/scale), delay,
                                tau_s, float(gain), float(intercept), filtered))
        score, delay, tau_s, gain, intercept, filtered = min(options, key=lambda item: item[0])
        prediction = gain*filtered+intercept
        segments = {
            name: _segment_error(prediction[start:end], target[start:end])
            for name, start, end in (
                ('train', 0, TRAIN), ('select', TRAIN, TUNE),
                ('held_out', TUNE, SAMPLES)
            )
        }
        result[axis] = {
            'status': 'diagnostic_offset_and_load_merged',
            'delay_samples': delay,
            'effective_delay_ms': delay*DT_US/1000,
            'tau_ms': tau_s*1000,
            'gain_current_raw_per_command_raw': gain,
            'intercept_raw_including_sensor_offset_and_load': intercept,
            'selection_normalized_rmse': score,
            'segments': segments,
            'warning': ('No zero-output BENCH accompanies this trial; sensor offset, '
                        'gravity/load current and holding-controller bias are not separable.'),
        }
    return result


def _vector_metrics(prediction, target, scale):
    error = prediction-target
    return {
        'rmse_deg': np.sqrt(np.mean(error**2, axis=0)).tolist(),
        'max_abs_deg': np.max(np.abs(error), axis=0).tolist(),
        'normalized_rmse': float(np.sqrt(np.mean((error/scale)**2))),
    }


def _arx_feature(output, reference_input, index, na, nb, delay):
    return np.r_[
        *[output[index-lag] for lag in range(1, na+1)],
        *[reference_input[index-delay-lag] for lag in range(nb)],
        1.,
    ]


def _arx_spectral_radius(coefficient, outputs, na):
    companion = np.zeros((outputs*na, outputs*na))
    companion[:outputs] = np.hstack([
        coefficient[lag*outputs:(lag+1)*outputs].T for lag in range(na)
    ])
    if na > 1:
        companion[outputs:, :-outputs] = np.eye(outputs*(na-1))
    return float(np.max(np.abs(np.linalg.eigvals(companion))))


def _arx_simulate(measured, reference_input, coefficient, start, end, na, nb, delay):
    predicted = np.array(measured, copy=True)
    for index in range(start, end):
        predicted[index] = _arx_feature(
            predicted, reference_input, index, na, nb, delay
        )@coefficient
        if not np.isfinite(predicted[index]).all() or np.max(np.abs(predicted[index])) > 1e6:
            return None
    return predicted


def closed_loop_arx(reference_input, measured):
    """Select a stable closed-loop ARX map and evaluate free-run prediction."""
    output_center = measured[:TRAIN].mean(axis=0)
    output_scale = np.maximum(measured[:TRAIN].std(axis=0), 360/8192)
    reference_center = reference_input[:TRAIN].mean(axis=0)
    reference_scale = np.maximum(reference_input[:TRAIN].std(axis=0), .01)
    output = (measured-output_center)/output_scale
    reference_scaled = (reference_input-reference_center)/reference_scale
    options = []
    for na in (1, 2, 4, 8):
        for nb in (4, 8, 16, 32):
            for delay in (0, 1, 2):
                history = max(na, delay+nb-1)
                x = np.asarray([
                    _arx_feature(output, reference_scaled, index, na, nb, delay)
                    for index in range(history, TRAIN)
                ])
                target = output[history:TRAIN]
                for ridge in (1e-6, 1e-4, 1e-2, 1.):
                    penalty = np.eye(x.shape[1])
                    penalty[-1, -1] = 0.
                    coefficient = np.linalg.lstsq(
                        np.vstack((x, np.sqrt(len(x)*ridge)*penalty)),
                        np.vstack((target, np.zeros((x.shape[1], target.shape[1])))),
                        rcond=None,
                    )[0]
                    radius = _arx_spectral_radius(coefficient, measured.shape[1], na)
                    if radius >= 1. or not np.isfinite(radius):
                        continue
                    prediction = _arx_simulate(
                        output, reference_scaled, coefficient, TRAIN, TUNE,
                        na, nb, delay,
                    )
                    if prediction is None:
                        continue
                    prediction_deg = prediction[TRAIN:TUNE]*output_scale+output_center
                    score = _vector_metrics(
                        prediction_deg, measured[TRAIN:TUNE], output_scale
                    )['normalized_rmse']
                    options.append((score, na, nb, delay, ridge, radius, coefficient))
    if not options:
        return {'status': 'no_stable_candidate'}
    score, na, nb, delay, ridge, radius, coefficient = min(options, key=lambda item: item[0])
    validations = {}
    for name, start, end in (
            ('selection_free_run', TRAIN, TUNE),
            ('held_out_late_excitation', TUNE, 4000),
            ('held_out_settle', 4000, SAMPLES),
            ('held_out_free_run', TUNE, SAMPLES)):
        prediction = _arx_simulate(
            output, reference_scaled, coefficient, start, end, na, nb, delay
        )
        prediction_deg = prediction[start:end]*output_scale+output_center
        hold = np.repeat(measured[start-1:start], end-start, axis=0)
        validations[name] = {
            'model': _vector_metrics(prediction_deg, measured[start:end], output_scale),
            'hold': _vector_metrics(hold, measured[start:end], output_scale),
        }
    continuous = _arx_simulate(
        output, reference_scaled, coefficient, TRAIN, SAMPLES, na, nb, delay
    )
    continuous_deg = continuous[TRAIN:]*output_scale+output_center
    validations['continuous_from_training_end'] = {
        'model': _vector_metrics(continuous_deg, measured[TRAIN:], output_scale),
        'hold': _vector_metrics(
            np.repeat(measured[TRAIN-1:TRAIN], SAMPLES-TRAIN, axis=0),
            measured[TRAIN:], output_scale,
        ),
    }
    selected = validations['selection_free_run']
    held = validations['held_out_late_excitation']
    selection_improvement = 1-(selected['model']['normalized_rmse']/
                               selected['hold']['normalized_rmse'])
    status = ('phase_A_internal_candidate_pending_phase_B'
              if selection_improvement >= .2
              else 'rejected_not_better_than_hold')
    return {
        'status': status,
        'structure': {
            'output_lags': na,
            'reference_lags': nb,
            'reference_delay_samples': delay,
            'reference_delay_ms': delay*DT_US/1000,
        },
        'ridge': ridge,
        'autonomous_spectral_radius': radius,
        'selection_normalized_rmse': score,
        'selection_improvement_vs_hold_fraction': selection_improvement,
        'output_center_deg': output_center.tolist(),
        'output_scale_deg': output_scale.tolist(),
        'reference_center_deg': reference_center.tolist(),
        'reference_scale_deg': reference_scale.tolist(),
        'normalized_coefficient': coefficient.tolist(),
        'validation': validations,
        'late_excitation_note': (
            'The 14-16 s window tapers to zero and has little remaining motion; '
            'its local hold baseline is stronger than the phase-A model. The '
            'predeclared phase-B trial remains the decisive independent test.'
            if held['model']['normalized_rmse'] >= held['hold']['normalized_rmse']
            else None
        ),
        'warning': ('This is the existing PID closed-loop reference map. Phase B '
                    'must be evaluated without refitting before any stronger conclusion.'),
    }


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
            or decoder.profile['profile'] != 3 or decoder.profile['build'] != BUILD
            or not verified['capture_timing']['full_duration_timing_accepted']):
        raise ValueError('capture does not pass trace-v4 timing/data checks: '
                         +str(verified['quality_issues']))
    if decoder.metadata != saved['metadata'] or decoder.profile != saved['profile']:
        raise ValueError('raw metadata/profile differs from saved report')

    rows = decoder.rows
    trace_us = np.asarray([row['trace_us'] for row in rows], dtype=np.int64)
    interval_us = np.asarray([row['interval_us'] for row in rows], dtype=np.int64)
    areas = np.asarray([[row['big_integral_raw_us'], row['small_integral_raw_us']]
                        for row in rows], dtype=float)
    grid_us, command = resample_integrated_input(trace_us, areas)
    source_t = trace_us/1e6
    grid_t = grid_us/1e6

    big = joint_delta([row['big_raw'] for row in rows])
    small = joint_delta([row['small_raw'] for row in rows])
    yaw_rad = np.unwrap(np.deg2rad([row['yaw_deg'] for row in rows]))
    heading = np.rad2deg(yaw_rad-yaw_rad[0])
    measured_source = np.column_stack((big, small, heading))
    measured = np.column_stack([
        np.interp(grid_t, source_t, measured_source[:, column])
        for column in range(3)
    ])
    recorded_reference_source = np.asarray([
        [row['big_reference_offset_cdeg']/100,
         row['small_heading_reference_offset_cdeg']/100]
        for row in rows
    ])
    recorded_reference = np.column_stack([
        np.interp(grid_t, source_t, recorded_reference_source[:, column])
        for column in range(2)
    ])
    phase_set = 'B' if decoder.profile['reverse'] else 'A'
    plan = json.loads(Path(plan_path).read_text(encoding='utf-8'))
    # The forced terminal record may be a fraction of a millisecond beyond the
    # nominal 20 s endpoint; the scheduled reference is already exactly zero.
    expected_source = reference(
        plan, phase_set, np.minimum(source_t, plan['duration_ms']/1000)
    )
    reference_error = recorded_reference_source-expected_source

    current_columns, feedback_age_columns = [], []
    for axis in AXES:
        sampled, age = causal_resample(
            [row[axis+'_feedback_us'] for row in rows],
            [row[axis+'_current'] for row in rows], grid_us)
        current_columns.append(sampled)
        feedback_age_columns.append(age)
    current = np.column_stack(current_columns)
    feedback_age = np.column_stack(feedback_age_columns)

    active = (grid_t >= 2.) & (grid_t < 16.)
    centered_reference = recorded_reference[active]-recorded_reference[active].mean(axis=0)
    singular = np.linalg.svd(
        centered_reference/np.maximum(centered_reference.std(axis=0), 1e-12),
        compute_uv=False,
    )
    trial = {
        'info': {'axis': 'dual_'+phase_set},
        'reference': recorded_reference,
        'y': measured,
    }
    closed_loop = closed_loop_fir([trial])
    closed_loop_state_model = closed_loop_arx(recorded_reference, measured)
    current_model = command_current_fit(command, current)

    source_area = np.sum(areas[1:], axis=0)/1000
    grid_area = np.sum(command[:-1]*DT_US, axis=0)
    audit = {
        'source': str(directory.resolve()),
        'raw_sha256': hashlib.sha256(raw).hexdigest(),
        'trial_id': decoder.metadata['id'],
        'firmware_build': '0x%08X' % decoder.metadata['build'],
        'phase_set': phase_set,
        'verified_quality_issues': verified['quality_issues'],
        'capture_timing': verified['capture_timing'],
        'reference_round_trip': {
            'maximum_abs_error_deg': float(np.max(np.abs(reference_error))),
            'rms_error_deg': np.sqrt(np.mean(reference_error**2, axis=0)).tolist(),
            'centidegree_quantization_limit_deg': .01,
        },
        'reference_excitation': {
            'rank': int(np.linalg.matrix_rank(centered_reference)),
            'standardized_singular_values': singular.tolist(),
            'correlation': float(np.corrcoef(centered_reference.T)[0, 1]),
            'axes': {
                axis: {
                    'minimum_deg': float(recorded_reference[:, column].min()),
                    'maximum_deg': float(recorded_reference[:, column].max()),
                    'std_deg_during_excitation': float(recorded_reference[active, column].std()),
                }
                for column, axis in enumerate(AXES)
            },
        },
        'measured_motion': {
            output: {
                'minimum_delta_deg': float(measured[:, column].min()),
                'maximum_delta_deg': float(measured[:, column].max()),
                'peak_abs_delta_deg': float(np.max(np.abs(measured[:, column]))),
            }
            for column, output in enumerate(OUTPUTS)
        },
        'attempted_command': {
            axis: {
                'minimum_software_units': float(command[:, column].min()),
                'maximum_software_units': float(command[:, column].max()),
                'std_software_units': float(command[:, column].std()),
                'source_area_software_command_us_after_first': float(source_area[column]),
                'uniform_grid_area_software_command_us': float(grid_area[column]),
            }
            for column, axis in enumerate(AXES)
        },
        'feedback_current': {
            axis: {
                'first_2s_median_raw': float(np.median(current[:500, column])),
                'minimum_raw': float(current[:, column].min()),
                'maximum_raw': float(current[:, column].max()),
                'resampled_age_us': {
                    'minimum': int(feedback_age[:, column].min()),
                    'maximum': int(feedback_age[:, column].max()),
                    'mean': float(feedback_age[:, column].mean()),
                },
            }
            for column, axis in enumerate(AXES)
        },
        'can_summary': verified['can_summary'],
        'interval_us_percentiles': {
            str(percentile): float(np.percentile(interval_us[1:], percentile))
            for percentile in (0, 1, 50, 99, 100)
        },
        'resampling': ('Measured angles and recorded references linearly interpolated by trace_us; '
                       'attempted command area exactly differenced on a 4 ms grid; '
                       'feedback current held causally from its own timestamp.'),
    }
    accepted = bool(
        audit['reference_round_trip']['maximum_abs_error_deg'] <= .011
        and audit['reference_excitation']['rank'] == 2
        and not verified['quality_issues']
    )
    result = {
        'capture_status': ('accepted_for_phase_'+phase_set+'_closed_loop_fit'
                           if accepted else 'rejected_capture'),
        'model_status': ('phase_A_candidate_pending_independent_phase_B_validation'
                         if accepted and phase_set == 'A'
                         else 'independent_phase_B_validation_only'
                         if accepted else 'not_identified'),
        'hardware_takeover_allowed': False,
        'audit': audit,
        'closed_loop_reference_model': closed_loop,
        'closed_loop_state_model': closed_loop_state_model,
        'command_to_feedback_current': current_model,
        'blockers': [
            'phase_B_independent_validation_not_collected',
            'closed_loop_reference_map_is_not_an_open_loop_MPC_plant',
            'feedback_current_not_calibrated_to_joint_torque',
            'zero_output_current_offset_not_separable_without_optional_BENCH',
        ],
        'conclusion': ('The full 20 s phase-A record is usable with trace-time resampling. '
                       'The fitted maps remain diagnostic until independent phase-B validation.'),
    }
    return result, {
        'time_s': grid_t,
        'reference': recorded_reference,
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
        json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8'
    )
    np.savetxt(
        args.output/'resampled.csv',
        np.column_stack((data['time_s'], data['reference'], data['measured'],
                         data['command'], data['current'], data['feedback_age'])),
        delimiter=',', comments='', fmt='%.9g',
        header=('trace_time_s,reference_big_deg,reference_small_deg,'
                'measured_big_joint_delta_deg,measured_small_joint_delta_deg,'
                'measured_heading_delta_deg,attempted_big_command,'
                'attempted_small_command,big_current_raw,small_current_raw,'
                'big_feedback_age_us,small_feedback_age_us'),
    )
    print(json.dumps({
        'capture_status': report['capture_status'],
        'model_status': report['model_status'],
        'hardware_takeover_allowed': False,
        'output': str(args.output),
    }, indent=2))
    return 0 if report['capture_status'].startswith('accepted') else 2


if __name__ == '__main__':
    raise SystemExit(main())
