"""Compare cable-present E with cable-free R1/F4/F5 and audit scale transfer.

The accepted C closed-loop model is never modified.  A second model with the
same preselected 12/4/0 ridge-1 structure is fit on R1 only and is reported as
a development diagnostic.  All signals are reconstructed on a 4 ms grid from
the recorded microsecond timestamps; software command is area preserving and
feedback current is sampled causally from its own feedback timestamp.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from yaw_cablefree_trace_protocol import Decoder as LadderDecoder
from yaw_cde_trace_protocol import Decoder as EDecoder
from identify_can_trace_offline import (causal_resample, joint_delta,
                                         resample_integrated_input)
from identify_cd_reference_offline import fit_frozen_arx
from analyze_cde_residual_friction import frozen_prediction, smooth_velocity


DT_US = 4000
DT = DT_US / 1e6
SAMPLES = 9000
BASELINE = 750
ACTIVE_END = 7750
OUTPUTS = ('big_joint_deg', 'small_joint_deg', 'heading_deg')
AXES = ('big', 'small')


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest().upper()


def metrics(prediction, measured):
    error = prediction-measured
    rmse = np.sqrt(np.mean(error**2, axis=0))
    return {
        'rmse_deg': rmse.tolist(),
        'mae_deg': np.mean(np.abs(error), axis=0).tolist(),
        'maximum_abs_error_deg': np.max(np.abs(error), axis=0).tolist(),
        'mean_error_deg': np.mean(error, axis=0).tolist(),
    }


def safe_corr(a, b):
    a, b = np.asarray(a), np.asarray(b)
    if len(a) < 3 or np.std(a) < 1e-12 or np.std(b) < 1e-12:
        return None
    return float(np.corrcoef(a, b)[0, 1])


def decoder_report(directory, decoder_class, trial_id):
    raw_path = Path(directory)/'raw.bin'
    decoder = decoder_class(trial_id)
    raw = raw_path.read_bytes()
    for offset in range(0, len(raw), 4096):
        decoder.feed(raw[offset:offset+4096])
    report = decoder.report()
    timing = report.get('capture_timing', {})
    if (not decoder.complete or decoder.crc_errors
            or report.get('quality_issues')
            or report.get('in_stream_discarded_wire_bytes', 0)
            or not timing.get('full_duration_timing_accepted')
            or decoder.metadata.get('phase') != 5
            or decoder.metadata.get('reason') != 0):
        raise ValueError(f'raw capture failed strict replay: {directory}: {report}')
    return decoder, report, {
        'directory': str(Path(directory).resolve()),
        'raw_sha256': sha256(raw_path),
        'raw_records': len(decoder.rows),
        'raw_complete': bool(decoder.complete),
        'crc_errors': decoder.crc_errors,
        'quality_issues': report.get('quality_issues', []),
        'pre_sync_discarded_wire_bytes': report.get('pre_sync_discarded_wire_bytes', 0),
        'in_stream_discarded_wire_bytes': report.get('in_stream_discarded_wire_bytes', 0),
        'capture_timing': timing,
    }


def load_capture(name, directory, decoder_class, trial_id, scale):
    decoder, raw_report, provenance = decoder_report(directory, decoder_class, trial_id)
    rows = decoder.rows
    trace_us = np.asarray([r['trace_us'] for r in rows], dtype=np.int64)
    areas = np.asarray([[r['big_integral_raw_us'], r['small_integral_raw_us']]
                        for r in rows], dtype=float)
    grid_us, command = resample_integrated_input(trace_us, areas, samples=SAMPLES)
    if len(grid_us) != SAMPLES or grid_us[-1] > trace_us[-1]:
        raise ValueError(f'{name}: insufficient time coverage for common grid')

    measured_source = np.column_stack((
        joint_delta([r['big_raw'] for r in rows]),
        joint_delta([r['small_raw'] for r in rows]),
        np.rad2deg(np.unwrap(np.deg2rad([r['yaw_deg'] for r in rows]))),
    ))
    measured_source[:, 2] -= measured_source[0, 2]
    measured = np.column_stack([
        np.interp(grid_us, trace_us, measured_source[:, i]) for i in range(3)
    ])
    reference_source = np.asarray([
        [r['big_reference_offset_cdeg']/100,
         r['small_heading_reference_offset_cdeg']/100] for r in rows
    ])
    reference = np.column_stack([
        np.interp(grid_us, trace_us, reference_source[:, i]) for i in range(2)
    ])
    current, feedback_age = [], []
    for axis in AXES:
        sampled, age = causal_resample(
            [r[axis+'_feedback_us'] for r in rows],
            [r[axis+'_current'] for r in rows], grid_us)
        current.append(sampled)
        feedback_age.append(age)
    current = np.column_stack(current)
    feedback_age = np.column_stack(feedback_age)
    return {
        'name': name,
        'scale': scale,
        'time_s': (grid_us-grid_us[0])/1e6,
        'reference': reference,
        'measured': measured,
        'command': command,
        'current': current,
        'feedback_age': feedback_age,
        'velocity': smooth_velocity(measured),
        'provenance': provenance,
        'raw_report': raw_report,
    }


def target_outputs(reference):
    # The second scheduled coordinate is absolute heading.  For planar yaw,
    # small-joint target = heading target - big-joint target.
    return np.column_stack((reference[:, 0],
                            reference[:, 1]-reference[:, 0],
                            reference[:, 1]))


def segment_metrics(data):
    target = target_outputs(data['reference'])
    error = data['measured']-target
    result = {}
    for label, first, last in (
            ('baseline_0_3s', 0, BASELINE),
            ('excitation_3_31s', BASELINE, ACTIVE_END),
            ('settle_31_36s', ACTIVE_END, SAMPLES)):
        e = error[first:last]
        result[label] = {
            'tracking_rmse_deg': np.sqrt(np.mean(e**2, axis=0)).tolist(),
            'tracking_mean_error_deg': np.mean(e, axis=0).tolist(),
            'tracking_p95_abs_error_deg': np.percentile(np.abs(e), 95, axis=0).tolist(),
        }
    closure = data['measured'][:, 2]-data['measured'][:, 0]-data['measured'][:, 1]
    result['kinematic_closure_heading_minus_joints'] = {
        'rmse_deg': float(np.sqrt(np.mean(closure[BASELINE:ACTIVE_END]**2))),
        'mean_deg': float(np.mean(closure[BASELINE:ACTIVE_END])),
    }
    return result


def direction_stats(values, velocity, threshold=.2):
    pos, neg = velocity > threshold, velocity < -threshold
    p = float(np.median(values[pos])) if pos.any() else None
    n = float(np.median(values[neg])) if neg.any() else None
    return {
        'velocity_threshold_dps': threshold,
        'positive_samples': int(pos.sum()),
        'negative_samples': int(neg.sum()),
        'positive_median': p,
        'negative_median': n,
        'half_direction_gap': None if p is None or n is None else (p-n)/2,
        'direction_midpoint': None if p is None or n is None else (p+n)/2,
    }


def friction_and_bias(data):
    active = slice(BASELINE, ACTIVE_END)
    result = {}
    for i, axis in enumerate(AXES):
        base = data['current'][:BASELINE, i]
        cur = data['current'][active, i]
        cmd = data['command'][active, i]
        vel = data['velocity'][active, i]
        pos = data['measured'][active, i]
        centered_current = cur-np.median(base)
        current_direction = direction_stats(centered_current, vel)
        command_direction = direction_stats(cmd, vel)
        for item in (current_direction, command_direction):
            item['half_direction_gap_per_reference_scale'] = (
                None if item['half_direction_gap'] is None
                else item['half_direction_gap']/data['scale'])
        result[axis] = {
            'baseline_current_raw': {
                'median': float(np.median(base)),
                'mean': float(np.mean(base)),
                'iqr': float(np.percentile(base, 75)-np.percentile(base, 25)),
                'std': float(np.std(base)),
            },
            'active_current_raw': {
                'median': float(np.median(cur)),
                'p05': float(np.percentile(cur, 5)),
                'p95': float(np.percentile(cur, 95)),
            },
            'direction_proxy': {
                'baseline_centered_current_raw': current_direction,
                'software_command': command_direction,
            },
            'position_dependence': {
                'position_vs_command_correlation': safe_corr(pos, cmd),
                'position_vs_baseline_centered_current_correlation': safe_corr(pos, centered_current),
            },
        }
    return result


def evaluate_model(data, model):
    prediction = frozen_prediction(data, model)
    active = slice(BASELINE, ACTIVE_END)
    measured = data['measured'][active]
    held = np.repeat(data['measured'][BASELINE-1:BASELINE], ACTIVE_END-BASELINE, axis=0)
    model_metrics = metrics(prediction[active], measured)
    hold_metrics = metrics(held, measured)
    model_rmse = np.asarray(model_metrics['rmse_deg'])
    hold_rmse = np.asarray(hold_metrics['rmse_deg'])
    return prediction, {
        'model': model_metrics,
        'hold': hold_metrics,
        'improvement_over_hold_fraction':
            (1-model_rmse/np.maximum(hold_rmse, 1e-12)).tolist(),
        'normalized_rmse_by_active_output_std':
            (model_rmse/np.maximum(np.std(measured, axis=0), 1e-12)).tolist(),
    }


def residual_diagnostics(data, prediction):
    active = slice(BASELINE, ACTIVE_END)
    # Positive residual means the model predicts above measurement, matching
    # the earlier C/D/E residual report convention.
    residual = prediction[active]-data['measured'][active]
    result = {}
    for i, output in enumerate(OUTPUTS):
        kernel = np.ones(250)/250  # one-second offline low-pass diagnostic
        slow = np.convolve(residual[:, i], kernel, mode='same')
        trim = slice(125, len(slow)-125)
        total_rmse = float(np.sqrt(np.mean(residual[:, i]**2)))
        slow_rmse = float(np.sqrt(np.mean(slow[trim]**2)))
        result[output] = {
            'mean_deg': float(np.mean(residual[:, i])),
            'rmse_deg': total_rmse,
            'p95_abs_deg': float(np.percentile(np.abs(residual[:, i]), 95)),
            'one_second_lowpass_rmse_deg': slow_rmse,
            'one_second_lowpass_fraction_of_total_rmse': slow_rmse/max(total_rmse, 1e-12),
            'position_correlation': safe_corr(data['measured'][active, i], residual[:, i]),
            'velocity_correlation': safe_corr(data['velocity'][active, i], residual[:, i]),
            'by_velocity_direction': direction_stats(residual[:, i], data['velocity'][active, i]),
        }
    return result


def scaled_waveform_comparison(base, other):
    active = slice(BASELINE, ACTIVE_END)
    ratio = other['scale']/base['scale']
    base_center = base['measured']-base['measured'][BASELINE-1]
    other_center = other['measured']-other['measured'][BASELINE-1]
    scaled = ratio*base_center
    result = metrics(scaled[active], other_center[active])
    normalized_difference = other_center/other['scale']-base_center/base['scale']
    result['per_unit_rmse_deg'] = np.sqrt(
        np.mean(normalized_difference[active]**2, axis=0)).tolist()
    return result


def write_resampled(path, data, legacy_prediction, r1_prediction):
    names = ['time_s', 'reference_big_deg', 'reference_heading_deg',
             'measured_big_joint_deg', 'measured_small_joint_deg', 'measured_heading_deg',
             'command_big', 'command_small', 'current_big_raw', 'current_small_raw',
             'legacy_pred_big_deg', 'legacy_pred_small_deg', 'legacy_pred_heading_deg',
             'r1_pred_big_deg', 'r1_pred_small_deg', 'r1_pred_heading_deg']
    columns = np.column_stack((data['time_s'], data['reference'], data['measured'],
                               data['command'], data['current'], legacy_prediction,
                               r1_prediction))
    with path.open('w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(names)
        writer.writerows(columns)


def analyze(e_dir, r1_dir, f4_dir, f5_dir, frozen_c_report, output):
    captures = {
        'E_cable': load_capture('E_cable', e_dir, EDecoder, 1, 1.),
        'R1_cable_free': load_capture('R1_cable_free', r1_dir, LadderDecoder, 1, 1.),
        'F4_cable_free': load_capture('F4_cable_free', f4_dir, LadderDecoder, 2, 4.),
        'F5_cable_free': load_capture('F5_cable_free', f5_dir, LadderDecoder, 3, 5.),
    }
    frozen_report = json.loads(Path(frozen_c_report).read_text(encoding='utf-8'))
    legacy_model = frozen_report['frozen_C_closed_loop_model']

    expected_scales = {'R1_cable_free': 1., 'F4_cable_free': 4., 'F5_cable_free': 5.}
    e_ref = captures['E_cable']['reference']
    reference_roundtrip = {}
    for name, scale in expected_scales.items():
        error = captures[name]['reference']/scale-e_ref
        reference_roundtrip[name] = {
            'scale': scale,
            'maximum_abs_per_unit_difference_deg': float(np.max(np.abs(error))),
            'rms_per_unit_difference_deg': np.sqrt(np.mean(error**2, axis=0)).tolist(),
        }

    # Fit only R1 coefficients with the old, already-selected structure.
    r1 = captures['R1_cable_free']
    r1_model = fit_frozen_arx(r1['reference'], r1['measured'])

    legacy_eval, r1_eval, legacy_predictions, r1_predictions = {}, {}, {}, {}
    for name, data in captures.items():
        legacy_predictions[name], legacy_eval[name] = evaluate_model(data, legacy_model)
        r1_predictions[name], r1_eval[name] = evaluate_model(data, r1_model)

    phase_diagnostics = {}
    for name, data in captures.items():
        phase_diagnostics[name] = {
            'tracking': segment_metrics(data),
            'current_and_friction': friction_and_bias(data),
            'legacy_frozen_C_residual': residual_diagnostics(data, legacy_predictions[name]),
        }

    e = captures['E_cable']
    r1 = captures['R1_cable_free']
    direct_change = {}
    for i, output_name in enumerate(OUTPUTS):
        active = slice(BASELINE, ACTIVE_END)
        e_track = e['measured'][active, i]-target_outputs(e['reference'])[active, i]
        r_track = r1['measured'][active, i]-target_outputs(r1['reference'])[active, i]
        direct_change[output_name] = {
            'E_tracking_rmse_deg': float(np.sqrt(np.mean(e_track**2))),
            'R1_tracking_rmse_deg': float(np.sqrt(np.mean(r_track**2))),
            'R1_vs_E_tracking_rmse_change_fraction':
                float(np.sqrt(np.mean(r_track**2))/max(np.sqrt(np.mean(e_track**2)), 1e-12)-1),
            'E_vs_R1_measured_waveform_rmse_deg':
                float(np.sqrt(np.mean((e['measured'][active, i]-r1['measured'][active, i])**2))),
        }

    baseline_change = {}
    for i, axis in enumerate(AXES):
        old = float(np.median(e['current'][:BASELINE, i]))
        new = float(np.median(r1['current'][:BASELINE, i]))
        baseline_change[axis] = {
            'E_cable_median_raw': old,
            'R1_cable_free_median_raw': new,
            'cable_free_minus_cable_raw': new-old,
        }

    report = {
        'analysis_status': 'completed_development_offline_comparison',
        'model_status': 'closed_loop_reference_map_only_not_open_loop_MPC_plant',
        'hardware_takeover_allowed': False,
        'timebase': {
            'uniform_period_us': DT_US,
            'uniform_samples': SAMPLES,
            'baseline_samples': BASELINE,
            'excitation_samples': ACTIVE_END-BASELINE,
            'method': 'area-preserving command; linear angle/reference; causal feedback-current timestamp',
        },
        'data_quality_and_provenance': {
            name: data['provenance'] for name, data in captures.items()
        },
        'reference_scale_round_trip_against_E': reference_roundtrip,
        'cable_removal_same_waveform_comparison': {
            'scope': 'E and R1 have the same scheduled reference; PID intended unchanged; start pose/time still confound strict causal attribution',
            'tracking_and_response_change': direct_change,
            'baseline_current_change': baseline_change,
        },
        'phase_diagnostics': phase_diagnostics,
        'cross_amplitude_generalization': {
            'primary_legacy_frozen_C_model': {
                'policy': 'original C coefficients, normalization and structure unchanged for E/R1/F4/F5',
                'frozen_report_path': str(Path(frozen_c_report).resolve()),
                'frozen_report_sha256': sha256(frozen_c_report),
                'autonomous_spectral_radius': legacy_model['autonomous_spectral_radius'],
                'evaluation': legacy_eval,
            },
            'secondary_R1_rebased_same_structure': {
                'policy': 'same preselected 12/4/0 ridge-1 structure, coefficients fit only on R1; post-collection development diagnostic',
                'autonomous_spectral_radius': r1_model['autonomous_spectral_radius'],
                'model': r1_model,
                'evaluation': r1_eval,
            },
            'no_model_amplitude_scaling': {
                'F4_from_4x_R1_measured_response': scaled_waveform_comparison(r1, captures['F4_cable_free']),
                'F5_from_5x_R1_measured_response': scaled_waveform_comparison(r1, captures['F5_cable_free']),
                'F5_from_1p25x_F4_measured_response': scaled_waveform_comparison(captures['F4_cable_free'], captures['F5_cable_free']),
            },
        },
        'interpretation_limits': [
            'raw current baseline combines sensor offset, gravity/holding demand and mechanical preload; it is not a calibrated zero-current measurement',
            'direction gaps are empirical friction/load proxies, not identified Coulomb-friction coefficients',
            'E versus R1 is a strong same-waveform mechanical comparison but start pose, temperature and run order were not randomized',
            'all four records are PID closed-loop reference experiments and do not identify the open-loop MPC plant',
            'F4/F5 are development data; any model selected or changed after seeing them needs a new independent validation waveform',
        ],
    }
    output.mkdir(parents=True, exist_ok=False)
    for name, data in captures.items():
        write_resampled(output/(name+'.csv'), data,
                        legacy_predictions[name], r1_predictions[name])
    (output/'report.json').write_text(
        json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--e-dir', required=True, type=Path)
    parser.add_argument('--r1-dir', required=True, type=Path)
    parser.add_argument('--f4-dir', required=True, type=Path)
    parser.add_argument('--f5-dir', required=True, type=Path)
    parser.add_argument('--frozen-c-report', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output must not exist')
    report = analyze(args.e_dir, args.r1_dir, args.f4_dir, args.f5_dir,
                     args.frozen_c_report, args.output)
    print(json.dumps({
        'analysis_status': report['analysis_status'],
        'model_status': report['model_status'],
        'hardware_takeover_allowed': False,
        'output': str(args.output),
    }, indent=2))


if __name__ == '__main__':
    main()
