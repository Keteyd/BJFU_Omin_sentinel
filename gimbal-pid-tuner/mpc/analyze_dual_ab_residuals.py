"""Develop the next identification design from completed dual-reference A/B data.

This is an offline diagnostic.  A and B are explicitly consumed as development
data, so neither remains eligible as an independent validation set for a model
selected with this report.  No hardware interface is imported or authorized.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.signal import coherence, find_peaks, savgol_filter, welch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from yaw_dual_trace_protocol import Decoder
from identify_dual_reference_offline import (AXES, OUTPUTS, _arx_feature,
                                              _arx_simulate,
                                              _arx_spectral_radius)
from preview_dual_reference_excitation import DEFAULT_PLAN
from validate_dual_reference_offline import (_read_verified_capture,
                                              EXCITATION)


DT = .004
FS = 1/DT
QUANTUM_DEG = 360/8192


def _prediction(model, data, start=EXCITATION[0], end=EXCITATION[1]):
    structure = model['structure']
    output_center = np.asarray(model['output_center_deg'])
    output_scale = np.asarray(model['output_scale_deg'])
    reference_center = np.asarray(model['reference_center_deg'])
    reference_scale = np.asarray(model['reference_scale_deg'])
    normalized_output = (data['measured']-output_center)/output_scale
    normalized_reference = (data['reference']-reference_center)/reference_scale
    simulated = _arx_simulate(
        normalized_output, normalized_reference,
        np.asarray(model['normalized_coefficient']), start, end,
        structure['output_lags'], structure['reference_lags'],
        structure['reference_delay_samples'],
    )
    if simulated is None:
        raise ValueError('frozen model produced a nonfinite cross-phase prediction')
    return simulated[start:end]*output_scale+output_center


def _autocorrelation(values, lag):
    values = values-values.mean()
    if lag >= len(values) or np.std(values) == 0:
        return None
    return float(np.corrcoef(values[:-lag], values[lag:])[0, 1])


def _spectral_summary(residual, references):
    report = {}
    spectra = {}
    for column, output in enumerate(OUTPUTS):
        values = residual[:, column]-residual[:, column].mean()
        frequency, psd = welch(values, fs=FS, window='hann', nperseg=1024,
                               noverlap=512, detrend='linear')
        total = np.trapezoid(psd, frequency)
        bands = {}
        for name, low, high in (
                ('below_0_5_hz', 0, .5), ('0_5_to_2_hz', .5, 2),
                ('2_to_10_hz', 2, 10), ('above_10_hz', 10, FS/2+.1)):
            selected = (frequency >= low) & (frequency < high)
            bands[name] = float(np.trapezoid(psd[selected], frequency[selected])
                                / max(total, 1e-20))
        eligible = (frequency >= .05) & (frequency <= 20)
        candidate = np.flatnonzero(eligible)
        local, _ = find_peaks(psd[candidate])
        peaks = candidate[local]
        if len(peaks):
            peaks = peaks[np.argsort(psd[peaks])[-5:][::-1]]
        coherence_report = {}
        for ref_column, axis in enumerate(AXES):
            cf, cxy = coherence(values, references[:, ref_column], fs=FS,
                                nperseg=1024, noverlap=512)
            mask = (cf >= .05) & (cf <= 5)
            index = np.flatnonzero(mask)[np.argmax(cxy[mask])]
            coherence_report[axis+'_reference'] = {
                'maximum_0_05_to_5_hz': float(cxy[index]),
                'frequency_hz': float(cf[index]),
            }
        report[output] = {
            'mean_deg': float(residual[:, column].mean()),
            'rmse_deg': float(np.sqrt(np.mean(residual[:, column]**2))),
            'maximum_abs_deg': float(np.max(np.abs(residual[:, column]))),
            'autocorrelation': {
                '4_ms': _autocorrelation(values, 1),
                '20_ms': _autocorrelation(values, 5),
                '100_ms': _autocorrelation(values, 25),
                '1_s': _autocorrelation(values, 250),
            },
            'psd_fraction': bands,
            'dominant_peaks_hz': [float(frequency[index]) for index in peaks],
            'reference_coherence': coherence_report,
        }
        spectra[output] = (frequency, psd)
    return report, spectra


def _raw_joint_positions(capture):
    capture = Path(capture)
    saved = json.loads((capture/'report.json').read_text(encoding='utf-8'))
    decoder = Decoder(saved['trial_id'])
    decoder.feed((capture/'raw.bin').read_bytes())
    return {
        'big': np.asarray([row['big_raw'] for row in decoder.rows], dtype=np.int64),
        'small': np.asarray([row['small_raw'] for row in decoder.rows], dtype=np.int64),
    }


def _friction_signature(data, raw_positions):
    result = {}
    for column, axis in enumerate(AXES):
        position = data['measured'][:, column]
        velocity = savgol_filter(position, 51, 3, deriv=1, delta=DT)
        acceleration = savgol_filter(position, 51, 3, deriv=2, delta=DT)
        command = data['command'][:, column]
        current = data['current'][:, column]
        baseline_command = float(np.median(command[:500]))
        centered_command = command-baseline_command
        positive = velocity > .10
        negative = velocity < -.10
        stationary = np.abs(velocity) < .03
        moving = positive | negative
        design = np.column_stack((
            np.ones(np.count_nonzero(moving)), velocity[moving],
            np.sign(velocity[moving]), acceleration[moving],
        ))
        coefficients = np.linalg.lstsq(design, centered_command[moving], rcond=None)[0]
        raw_delta = np.diff(raw_positions[axis].astype(float))
        raw_delta = (raw_delta+4096) % 8192-4096
        preceding_still = np.zeros(len(raw_delta), dtype=bool)
        zero_run = 0
        for index, delta in enumerate(raw_delta):
            preceding_still[index] = zero_run >= 4
            zero_run = zero_run+1 if delta == 0 else 0
        breakaway = np.flatnonzero(preceding_still & (raw_delta != 0))+1
        breakaway = breakaway[(breakaway >= 500) & (breakaway < 4000)]
        breakaway_effort = np.abs(centered_command[np.minimum(breakaway, len(command)-1)])
        result[axis] = {
            'encoder_quantum_deg': QUANTUM_DEG,
            'raw_zero_increment_fraction': float(np.mean(raw_delta == 0)),
            'smoothed_stationary_fraction_abs_velocity_below_0_03_dps': float(np.mean(stationary)),
            'moving_sample_counts': {
                'positive': int(np.count_nonzero(positive)),
                'negative': int(np.count_nonzero(negative)),
            },
            'baseline_command_software_units': baseline_command,
            'median_centered_command_by_motion': {
                'positive_velocity': float(np.median(centered_command[positive])),
                'negative_velocity': float(np.median(centered_command[negative])),
                'stationary': float(np.median(centered_command[stationary])),
            },
            'median_current_raw_by_motion': {
                'positive_velocity': float(np.median(current[positive])),
                'negative_velocity': float(np.median(current[negative])),
                'stationary': float(np.median(current[stationary])),
            },
            'diagnostic_command_regression_moving_only': {
                'intercept': float(coefficients[0]),
                'viscous_like_per_dps': float(coefficients[1]),
                'direction_like_software_units': float(coefficients[2]),
                'acceleration_like_per_dps2': float(coefficients[3]),
                'warning': ('Closed-loop effort contains tracking correction and load; '
                            'the direction term is a friction signature, not Coulomb torque.'),
            },
            'breakaway_after_at_least_four_zero_count_intervals': {
                'events': int(len(breakaway_effort)),
                'absolute_centered_command_p50': (float(np.percentile(breakaway_effort, 50))
                                                   if len(breakaway_effort) else None),
                'absolute_centered_command_p90': (float(np.percentile(breakaway_effort, 90))
                                                   if len(breakaway_effort) else None),
            },
        }
    return result


def _fit_fixed_arx(source, na, nb, delay, ridge):
    start, end = EXCITATION
    measured = source['measured']
    refs = source['reference']
    output_center = measured[start:end].mean(axis=0)
    output_scale = np.maximum(measured[start:end].std(axis=0), QUANTUM_DEG)
    reference_center = refs[start:end].mean(axis=0)
    reference_scale = np.maximum(refs[start:end].std(axis=0), .01)
    output = (measured-output_center)/output_scale
    reference = (refs-reference_center)/reference_scale
    history = max(na, delay+nb-1)
    first = max(start, history)
    x = np.asarray([_arx_feature(output, reference, index, na, nb, delay)
                    for index in range(first, end)])
    y = output[first:end]
    penalty = np.eye(x.shape[1]); penalty[-1, -1] = 0
    coefficient = np.linalg.solve(
        x.T@x+len(x)*ridge*penalty,
        x.T@y,
    )
    radius = _arx_spectral_radius(coefficient, measured.shape[1], na)
    return {
        'structure': {'output_lags': na, 'reference_lags': nb,
                      'reference_delay_samples': delay},
        'ridge': ridge, 'coefficient': coefficient,
        'output_center': output_center, 'output_scale': output_scale,
        'reference_center': reference_center, 'reference_scale': reference_scale,
        'spectral_radius': radius,
    }


def _cross_improvement(model, target):
    start, end = EXCITATION
    output = (target['measured']-model['output_center'])/model['output_scale']
    refs = (target['reference']-model['reference_center'])/model['reference_scale']
    s = model['structure']
    prediction = _arx_simulate(
        output, refs, model['coefficient'], start, end, s['output_lags'],
        s['reference_lags'], s['reference_delay_samples'],
    )
    if prediction is None:
        return None
    prediction = prediction[start:end]*model['output_scale']+model['output_center']
    measured = target['measured'][start:end]
    hold = np.repeat(target['measured'][start-1:start], end-start, axis=0)
    model_rmse = np.sqrt(np.mean((prediction-measured)**2, axis=0))
    hold_rmse = np.sqrt(np.mean((hold-measured)**2, axis=0))
    return {
        'model_rmse_deg': model_rmse.tolist(),
        'hold_rmse_deg': hold_rmse.tolist(),
        'improvement_fraction': (1-model_rmse/np.maximum(hold_rmse, 1e-12)).tolist(),
    }


def _fixed_prediction(model, target):
    start, end = EXCITATION
    output = (target['measured']-model['output_center'])/model['output_scale']
    refs = (target['reference']-model['reference_center'])/model['reference_scale']
    s = model['structure']
    prediction = _arx_simulate(
        output, refs, model['coefficient'], start, end, s['output_lags'],
        s['reference_lags'], s['reference_delay_samples'],
    )
    if prediction is None:
        raise ValueError('common fixed structure produced nonfinite prediction')
    return prediction[start:end]*model['output_scale']+model['output_center']


def _common_structure_grid(a, b):
    candidates = []
    for na in (2, 4, 8, 12):
        for nb in (4, 8, 16, 24):
            for delay in (0, 1, 2, 3):
                for ridge in (1e-6, 1e-4, 1e-2, 1.):
                    models = [_fit_fixed_arx(source, na, nb, delay, ridge)
                              for source in (a, b)]
                    if any(not np.isfinite(model['spectral_radius'])
                           or model['spectral_radius'] >= 1 for model in models):
                        continue
                    ab = _cross_improvement(models[0], b)
                    ba = _cross_improvement(models[1], a)
                    if ab is None or ba is None:
                        continue
                    joint_values = ab['improvement_fraction'][:2]+ba['improvement_fraction'][:2]
                    candidates.append({
                        'structure': models[0]['structure'],
                        'ridge': ridge,
                        'spectral_radius': {'fit_A': models[0]['spectral_radius'],
                                            'fit_B': models[1]['spectral_radius']},
                        'A_to_B': ab, 'B_to_A': ba,
                        'worst_joint_improvement_fraction': float(min(joint_values)),
                        'mean_joint_improvement_fraction': float(np.mean(joint_values)),
                    })
    candidates.sort(key=lambda item: (item['worst_joint_improvement_fraction'],
                                      item['mean_joint_improvement_fraction']), reverse=True)
    best = candidates[0] if candidates else None
    return {
        'candidate_count_stable_in_both_fits': len(candidates),
        'selection_role': ('development diagnostic using both A and B; a selected structure '
                           'must be frozen before new C/D validation'),
        'best_by_worst_cross_phase_joint_improvement': best,
        'top_10': candidates[:10],
        'existing_30_percent_gate_would_pass': bool(
            best and best['worst_joint_improvement_fraction'] >= .30
        ),
    }


def analyze(a_capture, b_capture, a_report_path, b_report_path, plan_path=DEFAULT_PLAN):
    a = _read_verified_capture(a_capture, 'A', plan_path)
    b = _read_verified_capture(b_capture, 'B', plan_path)
    a_report_path, b_report_path = Path(a_report_path), Path(b_report_path)
    a_report = json.loads(a_report_path.read_text(encoding='utf-8'))
    b_report = json.loads(b_report_path.read_text(encoding='utf-8'))
    if a_report['audit']['raw_sha256'] != a['audit']['raw_sha256']:
        raise ValueError('phase-A report/raw mismatch')
    if b_report['audit']['raw_sha256'] != b['audit']['raw_sha256']:
        raise ValueError('phase-B report/raw mismatch')
    cross = {}
    spectra = {}
    residual_rows = {}
    for name, source_report, target in (
            ('frozen_A_on_B', a_report, b), ('frozen_B_on_A', b_report, a)):
        prediction = _prediction(source_report['closed_loop_state_model'], target)
        residual = prediction-target['measured'][EXCITATION[0]:EXCITATION[1]]
        summary, spectrum = _spectral_summary(
            residual, target['reference'][EXCITATION[0]:EXCITATION[1]]
        )
        cross[name] = summary
        spectra[name] = spectrum
        residual_rows[name] = (target['time_s'][EXCITATION[0]:EXCITATION[1]],
                               target['measured'][EXCITATION[0]:EXCITATION[1]],
                               prediction, residual)
    grid = _common_structure_grid(a, b)
    best = grid['best_by_worst_cross_phase_joint_improvement']
    common_residuals = {}
    if best:
        s = best['structure']
        for name, source, target in (('fixed_common_A_on_B', a, b),
                                     ('fixed_common_B_on_A', b, a)):
            model = _fit_fixed_arx(
                source, s['output_lags'], s['reference_lags'],
                s['reference_delay_samples'], best['ridge'],
            )
            prediction = _fixed_prediction(model, target)
            residual = prediction-target['measured'][EXCITATION[0]:EXCITATION[1]]
            summary, spectrum = _spectral_summary(
                residual, target['reference'][EXCITATION[0]:EXCITATION[1]]
            )
            common_residuals[name] = summary
            spectra[name] = spectrum
            residual_rows[name] = (
                target['time_s'][EXCITATION[0]:EXCITATION[1]],
                target['measured'][EXCITATION[0]:EXCITATION[1]], prediction, residual,
            )
    findings = []
    if any(cross[direction][output]['autocorrelation']['100_ms'] > .5
           for direction in cross for output in OUTPUTS):
        findings.append('cross_phase_residuals_are_strongly_time_correlated')
    if any(cross[direction][output]['psd_fraction']['below_0_5_hz'] > .25
           for direction in cross for output in OUTPUTS[:2]):
        findings.append('joint_residual_energy_is_concentrated_at_low_frequency')
    if not grid['existing_30_percent_gate_would_pass']:
        findings.append('no_common_tested_arx_structure_passes_the_existing_cross_phase_gate')
    else:
        findings.append('a_common_development_structure_exists_but_requires_new_independent_data')
    result = {
        'analysis_status': 'A_and_B_consumed_for_next_design_development',
        'hardware_execution_authorized': False,
        'hardware_takeover_allowed': False,
        'source_hashes': {
            'phase_A_raw_sha256': a['audit']['raw_sha256'],
            'phase_B_raw_sha256': b['audit']['raw_sha256'],
            'phase_A_model_report_sha256': hashlib.sha256(a_report_path.read_bytes()).hexdigest(),
            'phase_B_model_report_sha256': hashlib.sha256(b_report_path.read_bytes()).hexdigest(),
        },
        'cross_phase_selected_model_residuals': cross,
        'quantization_and_friction_signatures': {
            'phase_A': _friction_signature(a, _raw_joint_positions(a_capture)),
            'phase_B': _friction_signature(b, _raw_joint_positions(b_capture)),
            'interpretation_limit': ('Controller effort, gravity/load and tracking error remain '
                                     'confounded; no value here is calibrated Coulomb torque.'),
        },
        'common_arx_structure_development_grid': grid,
        'best_common_structure_cross_residuals': common_residuals,
        'findings': findings,
        'design_implications': [
            'Do not reuse A or B as independent validation after this structure comparison.',
            'Add lower-frequency dwell/reversal content to expose stick-slip and bias separately.',
            'Add modest higher-frequency content to distinguish delay and short-lag dynamics.',
            'Use a longer excitation and require one fixed structure in both new phase directions.',
            'Keep the new C/D plan offline-only until firmware, host and field review are complete.',
        ],
        'recommended_next_artifact': 'dual_reference_excitation_plan_cd.json',
    }
    return result, spectra, residual_rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase_a_capture', type=Path)
    parser.add_argument('phase_b_capture', type=Path)
    parser.add_argument('--phase-a-report', required=True, type=Path)
    parser.add_argument('--phase-b-report', required=True, type=Path)
    parser.add_argument('--plan', type=Path, default=DEFAULT_PLAN)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output must not exist')
    report, spectra, residual_rows = analyze(
        args.phase_a_capture, args.phase_b_capture, args.phase_a_report,
        args.phase_b_report, args.plan,
    )
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'report.json').write_text(
        json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8'
    )
    for direction, rows in residual_rows.items():
        np.savetxt(
            args.output/(direction+'_residuals.csv'), np.column_stack(rows),
            delimiter=',', comments='', fmt='%.9g',
            header=('time_s,measured_big_deg,measured_small_deg,measured_heading_deg,'
                    'predicted_big_deg,predicted_small_deg,predicted_heading_deg,'
                    'residual_big_deg,residual_small_deg,residual_heading_deg'),
        )
    for direction, outputs in spectra.items():
        frequency = outputs[OUTPUTS[0]][0]
        np.savetxt(
            args.output/(direction+'_residual_psd.csv'),
            np.column_stack((frequency, *[outputs[name][1] for name in OUTPUTS])),
            delimiter=',', comments='', fmt='%.9g',
            header='frequency_hz,big_residual_psd,small_residual_psd,heading_residual_psd',
        )
    print(json.dumps({
        'analysis_status': report['analysis_status'],
        'findings': report['findings'],
        'best_common_structure': report['common_arx_structure_development_grid'][
            'best_by_worst_cross_phase_joint_improvement'],
        'hardware_takeover_allowed': False,
        'output': str(args.output),
    }, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
