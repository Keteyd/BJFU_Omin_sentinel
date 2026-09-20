"""Evaluate D once against the exact C-only frozen closed-loop model.

The validation policy, C report hash and thresholds are fixed before D is
collected.  D never refits the motion model.  Hardware takeover stays disabled.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from identify_cd_reference_offline import (ACTIVE_END, AXES, BASELINE, DT,
                                             OUTPUTS, _metrics, _hold_metrics,
                                             fit_command_current, load_capture)
from identify_dual_reference_offline import _arx_simulate


EXPECTED_C_REPORT_SHA256 = 'F4CADD0D94E98FBA9ED0D61E5B545FDA91D4A52AA2E16B0E9E607B7D0A3D40C6'
MINIMUM_JOINT_IMPROVEMENT = 0.30
CURRENT_GAIN_RELATIVE_DIFFERENCE_LIMIT = 0.10
CURRENT_DELAY_DIFFERENCE_LIMIT_SAMPLES = 1


def _residual_structure(residual):
    result = {}
    frequency = np.fft.rfftfreq(len(residual), DT)
    for column, output in enumerate(OUTPUTS):
        values = residual[:, column]-residual[:, column].mean()
        spectrum = np.abs(np.fft.rfft(values))**2
        total = float(spectrum[1:].sum())
        lag = 25
        autocorrelation = (float(np.corrcoef(values[:-lag], values[lag:])[0, 1])
                           if np.std(values) > 0 else None)
        result[output] = {
            'mean_error_deg': float(residual[:, column].mean()),
            'std_error_deg': float(residual[:, column].std()),
            'autocorrelation_100ms': autocorrelation,
            'power_fraction_below_0_5Hz':
                float(spectrum[(frequency > 0) & (frequency < .5)].sum()/max(total, 1e-20)),
        }
    return result


def validate(c_report_path, d_capture):
    c_report_path = Path(c_report_path)
    c_raw = c_report_path.read_bytes()
    c_hash = hashlib.sha256(c_raw).hexdigest().upper()
    if c_hash != EXPECTED_C_REPORT_SHA256:
        raise ValueError('C report differs from the predeclared frozen model hash')
    c_report = json.loads(c_raw)
    if (c_report.get('capture_status') != 'accepted'
            or c_report.get('model_status') != 'frozen_C_candidate_pending_independent_D'):
        raise ValueError('C report did not freeze an eligible candidate')
    model = c_report['frozen_C_closed_loop_model']
    if (model['structure'] != {'output_lags': 12, 'reference_lags': 4,
                                'reference_delay_samples': 0,
                                'reference_delay_ms': 0.0}
            or model['ridge'] != 1.0):
        raise ValueError('C model structure differs from the A/B-frozen policy')

    d_report, data = load_capture(d_capture)
    if d_report['capture_status'] != 'accepted' or d_report['audit']['phase_set'] != 'D':
        raise ValueError('input is not an accepted phase-D capture')
    measured = data['measured']
    refs = data['reference']
    output_center = np.asarray(model['output_center_deg'])
    output_scale = np.asarray(model['output_scale_deg'])
    reference_center = np.asarray(model['reference_center_deg'])
    reference_scale = np.asarray(model['reference_scale_deg'])
    output = (measured-output_center)/output_scale
    reference_scaled = (refs-reference_center)/reference_scale
    structure = model['structure']
    simulated = _arx_simulate(
        output, reference_scaled, np.asarray(model['normalized_coefficient']),
        BASELINE, len(measured), structure['output_lags'],
        structure['reference_lags'], structure['reference_delay_samples'])
    if simulated is None:
        primary = {
            'finite_full_prediction': False,
            'passed': False,
            'reason': 'nonfinite_frozen_C_prediction',
        }
    else:
        prediction = simulated*output_scale+output_center
        segments = {}
        for name, start, end in (
                ('D_excitation', BASELINE, ACTIVE_END),
                ('D_settle', ACTIVE_END, len(measured)),
                ('D_continuous', BASELINE, len(measured))):
            model_metrics = _metrics(prediction[start:end], measured[start:end])
            hold_metrics = _hold_metrics(measured, start, end)
            improvements = (1-np.asarray(model_metrics['rmse_deg'])/
                            np.maximum(hold_metrics['rmse_deg'], 1e-12))
            segments[name] = {
                'model': model_metrics,
                'hold': hold_metrics,
                'improvement_fraction': improvements.tolist(),
            }
        joint_improvement = segments['D_excitation']['improvement_fraction'][:2]
        primary = {
            'finite_full_prediction': True,
            'minimum_per_joint_improvement_required': MINIMUM_JOINT_IMPROVEMENT,
            'D_excitation_joint_improvement_fraction': joint_improvement,
            'segments': segments,
            'residual_structure_D_excitation': _residual_structure(
                prediction[BASELINE:ACTIVE_END]-measured[BASELINE:ACTIVE_END]),
            'passed': bool(min(joint_improvement) >= MINIMUM_JOINT_IMPROVEMENT),
        }

    # This diagnostic is computed after the frozen motion-model verdict and
    # never changes the C model or primary validation threshold.
    d_current = fit_command_current(data['command'], data['current'])
    comparisons = {}
    for axis in AXES:
        c_item = c_report['frozen_C_command_to_current'][axis]
        d_item = d_current[axis]
        relative_gain = abs(d_item['gain_current_raw_per_command_raw']-
                            c_item['gain_current_raw_per_command_raw']) / max(
                                abs(c_item['gain_current_raw_per_command_raw']), 1e-12)
        delay_difference = abs(d_item['delay_samples']-c_item['delay_samples'])
        comparisons[axis] = {
            'C_gain': c_item['gain_current_raw_per_command_raw'],
            'D_gain': d_item['gain_current_raw_per_command_raw'],
            'relative_gain_difference': relative_gain,
            'relative_gain_difference_limit': CURRENT_GAIN_RELATIVE_DIFFERENCE_LIMIT,
            'C_delay_samples': c_item['delay_samples'],
            'D_delay_samples': d_item['delay_samples'],
            'delay_difference_samples': delay_difference,
            'delay_difference_limit_samples': CURRENT_DELAY_DIFFERENCE_LIMIT_SAMPLES,
            'passed': bool(relative_gain <= CURRENT_GAIN_RELATIVE_DIFFERENCE_LIMIT
                           and delay_difference <= CURRENT_DELAY_DIFFERENCE_LIMIT_SAMPLES),
        }
    current_passed = all(item['passed'] for item in comparisons.values())
    overall = bool(primary['passed'] and current_passed)
    return {
        'validation_status': ('passed_frozen_C_on_independent_D'
                              if overall else 'rejected_frozen_C_on_independent_D'),
        'model_status': ('closed_loop_candidate_passed_C_D_gates'
                         if overall else 'not_identified'),
        'hardware_takeover_allowed': False,
        'policy_frozen_before_D': {
            'C_report_sha256': EXPECTED_C_REPORT_SHA256,
            'minimum_per_joint_improvement': MINIMUM_JOINT_IMPROVEMENT,
            'current_gain_relative_difference_limit': CURRENT_GAIN_RELATIVE_DIFFERENCE_LIMIT,
            'current_delay_difference_limit_samples': CURRENT_DELAY_DIFFERENCE_LIMIT_SAMPLES,
            'D_motion_model_refit_allowed': False,
        },
        'D_capture_audit': d_report['audit'],
        'primary_frozen_C_motion_validation': primary,
        'post_primary_D_command_to_current_diagnostic': {
            'D_fit': d_current,
            'C_D_comparison': comparisons,
            'passed': current_passed,
        },
        'blockers': [
            'closed_loop_reference_map_is_not_an_open_loop_MPC_plant',
            'feedback_current_is_not_calibrated_torque',
            'hardware_takeover_requires_separate_model_and_readiness_work',
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('d_capture', type=Path)
    parser.add_argument('--c-report', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output must not exist')
    report = validate(args.c_report, args.d_capture)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'report.json').write_text(
        json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({
        'validation_status': report['validation_status'],
        'model_status': report['model_status'],
        'hardware_takeover_allowed': False,
        'output': str(args.output),
    }, indent=2))
    return 0 if report['validation_status'].startswith('passed') else 2


if __name__ == '__main__':
    raise SystemExit(main())
