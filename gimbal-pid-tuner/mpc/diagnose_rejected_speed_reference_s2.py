"""Post-gate diagnostic for the record-short S2 trace; cannot change its formal rejection."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from identify_speed_reference_offline import ACTIVE, DT, evaluate, sha256
from validate_speed_reference_offline import apply_gate


def numerical_data(capture):
    capture = Path(capture)
    report = json.loads((capture/'report.json').read_text(encoding='utf-8'))
    values = np.genfromtxt(capture/'samples.csv', delimiter=',', names=True)
    trace = np.asarray(values['trace_us'], dtype=float)*1e-6
    trace -= trace[0]
    grid = np.arange(DT, 36.+DT/2, DT)
    interpolate = lambda name: np.interp(grid, trace, np.asarray(values[name], dtype=float))
    return report, {
        'time': grid,
        'reference': np.column_stack((interpolate('big_reference_offset_cdeg'),
                                      interpolate('small_heading_reference_offset_cdeg')))/100,
        'measured': np.column_stack((interpolate('big_feedback_rpm')*6,
                                     interpolate('gyro_z_rad_s')*180/np.pi)),
    }


def diagnose(model_path, policy_path, capture):
    model_path, policy_path, capture = map(Path, (model_path, policy_path, capture))
    model = json.loads(model_path.read_text(encoding='utf-8'))
    policy = json.loads(policy_path.read_text(encoding='utf-8'))
    if sha256(model_path) != policy['frozen_model_report_sha256']:
        raise ValueError('frozen model hash differs from pre-S2 policy')
    report, data = numerical_data(capture)
    item = model['frozen_model']
    definition = item['structure']
    structure = (definition['output_lags'], definition['reference_lags'],
                 definition['reference_delay_samples'], definition['ridge'])
    coefficient = np.asarray(
        item['normalized_coefficient_feature_rows_by_output_columns'], dtype=float)
    scales = {
        'output_baseline': np.asarray(list(item['output_baseline_dps'].values())),
        'output_scale': np.asarray(list(item['output_scale_dps'].values())),
        'input_scale': np.asarray(list(item['input_scale_dps'].values())),
    }
    metrics = evaluate(data, coefficient, structure, scales, ACTIVE)
    numerical_checks, numerical_pass = apply_gate(
        metrics, model['frozen_S2_acceptance_gate'])
    timing = report['capture_timing']
    data_checks = {
        'download_complete': report.get('download_complete') is True,
        'quality_issues_empty': not report.get('quality_issues'),
        'crc_errors_zero': report.get('crc_errors') == 0,
        'terminal_completed_ok': (report.get('last_status', {}).get('phase') == 5
                                  and report.get('last_status', {}).get('reason') == 0),
        'record_shortfall_at_most_5': timing.get('record_shortfall', 999) <= 5,
        'full_duration_timing_accepted': timing.get('full_duration_timing_accepted') is True,
        'firmware_stream_quality_clear': not report.get('firmware_stream_quality_latched'),
    }
    return {
        'status': 'S2_FORMALLY_REJECTED_BY_FROZEN_DATA_GATE',
        'all_frozen_gates_passed': False,
        'must_not_repeat_S2': True,
        'must_not_relax_frozen_gate': True,
        'hardware_takeover_allowed': False,
        'frozen_model_report_sha256': sha256(model_path),
        'frozen_policy_sha256': sha256(policy_path),
        'S2_raw_sha256': sha256(capture/'raw.bin'),
        'formal_data_gate_checks': data_checks,
        'formal_failure': {
            'records': report.get('records'),
            'nominal_records': timing.get('nominal_records'),
            'record_shortfall': timing.get('record_shortfall'),
            'frozen_maximum_record_shortfall': 5,
            'quality_issues': report.get('quality_issues'),
            'trace_span_us': timing.get('trace_span_us'),
        },
        'post_rejection_diagnostic_only': {
            'method': ('unchanged frozen model and 20 ms interpolation applied to the available '
                       'full-span samples; no coefficient, scale, structure, polarity or gate change'),
            'rolling_metrics': metrics,
            'numerical_threshold_checks': numerical_checks,
            'numerical_thresholds_would_pass_if_data_gate_had_passed': numerical_pass,
            'cannot_override_formal_rejection': True,
        },
        'interpretation': ('S2 cannot independently accept the model because the predeclared data gate '
                           'failed. Diagnostic dynamics may guide a future newly designed experiment, '
                           'but cannot relabel or repeat this frozen S2 validation.'),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--policy', type=Path, required=True)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = diagnose(args.model, args.policy, args.capture)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
