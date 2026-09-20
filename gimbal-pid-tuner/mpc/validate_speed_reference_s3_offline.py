"""Apply the unchanged frozen S1 speed model and prospective gates once to S3."""
from __future__ import annotations

import argparse
import bisect
import json
from pathlib import Path

import numpy as np

from identify_speed_reference_offline import (ACTIVE, evaluate, sha256)
from validate_speed_reference_offline import apply_gate


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PLAN = Path(__file__).with_name('speed_reference_s3_plan.json')
FROZEN_PLAN_SHA256 = 'BF41B63E5FA37AE20F065BB69603BF496E3E75CC5C4828D5A616F90267828B19'
EXPECTED_BUILD = 0x59490801
ALLOWED_DIAGNOSTIC_ISSUES = {'sample_count_not_9001', 'irregular_sample_interval'}


def _time_coverage(values, report, plan):
    rule = plan['prospective_S3_acceptance_gate']['data_quality']['time_domain_coverage']
    trace = np.asarray(values['trace_us'], dtype=np.int64)
    tick = np.asarray(values['tick_ms'], dtype=np.uint64)
    interval = np.diff(trace)
    initial = report.get('initial_metadata') or {}
    terminal = report.get('metadata') or {}
    if not len(trace) or not initial:
        return {'available': False}, False
    elapsed = int((int(tick[-1])-int(initial.get('start_ms', 0))) & 0xffffffff)
    span = int(trace[-1]-trace[0])
    worst_left = worst_right = 0
    bracketed = True
    start_us = round(rule['active_20ms_grid_start_s']*1e6)
    end_us = round(rule['active_20ms_grid_end_s']*1e6)
    for target in range(start_us, end_us, 20000):
        index = bisect.bisect_left(trace, target)
        if index == 0 or index == len(trace):
            bracketed = False
            break
        worst_left = max(worst_left, target-int(trace[index-1]))
        worst_right = max(worst_right, int(trace[index])-target)
    checks = {
        'available': True,
        'trace_strictly_increasing': bool(len(interval) and np.all(interval > 0)),
        'source_interval_at_most_10000us': bool(
            len(interval) and np.all(interval <= rule['source_interval_us_maximum'])),
        'first_trace_us': bool(0 <= trace[0] <= rule['first_trace_us_maximum']),
        'last_trace_us': bool(rule['last_trace_us_minimum'] <= trace[-1]
                              <= rule['last_trace_us_maximum']),
        'trace_span_us': bool(span >= rule['trace_span_us_minimum']),
        'terminal_elapsed_ms': bool(rule['terminal_elapsed_ms_minimum'] <= elapsed
                                    <= rule['terminal_elapsed_ms_maximum']),
        'terminal_metadata_count_matches_rows': bool(terminal.get('count') == len(trace)),
        'active_20ms_grid_bracketed': bool(
            bracketed and worst_left <= rule['maximum_nearest_sample_distance_each_side_us']
            and worst_right <= rule['maximum_nearest_sample_distance_each_side_us']),
    }
    detail = {
        'checks': checks,
        'actual_records_diagnostic_only': int(len(trace)),
        'nominal_records': int(plan['nominal_records']),
        'record_shortfall_diagnostic_only': int(plan['nominal_records']-len(trace)),
        'first_trace_us': int(trace[0]), 'last_trace_us': int(trace[-1]),
        'trace_span_us': span, 'terminal_elapsed_ms': elapsed,
        'minimum_interval_us': int(interval.min()) if len(interval) else None,
        'maximum_interval_us': int(interval.max()) if len(interval) else None,
        'maximum_left_sample_distance_us': worst_left if bracketed else None,
        'maximum_right_sample_distance_us': worst_right if bracketed else None,
    }
    return detail, bool(all(checks.values()))


def data_gate(report, values, plan):
    raw_issues = set(report.get('diagnostic_quality_issues', report.get('quality_issues', [])))
    terminal = report.get('last_status') or {}
    profile = report.get('profile') or {}
    checks = {
        'download_complete': report.get('download_complete') is True,
        'error_null': report.get('error') is None,
        'only_permitted_generic_quality_labels': not bool(
            raw_issues-ALLOWED_DIAGNOSTIC_ISSUES),
        'effective_quality_issues_empty': not bool(report.get('quality_issues')),
        'crc_errors_zero': report.get('crc_errors') == 0,
        'terminal_completed_ok': terminal.get('phase') == 5 and terminal.get('reason') == 0,
        'firmware_stream_quality_clear': report.get('firmware_stream_quality_latched') is False,
        'feedback_failure_null': report.get('feedback_failure') is None,
        'in_stream_discarded_wire_bytes_zero':
            report.get('in_stream_discarded_wire_bytes') == 0,
        'profile_identity': (
            profile.get('build') == EXPECTED_BUILD and profile.get('profile') == 5
            and profile.get('reverse') == 2 and profile.get('samples') == 9001
            and profile.get('period_ms') == 4 and profile.get('duration_ms') == 36000),
        'reference_plan_hash': report.get('frozen_reference_plan_sha256') == FROZEN_PLAN_SHA256,
    }
    can_checks = {}
    for axis in ('big', 'small'):
        item = (report.get('can_summary') or {}).get(axis) or {}
        counters = item.get('interval_counter_sums') or {}
        can_checks[axis] = bool(
            item.get('counter_saturated') is False
            and counters.get('failed') == 0 and counters.get('errors') == 0
            and counters.get('aborted') == 0
            and item.get('incomplete_coverage_intervals_after_first') == 0)
    timing, timing_passed = _time_coverage(values, report, plan)
    checks['CAN_both_axes'] = bool(all(can_checks.values()))
    checks['time_domain_coverage'] = timing_passed
    return {'checks': checks, 'CAN_checks': can_checks, 'timing': timing}, bool(all(checks.values()))


def load_s3(capture, plan):
    capture = Path(capture)
    report = json.loads((capture/'report.json').read_text(encoding='utf-8'))
    values = np.genfromtxt(capture/'samples.csv', delimiter=',', names=True)
    if values.shape == ():
        values = np.asarray([values], dtype=values.dtype)
    audit, passed = data_gate(report, values, plan)
    source = {
        'capture': str(capture.resolve()),
        'raw_sha256': sha256(capture/'raw.bin'),
        'samples_csv_sha256': sha256(capture/'samples.csv'),
    }
    if not passed:
        return None, audit, source
    trace = np.asarray(values['trace_us'], dtype=float)*1e-6
    trace -= trace[0]
    grid = np.arange(.020, 36.0001, .020)
    interpolate = lambda name: np.interp(grid, trace, np.asarray(values[name], dtype=float))
    reference = np.column_stack((interpolate('big_reference_offset_cdeg'),
                                 interpolate('small_heading_reference_offset_cdeg')))/100
    measured = np.column_stack((interpolate('big_feedback_rpm')*6,
                                interpolate('gyro_z_rad_s')*180/np.pi))
    if len(grid) != 1800 or not np.isfinite(reference).all() or not np.isfinite(measured).all():
        raise ValueError('failed to form the frozen finite 36 s/20 ms S3 grid')
    return {'time': grid, 'reference': reference, 'measured': measured}, audit, source


def validate(model_path, capture, plan_path=DEFAULT_PLAN):
    model_path, plan_path = Path(model_path), Path(plan_path)
    if sha256(plan_path) != FROZEN_PLAN_SHA256:
        raise ValueError('S3 plan differs from the pre-collection frozen artifact')
    plan = json.loads(plan_path.read_text(encoding='utf-8'))
    expected_model_hash = plan['unchanged_frozen_model']['report_sha256']
    if sha256(model_path) != expected_model_hash:
        raise ValueError('S1 model differs from the S3 frozen plan')
    frozen = json.loads(model_path.read_text(encoding='utf-8'))
    if frozen.get('status') != 'S1_SPEED_REFERENCE_MODEL_FROZEN_PENDING_S2':
        raise ValueError('model file is not the unchanged pre-S2 S1 artifact')
    data, audit, source = load_s3(capture, plan)
    if data is None:
        return {
            'status': 'S3_REJECTED_BY_FROZEN_DATA_GATE',
            'hardware_takeover_allowed': False,
            'frozen_plan_sha256': FROZEN_PLAN_SHA256,
            'frozen_model_report_sha256': expected_model_hash,
            'S3_source': source,
            'data_gate': audit,
            'all_frozen_gates_passed': False,
            'model_evaluation_skipped': True,
        }
    item = frozen['frozen_model']
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
    model_policy = plan['prospective_S3_acceptance_gate']['model_quality']
    checks, model_passed = apply_gate(metrics, model_policy)
    passed = bool(model_passed)
    return {
        'status': ('FROZEN_SPEED_REFERENCE_MODEL_PASSED_S3' if passed else
                   'FROZEN_SPEED_REFERENCE_MODEL_REJECTED_BY_S3'),
        'hardware_takeover_allowed': False,
        'frozen_plan_sha256': FROZEN_PLAN_SHA256,
        'frozen_model_report': str(model_path.resolve()),
        'frozen_model_report_sha256': expected_model_hash,
        'S3_source': source,
        'data_gate': audit,
        'S3_rolling_metrics': metrics,
        'model_gate_checks': checks,
        'all_frozen_gates_passed': passed,
        'policy': plan['prospective_S3_acceptance_gate'],
        'interpretation': ('independent validation of the retained-PID closed-loop '
                           'speed-reference predictor; no voltage/current-to-mechanics '
                           'claim and no hardware takeover'),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--plan', type=Path, default=DEFAULT_PLAN)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = validate(args.model, args.capture, args.plan)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['all_frozen_gates_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
