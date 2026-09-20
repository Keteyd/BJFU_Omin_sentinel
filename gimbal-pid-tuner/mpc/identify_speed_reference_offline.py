"""Fit and freeze the S1 direct-speed closed-loop predictor; never controls hardware."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


DT = .020
GRID = tuple((na, nb, delay, ridge)
             for na in (1, 2, 4) for nb in (1, 2, 4) for delay in (0, 1, 2)
             for ridge in (1e-5, 1e-3, 1e-2, .1))
TRAIN = (3., 17.)
SELECT = (17., 24.)
INTERNAL_TEST = (24., 31.)
ACTIVE = (3., 31.)
HORIZONS = (5, 10, 25)  # 100, 200 and 500 ms on the MPC grid.
OUTPUTS = ('big_motor_speed_dps', 'small_inertial_heading_rate_dps')
INPUTS = ('big_motor_speed_reference_dps', 'small_inertial_heading_rate_reference_dps')


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest().upper()


def load_capture(capture, phase_set=0):
    capture = Path(capture)
    report = json.loads((capture/'report.json').read_text(encoding='utf-8'))
    if (report.get('error') or not report.get('download_complete')
            or report.get('quality_issues') or report.get('crc_errors')
            or report.get('firmware_stream_quality_latched')
            or not report.get('capture_timing', {}).get('full_duration_timing_accepted')
            or report.get('capture_timing', {}).get('record_shortfall', 999) > 5
            or report.get('last_status', {}).get('phase') != 5
            or report.get('last_status', {}).get('reason') != 0
            or report.get('profile', {}).get('build') != 0x59490702
            or report.get('profile', {}).get('profile') != 5
            or report.get('profile', {}).get('reverse') != phase_set):
        raise ValueError('capture is not an accepted 0x59490702 speed-reference trace')
    values = np.genfromtxt(capture/'samples.csv', delimiter=',', names=True)
    trace = np.asarray(values['trace_us'], dtype=float)*1e-6
    trace -= trace[0]
    grid = np.arange(DT, 36.+DT/2, DT)
    interpolate = lambda name: np.interp(grid, trace, np.asarray(values[name], dtype=float))
    reference = np.column_stack((interpolate('big_reference_offset_cdeg'),
                                 interpolate('small_heading_reference_offset_cdeg')))/100
    measured = np.column_stack((interpolate('big_feedback_rpm')*6,
                                interpolate('gyro_z_rad_s')*180/np.pi))
    if len(grid) != 1800 or not np.isfinite(reference).all() or not np.isfinite(measured).all():
        raise ValueError('failed to form a finite 36 s/20 ms identification grid')
    return {
        'time': grid, 'reference': reference, 'measured': measured,
        'report': report, 'capture': str(capture.resolve()),
        'raw_sha256': sha256(capture/'raw.bin'),
        'samples_csv_sha256': sha256(capture/'samples.csv'),
    }


def indices(data, bounds):
    return np.flatnonzero((data['time'] >= bounds[0]-1e-9)
                          & (data['time'] < bounds[1]-1e-9))


def scales_for(data, bounds=TRAIN):
    baseline = indices(data, (0., 3.))
    fit = indices(data, bounds)
    return {
        'output_baseline': np.median(data['measured'][baseline], axis=0),
        'output_scale': np.maximum(np.std(data['measured'][fit], axis=0), .1),
        'input_scale': np.maximum(np.std(data['reference'][fit], axis=0), .1),
    }


def normalized(data, scales):
    return ((data['measured']-scales['output_baseline'])/scales['output_scale'],
            data['reference']/scales['input_scale'])


def feature(output, reference, k, na, nb, delay):
    return np.concatenate(
        [output[k-lag] for lag in range(1, na+1)]
        + [reference[k-delay-lag] for lag in range(nb)])


def spectral_radius(coefficient, na):
    companion = np.zeros((2*na, 2*na))
    companion[:2] = np.hstack([
        coefficient[2*lag:2*(lag+1)].T for lag in range(na)])
    if na > 1:
        companion[2:, :-2] = np.eye(2*(na-1))
    return float(np.max(np.abs(np.linalg.eigvals(companion))))


def fit(data, structure, scales, bounds):
    na, nb, delay, ridge = structure
    output, reference = normalized(data, scales)
    pick = indices(data, bounds)
    history = max(na, delay+nb-1)
    pick = pick[pick >= history]
    x = np.asarray([feature(output, reference, k, na, nb, delay) for k in pick])
    y = output[pick]
    coefficient = np.linalg.solve(
        x.T@x+len(x)*ridge*np.eye(x.shape[1]), x.T@y)
    return coefficient


def endpoint_metrics(data, coefficient, structure, scales, bounds, horizon, stride=5):
    na, nb, delay, _ = structure
    measured, reference = normalized(data, scales)
    pick = indices(data, bounds)
    contained = np.zeros(len(data['time']), dtype=bool)
    contained[pick] = True
    starts = pick[(pick >= max(na, delay+nb-1))
                  & (pick+horizon < len(contained))]
    starts = starts[contained[np.minimum(starts+horizon, len(contained)-1)]][::stride]
    predicted, actual, held = [], [], []
    for start in starts:
        working = measured.copy()
        for k in range(start, start+horizon+1):
            working[k] = feature(working, reference, k, na, nb, delay)@coefficient
        predicted.append(working[start+horizon]*scales['output_scale']
                         + scales['output_baseline'])
        actual.append(data['measured'][start+horizon])
        held.append(data['measured'][start-1])
    predicted, actual, held = map(np.asarray, (predicted, actual, held))
    rmse = np.sqrt(np.mean((predicted-actual)**2, axis=0))
    hold_rmse = np.sqrt(np.mean((held-actual)**2, axis=0))
    return {
        'horizon_ms': horizon*20,
        'origins': int(len(starts)),
        'rmse_dps': dict(zip(OUTPUTS, rmse.tolist())),
        'hold_rmse_dps': dict(zip(OUTPUTS, hold_rmse.tolist())),
        'improvement_over_hold_fraction': dict(
            zip(OUTPUTS, (1-rmse/np.maximum(hold_rmse, 1e-12)).tolist())),
    }


def evaluate(data, coefficient, structure, scales, bounds):
    return {str(horizon*20): endpoint_metrics(
        data, coefficient, structure, scales, bounds, horizon) for horizon in HORIZONS}


def dc_gain(coefficient, structure, scales):
    na, nb, _, _ = structure
    a_sum = sum((coefficient[2*i:2*(i+1)].T for i in range(na)), np.zeros((2, 2)))
    offset = 2*na
    b_sum = sum((coefficient[offset+2*i:offset+2*(i+1)].T for i in range(nb)),
                np.zeros((2, 2)))
    normalized_gain = np.linalg.solve(np.eye(2)-a_sum, b_sum)
    return (np.diag(scales['output_scale']) @ normalized_gain
            @ np.diag(1/scales['input_scale']))


def identify(capture):
    data = load_capture(capture)
    scales = scales_for(data)
    candidates = []
    for structure in GRID:
        coefficient = fit(data, structure, scales, TRAIN)
        radius = spectral_radius(coefficient, structure[0])
        if not np.isfinite(radius) or radius >= .995:
            continue
        selection = evaluate(data, coefficient, structure, scales, SELECT)
        score = float(np.mean([
            value/output_scale for metrics in selection.values()
            for value, output_scale in zip(metrics['rmse_dps'].values(), scales['output_scale'])]))
        candidates.append((score, structure, coefficient, radius, selection))
    if not candidates:
        raise RuntimeError('no stable speed-reference predictor in frozen grid')
    score, structure, coefficient, radius, selection = min(candidates, key=lambda item: item[0])
    internal = evaluate(data, coefficient, structure, scales, INTERNAL_TEST)
    # S1 is development data. Once structure selection and the internal diagnostic are recorded,
    # refit the same structure on the complete excitation window for the one-time S2 check.
    frozen_scales = scales_for(data, ACTIVE)
    frozen_coefficient = fit(data, structure, frozen_scales, ACTIVE)
    frozen_radius = spectral_radius(frozen_coefficient, structure[0])
    active_fit = evaluate(data, frozen_coefficient, structure, frozen_scales, ACTIVE)
    gain = dc_gain(frozen_coefficient, structure, frozen_scales)
    gate = {
        'data_quality': 'download complete; no quality/CRC/firmware/CAN fault; full 36 s; shortfall <=5',
        'primary_window_s': [3, 31],
        'primary_horizon_ms': 200,
        'minimum_improvement_over_hold_fraction': {
            OUTPUTS[0]: .20, OUTPUTS[1]: .50,
        },
        'maximum_rmse_dps': {OUTPUTS[0]: 6.0, OUTPUTS[1]: 6.0},
        'additional_requirements': [
            'frozen model autonomous spectral radius below 0.995',
            '100 ms and 500 ms rolling prediction improvements are reported without threshold tuning',
            'no coefficient, scale, structure, resampling, polarity or threshold changes after S2 is viewed',
        ],
        'pass_rule': 'all data-quality, 200 ms per-output improvement and RMSE limits must pass',
    }
    return {
        'status': 'S1_SPEED_REFERENCE_MODEL_FROZEN_PENDING_S2',
        'hardware_takeover_allowed': False,
        'model_scope': ('closed-loop speed-reference predictor with existing speed PIDs retained; '
                        'not a voltage/current-to-mechanics plant'),
        'source': {key: data[key] for key in ('capture', 'raw_sha256', 'samples_csv_sha256')},
        'resampling': {'period_ms': 20, 'method': 'linear interpolation on recorded trace_us',
                       'samples': len(data['time'])},
        'signals': {'inputs': INPUTS, 'outputs': OUTPUTS,
                    'big_output_note': 'CAN motor RPM converted by 6 deg/s per RPM',
                    'small_output_note': 'IMU gyro Z converted from rad/s to deg/s'},
        'selection': {
            'grid_candidates': len(GRID), 'eligible_stable_candidates': len(candidates),
            'training_window_s': TRAIN, 'selection_window_s': SELECT,
            'internal_test_window_s': INTERNAL_TEST,
            'score_mean_rmse_divided_by_training_scale': score,
            'selected_structure': {'output_lags': structure[0], 'reference_lags': structure[1],
                                   'reference_delay_samples': structure[2],
                                   'ridge': structure[3]},
            'training_model_spectral_radius': radius,
            'selection_rolling_metrics': selection,
            'internal_test_rolling_metrics': internal,
        },
        'frozen_model': {
            'sample_period_ms': 20,
            'structure': {'output_lags': structure[0], 'reference_lags': structure[1],
                          'reference_delay_samples': structure[2], 'ridge': structure[3]},
            'output_baseline_dps': dict(zip(OUTPUTS, frozen_scales['output_baseline'].tolist())),
            'output_scale_dps': dict(zip(OUTPUTS, frozen_scales['output_scale'].tolist())),
            'input_scale_dps': dict(zip(INPUTS, frozen_scales['input_scale'].tolist())),
            'normalized_coefficient_feature_rows_by_output_columns': frozen_coefficient.tolist(),
            'autonomous_spectral_radius': frozen_radius,
            'dc_gain_output_by_input': gain.tolist(),
            'S1_active_rolling_metrics': active_fit,
        },
        'frozen_S2_acceptance_gate': gate,
        'next_action': 'collect S2 exactly once, then validate without refitting',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = identify(args.capture)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
