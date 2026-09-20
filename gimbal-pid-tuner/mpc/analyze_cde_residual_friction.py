"""Post-validation C/D/E residual and friction diagnostics.

This script never refits the accepted frozen-C motion model. It evaluates
residual feature sets after E has been consumed, so every correction candidate
remains development-only and requires a new independent phase.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


DT = .004
START, END = 750, 7750
OUTPUTS = ('big_joint_deg', 'small_joint_deg', 'heading_deg')


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest().upper()


def load_csv(path):
    values = np.genfromtxt(path, delimiter=',', names=True)
    get = lambda name: np.asarray(values[name], dtype=float)
    return {
        'reference': np.column_stack((get('reference_big_deg'),
                                      get('reference_small_deg'))),
        'measured': np.column_stack((get('measured_big_joint_delta_deg'),
                                     get('measured_small_joint_delta_deg'),
                                     get('measured_heading_delta_deg'))),
        'command': np.column_stack((get('attempted_big_command'),
                                    get('attempted_small_command'))),
        'current': np.column_stack((get('big_current_raw'),
                                    get('small_current_raw'))),
    }


def arx_feature(output, reference, index, na, nb, delay):
    pieces = ([output[index-lag] for lag in range(1, na+1)] +
              [reference[index-delay-lag] for lag in range(nb)] +
              [np.ones(1)])
    return np.concatenate(pieces)


def frozen_prediction(data, model):
    structure = model['structure']
    na = structure['output_lags']
    nb = structure['reference_lags']
    delay = structure['reference_delay_samples']
    output_center = np.asarray(model['output_center_deg'])
    output_scale = np.asarray(model['output_scale_deg'])
    reference_center = np.asarray(model['reference_center_deg'])
    reference_scale = np.asarray(model['reference_scale_deg'])
    coefficient = np.asarray(model['normalized_coefficient'])
    output = (data['measured']-output_center)/output_scale
    reference = (data['reference']-reference_center)/reference_scale
    predicted = np.array(output, copy=True)
    for index in range(START, len(output)):
        predicted[index] = arx_feature(predicted, reference, index, na, nb, delay)@coefficient
    return predicted*output_scale+output_center


def smooth_velocity(measured):
    """Return a causal 52 ms moving average of backward-difference velocity."""
    raw = np.vstack((np.zeros((1, measured.shape[1])),
                     np.diff(measured, axis=0)/DT))
    cumulative = np.vstack((np.zeros((1, measured.shape[1])),
                            np.cumsum(raw, axis=0)))
    velocity = np.empty_like(raw)
    for index in range(len(raw)):
        first = max(0, index-12)
        velocity[index] = ((cumulative[index+1]-cumulative[first]) /
                           (index-first+1))
    return velocity


def features(data, kind):
    ref = data['reference']
    ref_rate = np.gradient(ref, DT, axis=0)
    command = data['command']
    current = data['current']-np.median(data['current'][:START], axis=0)
    measured = data['measured']
    velocity = smooth_velocity(measured)
    lag = lambda values: np.vstack((values[:1], values[:-1]))
    blocks = [ref, ref_rate]
    if kind in ('command_current', 'friction_state'):
        blocks += [lag(command), lag(current)]
    if kind == 'friction_state':
        lag_velocity = lag(velocity)
        blocks += [lag(measured), lag_velocity, np.sign(lag_velocity),
                   np.abs(lag_velocity)]
    return np.column_stack(blocks)


def ridge_fit(train_x, train_y, test_x, ridge=1e-2):
    if train_x.shape[1] == 0:
        prediction = np.repeat(train_y.mean(axis=0, keepdims=True), len(test_x), axis=0)
        return prediction
    center = train_x.mean(axis=0)
    scale = np.maximum(train_x.std(axis=0), 1e-9)
    x = (train_x-center)/scale
    z = (test_x-center)/scale
    design = np.column_stack((x, np.ones(len(x))))
    penalty = np.eye(design.shape[1])
    penalty[-1, -1] = 0
    coefficient = np.linalg.solve(
        design.T@design+len(design)*ridge*penalty, design.T@train_y)
    return np.column_stack((z, np.ones(len(z))))@coefficient


def metrics(before, after):
    before_rmse = np.sqrt(np.mean(before**2, axis=0))
    after_rmse = np.sqrt(np.mean(after**2, axis=0))
    return {
        'frozen_rmse_deg': before_rmse.tolist(),
        'corrected_rmse_deg': after_rmse.tolist(),
        'improvement_fraction':
            (1-after_rmse/np.maximum(before_rmse, 1e-12)).tolist(),
        'frozen_mean_error_deg': before.mean(axis=0).tolist(),
        'corrected_mean_error_deg': after.mean(axis=0).tolist(),
    }


def signed_groups(residual, signal, threshold):
    positive = signal > threshold
    negative = signal < -threshold
    return {
        'threshold': float(threshold),
        'positive_samples': int(positive.sum()),
        'negative_samples': int(negative.sum()),
        'positive_mean_residual_deg':
            float(residual[positive].mean()) if positive.any() else None,
        'negative_mean_residual_deg':
            float(residual[negative].mean()) if negative.any() else None,
        'positive_minus_negative_deg':
            float(residual[positive].mean()-residual[negative].mean())
            if positive.any() and negative.any() else None,
    }


def analyze(c_csv, d_csv, e_csv, c_report):
    paths = {'C': Path(c_csv), 'D': Path(d_csv), 'E': Path(e_csv)}
    datasets = {name: load_csv(path) for name, path in paths.items()}
    frozen_report = json.loads(Path(c_report).read_text(encoding='utf-8'))
    model = frozen_report['frozen_C_closed_loop_model']
    phase = {}
    for name, data in datasets.items():
        prediction = frozen_prediction(data, model)
        residual = prediction-data['measured']
        data['residual'] = residual
        data['velocity'] = smooth_velocity(data['measured'])
        active = slice(START, min(END, len(residual)))
        phase[name] = {
            'samples': len(residual),
            'active_frozen_rmse_deg':
                np.sqrt(np.mean(residual[active]**2, axis=0)).tolist(),
            'baseline_current_median_raw':
                np.median(data['current'][:START], axis=0).tolist(),
            'direction_diagnostics': {},
        }
        for column, output in enumerate(OUTPUTS[:2]):
            r = residual[active, column]
            v = data['velocity'][active, column]
            command = data['command'][active, column]
            phase[name]['direction_diagnostics'][output] = {
                'by_velocity_direction': signed_groups(r, v, .2),
                'by_command_direction': signed_groups(r, command, .05),
                'position_residual_correlation': float(np.corrcoef(
                    data['measured'][active, column], r)[0, 1]),
                'velocity_residual_correlation': float(np.corrcoef(v, r)[0, 1]),
            }

    feature_sets = ('bias_only', 'reference_only', 'command_current', 'friction_state')
    cross_phase = {}
    for kind in feature_sets:
        cross_phase[kind] = {}
        for held_out in datasets:
            train_names = [name for name in datasets if name != held_out]
            train_x, train_y = [], []
            for name in train_names:
                data = datasets[name]
                active = slice(START, min(END, len(data['residual'])))
                x = np.empty((len(data['residual']), 0)) if kind == 'bias_only' else features(data, kind)
                train_x.append(x[active])
                train_y.append(data['residual'][active])
            data = datasets[held_out]
            active = slice(START, min(END, len(data['residual'])))
            test_x = (np.empty((len(data['residual']), 0)) if kind == 'bias_only'
                      else features(data, kind))[active]
            before = data['residual'][active]
            correction = ridge_fit(np.vstack(train_x), np.vstack(train_y), test_x)
            cross_phase[kind][held_out] = metrics(before, before-correction)

    summary = {}
    for kind, heldouts in cross_phase.items():
        improvements = np.asarray([
            heldouts[name]['improvement_fraction'] for name in ('C', 'D', 'E')])
        summary[kind] = {
            'mean_improvement_fraction': improvements.mean(axis=0).tolist(),
            'minimum_improvement_fraction': improvements.min(axis=0).tolist(),
            'all_joint_holdouts_improve': bool(np.all(improvements[:, :2] > 0)),
        }

    return {
        'analysis_status': 'post_E_development_diagnostic_only',
        'model_status': 'closed_loop_candidate_passed_C_E_gates_unchanged',
        'hardware_takeover_allowed': False,
        'data_policy': {
            'E_has_been_consumed': True,
            'frozen_C_model_refit': False,
            'residual_correction_candidates_are_validated': False,
            'new_independent_phase_required_for_any_selected_correction': True,
        },
        'sources': {
            name: {'path': str(path.resolve()), 'sha256': sha256(path)}
            for name, path in paths.items()
        },
        'frozen_C_report_sha256': sha256(c_report),
        'outputs': list(OUTPUTS),
        'phase_diagnostics': phase,
        'leave_one_phase_out_residual_correction': cross_phase,
        'feature_set_summary': summary,
        'feature_interpretation': {
            'reference_only': 'scheduled reference and reference rate',
            'command_current': 'reference plus causal lagged software command and baseline-centered raw current',
            'friction_state': 'adds lagged measured position, causal-smoothed velocity, direction and absolute speed; observer correction diagnostic, not a free-run plant',
        },
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--c-csv', required=True, type=Path)
    parser.add_argument('--d-csv', required=True, type=Path)
    parser.add_argument('--e-csv', required=True, type=Path)
    parser.add_argument('--c-report', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output must not exist')
    report = analyze(args.c_csv, args.d_csv, args.e_csv, args.c_report)
    args.output.mkdir(parents=True)
    (args.output/'report.json').write_text(
        json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({
        'analysis_status': report['analysis_status'],
        'model_status': report['model_status'],
        'feature_set_summary': report['feature_set_summary'],
        'hardware_takeover_allowed': False,
        'output': str(args.output),
    }, indent=2))


if __name__ == '__main__':
    main()
