"""Exploratory slow-capture ARX evaluation; never a hardware controller."""

import argparse
import json
from pathlib import Path

import numpy as np

from audit_slow_pair import load, review
from identify_offline import Trial, evaluate, fit


def uniform_samples(t, y, u):
    """Interpolate angles, hold recorded commands; exclude the terminal off sample."""
    grid = np.arange(5000) * .004
    if len(t) != 5001 or np.any(np.diff(t) <= 0) or t[0] != 0 or t[-1] != 20:
        raise ValueError('need a complete monotonic 20-second source')
    angles = np.column_stack([np.interp(grid, t, y[:, i]) for i in range(y.shape[1])])
    indices = np.searchsorted(t, grid, side='right') - 1
    return angles, u[indices]


def analyze(paths, allow_jitter=False):
    audit = review(paths)
    trials = []
    for path in paths:
        info, y, u, t, _ = load(path)
        if info['raw_quality_issues'] and not allow_jitter:
            raise ValueError('review timing first; --allow-one-ms-jitter explicitly opts into resampling')
        y, u = uniform_samples(t, y, u)
        source_report = json.loads((Path(path)/'report.json').read_text(encoding='utf-8'))
        trials.append(Trial(Path(path).name, y, u, source_report['metadata'],
                            info['sha256'], info['source']))
    candidates, models = [], []
    for order in (2, 5):
        for delay in (1, 3):
            for ridge in (.001, .1):
                model = fit(trials, order, 3, delay, ridge, end=1750)
                entry = dict(order=order, delay=delay, ridge=ridge)
                try:
                    scores = [evaluate(model, trial, 1750, 3500, 50) for trial in trials]
                    score = float(np.mean([s['model']['normalized_rmse']**2 for s in scores]))
                    if not np.isfinite(score):
                        raise FloatingPointError('nonfinite tuning score')
                    entry['tuning_score'] = score
                    models.append((score, model))
                except FloatingPointError:
                    entry['rejected'] = 'divergent_tuning_rollout'
                candidates.append(entry)
    if not models:
        raise ValueError('all candidates diverged')
    _, model = min(models, key=lambda v: v[0])
    validation = []
    for trial in trials:
        try:
            checks = [evaluate(model, trial, 3500, 5000, horizon) for horizon in (1, 25, 50)]
            validation.append(dict(axis=trial.metadata['axis'], trial=trial.name, test=checks))
        except FloatingPointError:
            validation.append(dict(axis=trial.metadata['axis'], test_error='divergent_test_rollout'))
    blockers = ['no_independent_repeat_validation', 'closed_loop_estimation_bias_unresolved',
                'angle_relationship_not_physically_validated', 'not_a_deployable_MPC_state_model']
    if model.radius() > 1.000001:
        blockers.append('autonomous_model_unstable')
    for v in validation:
        if 'test_error' in v:
            blockers.append('test_rollout_diverged')
        for test in v.get('test', [])[1:]:
            if test['model']['normalized_rmse'] >= min(test['hold_baseline']['normalized_rmse'],
                                                     test['constant_velocity_baseline']['normalized_rmse']):
                blockers.append('test_does_not_beat_simple_baselines')
    return dict(model_status='experimental_unvalidated', hardware_takeover_allowed=False,
                split_ms=dict(train=[0, 7000], tune=[7000, 14000], test=[14000, 20000]),
                preprocessing='Actual timestamps; linear angle interpolation, recorded-command ZOH; terminal off sample excluded; raw files unchanged',
                selection='Eight candidates ranked only on tuning 200ms rollout; no refit on tune/test',
                caveats=['Recorded future inputs used in prediction; no future output leakage.',
                         'Overlapping test windows are not independent trials.',
                         'Software command is not calibrated torque/current.',
                         'Input rank two alone does not prove persistent excitation or identifiability.'],
                audit=audit, candidates=candidates, selected=model.artifact(), validation=validation,
                blockers=sorted(set(blockers)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('captures', nargs=2, type=Path)
    parser.add_argument('--allow-one-ms-jitter', action='store_true')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = analyze(args.captures, args.allow_one_ms_jitter)
    args.output.mkdir(exist_ok=False)
    (args.output/'report.json').write_text(json.dumps(result, indent=2, allow_nan=False), encoding='utf-8')
    (args.output/'candidate.json').write_text(json.dumps(result['selected'], indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({k: result[k] for k in ('model_status', 'hardware_takeover_allowed', 'validation', 'blockers')}, indent=2))


if __name__ == '__main__':
    main()
