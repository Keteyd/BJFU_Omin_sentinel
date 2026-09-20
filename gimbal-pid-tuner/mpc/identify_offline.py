"""Read-only capture analysis and experimental MIMO ARX; never hardware control."""

import argparse
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from yaw_identification_protocol import Decoder  # noqa: E402

OUTPUTS = ('big_joint_delta_deg', 'small_joint_delta_deg', 'imu_heading_delta_deg')
DT = .004
TRAIN_END = 500       # [0, 2) seconds
TUNE_END = 750        # [2, 3) seconds; test [3, 4)
PROVENANCE = 'EXPERIMENTAL_CLOSED_LOOP_ARX_NOT_FOR_HARDWARE'


@dataclass
class Trial:
    name: str
    y: np.ndarray
    u: np.ndarray
    metadata: dict
    sha256: str = ''
    source_path: str = ''


def load_trial(directory):
    directory = Path(directory)
    report = json.loads((directory / 'report.json').read_text(encoding='utf-8'))
    decoder = Decoder(report['trial_id'])
    raw = (directory / 'raw.bin').read_bytes()
    decoder.feed(raw)
    if decoder.report()['quality_issues'] or report.get('error'):
        raise ValueError('incomplete or invalid capture: ' + str(directory))
    if decoder.metadata != report['metadata']:
        raise ValueError('report metadata differs from raw capture')
    rows = decoder.rows
    phases = [r['phase'] for r in rows]
    if phases != [2] * 125 + [3] * 750 + [4] * 125 + [5]:
        raise ValueError('unexpected phase schedule')
    if any((r['flags'] & 29) != 29 for r in rows[:-1]):
        raise ValueError('missing enabled/valid feedback flags')
    if rows[0]['tick_ms'] != decoder.metadata['start_ms']:
        raise ValueError('capture start differs from metadata')
    c = decoder.metadata['configuration']
    # Drop the terminal output-off sample. Never concatenate derivatives across trials.
    big = np.unwrap(np.array([r['big_raw'] for r in rows[:-1]]) * 2 * np.pi / 8192)
    small = np.unwrap(np.array([r['small_raw'] for r in rows[:-1]]) * 2 * np.pi / 8192)
    y = np.column_stack((np.rad2deg(big) - c['big_anchor_deg'],
                         np.rad2deg(small) - c['small_zero_deg'] - c['small_joint_start_deg'],
                         [r['yaw_deg'] - c['small_heading_anchor_deg'] for r in rows[:-1]]))
    u = np.array([[r['big_command_raw'], r['small_command_raw']] for r in rows[:-1]]) / 1000
    if not np.isfinite(y).all() or not np.isfinite(u).all():
        raise ValueError('nonfinite observation')
    if np.any(np.abs(u) > [c['big_effort_limit'], c['small_effort_limit']]):
        raise ValueError('output exceeds recorded configuration')
    return Trial(directory.name, y, u, decoder.metadata, hashlib.sha256(raw).hexdigest(), str(directory.resolve()))


def check_pair(trials):
    if len(trials) != 2 or {t.metadata['axis'] for t in trials} != {1, 2}:
        raise ValueError('need one big-axis and one small-axis trial')
    if len({t.sha256 for t in trials}) != 2:
        raise ValueError('duplicate capture')
    a, b = [t.metadata['configuration'] for t in trials]
    changing = {'amplitude_deg', 'big_anchor_deg', 'small_heading_anchor_deg',
                'small_joint_start_deg', 'pitch_target_rad', 'imu_roll_start_deg',
                'imu_pitch_start_deg'}
    for key in a:
        if key not in changing and a[key] != b[key]:
            raise ValueError('control configuration changed: ' + key)
    if trials[0].metadata['build'] != trials[1].metadata['build']:
        raise ValueError('different firmware builds')
    inputs = np.vstack([t.u[:TRAIN_END]-t.u[:TRAIN_END].mean(axis=0) for t in trials])
    if np.linalg.matrix_rank(inputs) < 2:
        raise ValueError('training lacks two input directions')
    for key, tolerance in [('small_joint_start_deg', 2.), ('imu_roll_start_deg', .5),
                           ('imu_pitch_start_deg', .5), ('pitch_target_rad', .01)]:
        if abs(a[key] - b[key]) > tolerance:
            raise ValueError('poses too different for this local analysis: ' + key)


@dataclass
class ARX:
    order: int
    input_order: int
    delay: int
    ridge: float
    coefficient: np.ndarray
    intercept: np.ndarray
    output_scale: np.ndarray

    @property
    def history(self):
        return max(self.order, self.delay + self.input_order - 1)

    def features(self, y, u, k):
        return np.concatenate([y[k-i] for i in range(1, self.order+1)] +
                              [u[k-self.delay-i] for i in range(self.input_order)])

    def radius(self):
        n = 3 * self.order
        a = np.zeros((n, n))
        a[:3, :] = self.coefficient[:n, :].T
        a[3:, :-3] = np.eye(n-3)
        return float(np.max(np.abs(np.linalg.eigvals(a))))

    def rollout(self, trial, origin, steps):
        if origin < self.history or steps < 1 or origin + steps > len(trial.y):
            raise ValueError('invalid rollout window')
        # Copy only pre-origin measurements. Future y cannot leak into simulation.
        history = self.history
        y = np.empty((history + steps, 3))
        y[:history] = trial.y[origin-history:origin]
        u = trial.u[origin-history:origin+steps]
        with np.errstate(over='raise', invalid='raise'):
            for k in range(history, history+steps):
                y[k] = self.features(y, u, k) @ self.coefficient + self.intercept
                if not np.isfinite(y[k]).all() or np.max(np.abs(y[k])) > 1e6:
                    raise FloatingPointError('candidate diverged')
        return y[history:]

    def artifact(self):
        return dict(provenance=PROVENANCE, hardware_takeover_allowed=False,
                    dt_s=DT, output_names=OUTPUTS,
                    input_names=['big_software_effort', 'small_software_effort'],
                    order=self.order, input_order=self.input_order, delay_samples=self.delay,
                    ridge=self.ridge, coefficient=self.coefficient.tolist(),
                    intercept=self.intercept.tolist(), output_scale=self.output_scale.tolist(),
                    feature_layout='y[k-1],...,y[k-order],u[k-delay],...,u[k-delay-input_order+1]',
                    autonomous_spectral_radius=self.radius(),
                    not_physical_inertia_or_torque_calibration=True)


def fit(trials, order, input_order, delay, ridge, end=TRAIN_END):
    if min(order, input_order, delay) < 1 or not np.isfinite(ridge) or ridge < 0:
        raise ValueError('invalid ARX structure')
    model = ARX(order, input_order, delay, ridge, None, None, None)
    if end <= model.history or any(end > len(t.y) for t in trials):
        raise ValueError('insufficient training samples')
    x, y = [], []
    for t in trials:
        for k in range(model.history, end):
            x.append(model.features(t.y, t.u, k)); y.append(t.y[k])
    x, y = np.array(x), np.array(y)
    if not np.isfinite(x).all() or not np.isfinite(y).all():
        raise ValueError('nonfinite training data')
    xm, ym = x.mean(axis=0), y.mean(axis=0)
    xs = np.maximum(x.std(axis=0), 1e-8)
    ys = np.maximum(y.std(axis=0), 360/8192)
    z, target = (x-xm)/xs, (y-ym)/ys
    # Ridge via augmented least squares, with centering for an unpenalized intercept.
    augmented = np.vstack((z, np.sqrt(len(x)*ridge)*np.eye(x.shape[1])))
    rhs = np.vstack((target, np.zeros((x.shape[1], 3))))
    coefficient = np.linalg.lstsq(augmented, rhs, rcond=None)[0]
    model.coefficient = coefficient * ys / xs[:, None]
    model.intercept = ym - xm @ model.coefficient
    model.output_scale = ys
    return model


def metrics(errors, scales):
    error = np.vstack(errors)
    return dict(rmse_deg=np.sqrt(np.mean(error**2, axis=0)).tolist(),
                max_abs_error_deg=np.max(np.abs(error), axis=0).tolist(),
                normalized_rmse=float(np.sqrt(np.mean((error/scales)**2))))


def evaluate(model, trial, start, end, horizon):
    if start < max(model.history, 6) or end > len(trial.y) or end-start < horizon:
        raise ValueError('invalid evaluation split')
    errors, hold, velocity = [], [], []
    for origin in range(start, end-horizon+1, 1 if horizon == 1 else 10):
        target = trial.y[origin:origin+horizon]
        predicted = model.rollout(trial, origin, horizon)
        anchor = trial.y[origin-1]
        slope = (anchor-trial.y[origin-6])/5
        errors.append(predicted-target)
        hold.append(np.broadcast_to(anchor, target.shape)-target)
        velocity.append(anchor+np.arange(1, horizon+1)[:, None]*slope-target)
    return dict(horizon_ms=horizon*4, windows=len(errors),
                model=metrics(errors, model.output_scale),
                hold_baseline=metrics(hold, model.output_scale),
                constant_velocity_baseline=metrics(velocity, model.output_scale))


def select(trials):
    candidates = []
    models = []
    for order in (2, 5, 10):
        for delay in (1, 3, 5):
            for ridge in (1e-6, 1e-3, .1):
                model = fit(trials, order, 5, delay, ridge)
                entry = dict(order=order, input_order=5, delay_samples=delay, ridge=ridge)
                try:
                    results = [evaluate(model, t, TRAIN_END, TUNE_END, 50) for t in trials]
                    score = float(np.mean([r['model']['normalized_rmse']**2 for r in results]))
                    if not np.isfinite(score):
                        raise FloatingPointError('nonfinite selection score')
                    entry.update(tuning_score=score, spectral_radius=model.radius())
                    models.append((score, model))
                except FloatingPointError:
                    entry.update(tuning_score=None, rejected='divergent_tuning_rollout')
                candidates.append(entry)
    if not models:
        raise ValueError('all candidate models diverged')
    return min(models, key=lambda item: item[0])[1], candidates


def geometry_audit(trials):
    """Diagnostic only: allow per-trial linear drift and offset; no estimator write."""
    features, heading = [], []
    for i, t in enumerate(trials):
        x = np.zeros((len(t.y), 6))
        x[:, :2] = t.y[:, :2]
        x[:, 2+i] = np.arange(len(t.y))*DT
        x[:, 4+i] = 1
        features.append(x)
        heading.append(t.y[:, 2])
    train_x = np.vstack([x[:TRAIN_END] for x in features])
    train_y = np.concatenate([y[:TRAIN_END] for y in heading])
    coefficient = np.linalg.lstsq(train_x, train_y, rcond=None)[0]
    checks = []
    for t, x, y in zip(trials, features, heading):
        err = x[TUNE_END:] @ coefficient-y[TUNE_END:]
        checks.append(dict(trial=t.name, test_rmse_deg=float(np.sqrt(np.mean(err**2)))))
    return dict(diagnostic_only=True, encoder_coefficients=coefficient[:2].tolist(),
                drift_dps=coefficient[2:4].tolist(), offsets_deg=coefficient[4:].tolist(),
                training_condition_number=float(np.linalg.cond(train_x)), test=checks,
                warning='Short-window fitted signs/gains are not validated IMU mounting calibration.')


def analyze(trials):
    check_pair(trials)
    model, candidates = select(trials)
    results, curves = [], {}
    for t in trials:
        entry = dict(trial=t.name, trial_id=t.metadata['id'], axis=t.metadata['axis'])
        entry['test'] = [evaluate(model, t, TUNE_END, len(t.y), n) for n in (1, 25, 50)]
        free = model.rollout(t, TUNE_END, len(t.y)-TUNE_END)
        entry['test_full_second'] = metrics([free-t.y[TUNE_END:]], model.output_scale)
        entry['test_full_second_hold'] = metrics([
            t.y[TUNE_END-1]-t.y[TUNE_END:]], model.output_scale)
        curves[t.name] = np.column_stack((np.arange(TUNE_END, len(t.y))*DT,
                                         t.y[TUNE_END:], free, t.u[TUNE_END:]))
        results.append(entry)
    reasons = ['no_independent_repeated_trial_validation',
               'closed_loop_estimation_bias_not_resolved',
               'low_speed_friction_and_quantization_not_validated',
               'not_compatible_with_synthetic_six_state_mpc_model']
    if model.radius() > 1.000001:
        reasons.append('autonomous_model_unstable')
    for entry in results:
        for r in entry['test'][1:]:
            if r['model']['normalized_rmse'] >= min(r['hold_baseline']['normalized_rmse'],
                                                   r['constant_velocity_baseline']['normalized_rmse']):
                reasons.append('held_out_rollout_does_not_beat_simple_baselines')
    report = dict(model_status='experimental_unvalidated', hardware_takeover_allowed=False,
                  sources=[dict(path=t.source_path or t.name, raw_sha256=t.sha256, metadata=t.metadata) for t in trials],
                  split_ms=dict(train=[0, 2000], tune=[2000, 3000], test=[3000, 4000]),
                  selection='27 structures, mean normalized 200-ms tuning rollout MSE; no test-based reselection',
                  output_order=OUTPUTS, geometry=geometry_audit(trials),
                  candidates=candidates, selected=model.artifact(), validation=results,
                  blockers=sorted(set(reasons)),
                  caveats=['Windows overlap; scores are not independent confidence intervals.',
                           'Rollouts use recorded future drive inputs but no future measured outputs.',
                           'No physical inertia, torque or general stability claim.',
                           'Model not refitted on tuning/test data.'])
    return report, curves


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('captures', nargs=2, type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    trials = [load_trial(p) for p in args.captures]
    report, curves = analyze(trials)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
    (args.output/'candidate.json').write_text(json.dumps(report['selected'], indent=2, allow_nan=False), encoding='utf-8')
    for name, curve in curves.items():
        np.savetxt(args.output/(name+'_test.csv'), curve, delimiter=',', comments='',
                   header='time_s,big_measured_deg,small_measured_deg,heading_measured_deg,'
                          'big_simulated_deg,small_simulated_deg,heading_simulated_deg,big_effort,small_effort')
    print(json.dumps(dict(output=str(args.output), model_status=report['model_status'],
                         hardware_takeover_allowed=False,
                         selected={k: report['selected'][k] for k in
                                   ('order', 'input_order', 'delay_samples', 'ridge', 'autonomous_spectral_radius')},
                         blockers=report['blockers']), indent=2, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
