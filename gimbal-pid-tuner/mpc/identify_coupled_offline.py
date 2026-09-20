"""Local normalized two-joint grey-box fit. Offline only, never hardware-ready."""

import argparse
from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np
from scipy.linalg import expm
from scipy.optimize import least_squares

from audit_slow_pair import load, review
from identify_slow_offline import uniform_samples

DT = .004
HISTORY = 11
TRAIN = 1750
TUNE = 3500
VS = np.deg2rad(1.)
LOW = np.array([-4., -3., -8., -8., -12., -12., -8., -8.])
HIGH = np.array([4., 3., 8., 8., 4., 4., 8., 8.])
PARAMETERS = ('log_mass_ratio', 'mass_correlation', 'log_big_damping', 'log_small_damping',
              'log_big_friction', 'log_small_friction', 'log_big_command_gain', 'log_small_command_gain')


def causal_velocity(q):
    """Trailing 40ms linear slope; no future measurements at prediction origins."""
    t = np.arange(HISTORY) * DT
    weights = (t - t.mean()) / np.sum((t-t.mean())**2)
    v = np.full_like(q, np.nan)
    for k in range(HISTORY-1, len(q)):
        v[k] = weights @ q[k-HISTORY+1:k+1]
    return v


def lagged_input(u, tau):
    if tau < 0:
        raise ValueError('negative actuator time constant')
    z = np.array(u, copy=True)
    alpha = 1. if tau == 0 else -np.expm1(-DT/tau)
    for k in range(1, len(u)):
        z[k] = z[k-1] + alpha*(u[k]-z[k-1])
    return z


def parameters(p):
    ratio = np.exp(p[0])
    cross = .95 * np.tanh(p[1]) * np.sqrt(ratio)
    return np.array([[1., cross], [cross, ratio]]), np.exp(p[2:4]), np.exp(p[4:6]), np.exp(p[6:8])


@dataclass
class Coupled:
    p: np.ndarray
    tau: float

    def __post_init__(self):
        self.mass, self.damping, self.friction, self.gain = parameters(self.p)
        inv = np.linalg.inv(self.mass)
        a = np.zeros((4, 4))
        a[:2, 2:] = np.eye(2)
        a[2:, 2:] = -inv @ np.diag(self.damping)
        b = np.vstack((np.zeros((2, 2)), inv))
        augmented = np.zeros((6, 6))
        augmented[:4, :4], augmented[:4, 4:] = a, b
        discrete = expm(augmented * DT)
        self.ad, self.bd = discrete[:4, :4], discrete[:4, 4:]

    def step(self, x, u):
        force = u*self.gain - self.friction*np.tanh(x[..., 2:]/VS)
        return x @ self.ad.T + force @ self.bd.T

    def artifact(self):
        return dict(hardware_takeover_allowed=False, model_status='experimental_unvalidated',
                    coordinates='q_big relative to base; q_small relative to big; radians',
                    state_order=['q_big', 'q_small', 'v_big', 'v_small'],
                    heading_matrix=[1., 1., 0., 0.], angle_scales=[1., 1.],
                    normalized_mass=self.mass.tolist(), damping=self.damping.tolist(),
                    coulomb_friction=self.friction.tolist(), command_gain=self.gain.tolist(),
                    actuator_tau_s=self.tau, friction_smoothing_rad_s=VS,
                    mass_eigenvalues=np.linalg.eigvalsh(self.mass).tolist(),
                    warning='M11=1 fixes scale ambiguity. Not physical kg*m^2 or calibrated torque. Local fixed-Pitch model only.')


def prepare(paths, allow_jitter):
    audit = review(paths)
    trials = []
    for path in paths:
        info, y, u, t, _ = load(path)
        if info['raw_quality_issues'] and not allow_jitter:
            raise ValueError('explicit --allow-one-ms-jitter required')
        y, u = uniform_samples(t, y, u)
        q = np.deg2rad(y[:, :2])
        trials.append(dict(info=info, y=y, q=q, v=causal_velocity(q), u=u))
    return audit, trials


def integral_rows(trials, tau):
    rows = []
    for trial in trials:
        q, v = trial['q'], trial['v']
        u = lagged_input(trial['u'][:TRAIN], tau)
        for k in range(HISTORY-1, TRAIN-25, 10):
            end = k+25
            rows.append(np.concatenate((v[end]-v[k], q[end]-q[k],
                                         DT*np.tanh(v[k:end]/VS).sum(axis=0),
                                         DT*u[k:end].sum(axis=0))))
    return np.array(rows)


def velocity_residual(p, rows, scale):
    m, d, f, g = parameters(p)
    impulse = rows[:, 6:8]*g - rows[:, 2:4]*d - rows[:, 4:6]*f
    # Compare delta-velocity, not unweighted momentum: otherwise shrinking one
    # mass-matrix row artificially reduces its error without improving motion.
    predicted = np.linalg.solve(m, impulse.T).T
    return ((rows[:, :2]-predicted)/scale).ravel()


def fit_model(trials, tau):
    rows = integral_rows(trials, tau)
    scale = np.maximum(rows[:, :2].std(axis=0), .01)

    def residual(p):
        return velocity_residual(p, rows, scale)

    fits = []
    for ratio in (.3, 3.):
        initial = np.array([np.log(ratio), .3, 0., 0., -4., -4., -1., -1.])
        result = least_squares(residual, initial, bounds=(LOW, HIGH), loss='soft_l1',
                               f_scale=.2, max_nfev=600)
        if result.success and np.isfinite(result.cost):
            fits.append(result)
    if not fits:
        raise ValueError('constrained optimizer did not converge')
    best = min(fits, key=lambda r: r.cost)
    diagnostic = dict(cost=float(best.cost), nfev=best.nfev,
                      at_bound=np.flatnonzero(np.minimum(best.x-LOW, HIGH-best.x) < .01).tolist(),
                      at_bound_names=[PARAMETERS[i] for i in np.flatnonzero(np.minimum(best.x-LOW, HIGH-best.x) < .01)],
                      jacobian_rank=int(np.linalg.matrix_rank(best.jac)),
                      jacobian_condition=float(np.linalg.cond(best.jac)))
    return Coupled(best.x, tau), diagnostic


def evaluate(model, trial, start, end, horizon):
    if start < HISTORY or end > len(trial['q']) or end-start < horizon or horizon < 1:
        raise ValueError('invalid prediction window')
    origins = np.arange(start, end-horizon+1, 10)
    q, v, y = trial['q'], trial['v'], trial['y']
    anchor = q[origins-1]
    x = np.column_stack((anchor, v[origins-1]))
    # An observed heading offset initializes each window but is never updated
    # inside its rollout. Future input commands are recorded, not predicted.
    anchor_heading = y[origins-1, 2]
    filtered = lagged_input(trial['u'], model.tau)
    errors = {key: [] for key in ('model', 'hold', 'velocity', 'measured_joint_sum')}
    for step in range(horizon):
        k = origins + step
        x = model.step(x, filtered[k-1])
        pred = np.column_stack((np.rad2deg(x[:, :2]),
                                anchor_heading + np.rad2deg((x[:, :2]-anchor).sum(axis=1))))
        constant = anchor + v[origins-1] * DT*(step+1)
        cv = np.column_stack((np.rad2deg(constant),
                              anchor_heading + np.rad2deg((constant-anchor).sum(axis=1))))
        hold = y[origins-1]
        oracle = anchor_heading + np.rad2deg((q[k]-anchor).sum(axis=1))
        errors['model'].append(pred-y[k])
        errors['hold'].append(hold-y[k])
        errors['velocity'].append(cv-y[k])
        errors['measured_joint_sum'].append((oracle-y[k, 2])[:, None])
    if not all(np.isfinite(np.array(e)).all() for e in errors.values()):
        raise FloatingPointError('nonfinite prediction')
    return dict(horizon_ms=horizon*4, windows=len(origins), **{
        key: dict(rmse_deg=np.sqrt(np.mean(np.vstack(e)**2, axis=0)).tolist())
        for key, e in errors.items()})


def analyze(paths, allow_jitter):
    audit, trials = prepare(paths, allow_jitter)
    candidates, options = [], []
    for tau in (0., .02, .05):
        model, fit_info = fit_model(trials, tau)
        metrics = [evaluate(model, t, TRAIN, TUNE, 50) for t in trials]
        score = float(np.mean([np.mean(np.square(v['model']['rmse_deg'])) for v in metrics]))
        candidates.append(dict(tau=tau, tuning_score=score, fitting=fit_info))
        options.append((score, model, fit_info))
    _, model, fit_info = min(options, key=lambda item: item[0])
    tests = [dict(axis=t['info']['axis'], metrics=[evaluate(model, t, TUNE, 5000, h)
                                                for h in (25, 50, 125)]) for t in trials]
    blockers = ['previously_inspected_time_split_not_independent_validation',
                'closed_loop_bias_and_actuator_torque_scale_unresolved',
                'fixed_axis_heading_constraint_has_measurement_residual',
                'local_model_not_validated_across_pose_or_payload']
    if fit_info['at_bound']:
        blockers.append('fit_parameters_at_search_bounds')
    if fit_info['jacobian_condition'] > 1e6:
        blockers.append('poorly_conditioned_parameter_fit')
    for t in tests:
        for m in t['metrics']:
            if np.mean(np.square(m['model']['rmse_deg'])) >= min(
                np.mean(np.square(m['hold']['rmse_deg'])), np.mean(np.square(m['velocity']['rmse_deg']))):
                blockers.append('prediction_not_better_than_simple_baselines')
    return dict(hardware_takeover_allowed=False, model_status='experimental_unvalidated',
                audit=audit, selected=model.artifact(), candidates=candidates, validation=tests,
                blockers=sorted(set(blockers)), split_ms=dict(train=[0, 7000], tune=[7000, 14000], test=[14000, 20000]),
                method='Positive definite normalized mass, nonnegative damping/friction, diagonal positive command gains; integrated momentum fit over training-only 100ms windows.',
                caveats=['Four mechanical states plus optional two actuator lag states; no fitted angle scale.',
                         '40ms trailing velocity estimates; prediction windows overlap.',
                         'Measured-joint-sum diagnostic uses future joint measurements, not a deployable predictor.',
                         'No MPC solver/controller output generated; no serial or firmware changes.'])


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
    print(json.dumps({k: result[k] for k in ('selected', 'validation', 'blockers')}, indent=2))


if __name__ == '__main__':
    main()
