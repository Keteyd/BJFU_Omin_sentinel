"""Offline continuous-time alignment experiment. Historical fitters stay unchanged.

Commands u[k] are assumed held on [t[k], t[k+1]). Actuator states live at
sample nodes. Fit input moments and prediction use this same convention.
"""

import argparse
import itertools
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from identify_coupled_offline import DT, HISTORY, TRAIN, TUNE, LOW, HIGH, PARAMETERS, prepare
from identify_stiction_offline import ENCODER_RAD, StickSlip, endpoint_velocity, free_run_failures
from identify_weak_stiction_offline import WINDOW, SIGN_RADIUS, weak_residual


def lag_coefficients(tau, dt):
    """Decay, mean transient, and falling-triangle transient coefficients."""
    if not np.isfinite(tau) or tau < 0 or not np.isfinite(dt) or dt <= 0:
        raise ValueError('finite nonnegative tau and positive dt required')
    if tau == 0:
        return 0., 0., 0.
    a = dt/tau
    mean = -np.expm1(-a)/a
    falling = (.5-a/6+a*a/24-a**3/120) if a < 1e-3 else (1-mean)/a
    return np.exp(-a), mean, falling


def actuator_history(u, tau, dt=DT, initial=None):
    """Return node state, interval mean, and integral((dt-t)*z)/dt**2.

Initial state defaults to u[0] (steady prehistory assumption, not measured).
Node k uses only commands before k. Interval integrals additionally use u[k].
"""
    u = np.asarray(u, dtype=float)
    if u.ndim != 2 or u.shape[1] != 2 or not len(u) or not np.isfinite(u).all():
        raise ValueError('finite nonempty N by 2 commands required')
    decay, mean, falling = lag_coefficients(tau, dt)
    nodes = np.empty_like(u)
    nodes[0] = u[0] if initial is None else initial
    if not np.isfinite(nodes[0]).all():
        raise ValueError('finite initial actuator state required')
    if tau == 0:
        nodes[:] = u
    else:
        for k in range(1, len(u)):
            nodes[k] = u[k-1]+decay*(nodes[k-1]-u[k-1])
    return nodes, u+mean*(nodes-u), .5*u+falling*(nodes-u)


def triangular_input(u, tau):
    """Input averaged over the triangle associated with centered q differences.

For k>=1: integral((DT-|t-tk|)*z(t), tk-DT..tk+DT)/DT**2.
The first node assumes the same steady command prehistory as actuator_history.
"""
    _, average, falling = actuator_history(u, tau)
    rising = average-falling
    return np.vstack((.5*np.asarray(u)[0], rising[:-1]))+falling


def timed_rows(trials, tau):
    w = (1-np.linspace(-1., 1., WINDOW+1)**2)**2
    acceleration = np.convolve(w, [1., -2., 1.])/DT
    rows = []
    for trial in trials:
        q, raw_u = np.asarray(trial['q'][:TRAIN]), np.asarray(trial['u'][:TRAIN])
        if q.shape != (TRAIN, 2) or raw_u.shape != (TRAIN, 2) or not np.isfinite(q).all():
            raise ValueError('finite 7 second two-axis training records required')
        weighted_u = triangular_input(raw_u, tau)
        threshold = 2*ENCODER_RAD+32*np.finfo(float).eps*max(1., float(np.abs(q).max()))
        for k in range(SIGN_RADIUS, TRAIN-WINDOW-SIGN_RADIUS, 10):
            j = np.arange(k, k+WINDOW+1)
            delta = q[j+SIGN_RADIUS]-q[j-SIGN_RADIUS]
            signs = np.where(np.abs(delta) > threshold, np.sign(delta), 0.)
            local = q[k-1:k+WINDOW+2]-q[k-1]
            rows.append(np.r_[acceleration @ local, w @ ((q[j+1]-q[j-1])/2),
                              DT*w @ signs, DT*w @ weighted_u[j], DT*w @ (signs == 0)])
    if not rows:
        raise ValueError('no training records')
    return np.array(rows)


class TimedStickSlip(StickSlip):
    """Six states: four mechanical plus two actuator states at the same node."""

    def __post_init__(self):
        super().__post_init__()
        self.substep = DT/4
        self.decay, self.mean, _ = lag_coefficients(self.tau, self.substep)
        self.matrix = self.mass+self.substep*np.diag(self.damping)
        self.active_sets = []
        for sign in itertools.product((0, -1, 1), repeat=2):
            sign = np.array(sign)
            free, fixed = np.flatnonzero(sign), np.flatnonzero(sign == 0)
            inverse = np.linalg.inv(self.matrix[np.ix_(free, free)]) if len(free) else np.empty((0, 0))
            self.active_sets.append((sign, free, fixed, inverse))

    def step(self, x, u):
        single = np.asarray(x).ndim == 1
        x = np.atleast_2d(np.asarray(x, dtype=float)).copy()
        u = np.broadcast_to(np.asarray(u, dtype=float), (len(x), 2))
        if x.shape[1] != 6 or not np.isfinite(x).all() or not np.isfinite(u).all():
            raise ValueError('finite six-state vectors and two-axis commands required')
        for _ in range(4):
            drive = u+self.mean*(x[:, 4:]-u)
            rhs = x[:, 2:4] @ self.mass.T+self.substep*self.gain*drive
            v, found = np.zeros((len(x), 2)), np.zeros(len(x), dtype=bool)
            for signs, free, fixed, inverse in self.active_sets:
                candidate = np.zeros_like(v)
                candidate[:, free] = (rhs[:, free]-self.substep*self.friction[free]*signs[free]) @ inverse.T
                valid = np.all(candidate[:, free]*signs[free] >= -1e-12, axis=1)
                reaction = rhs-candidate @ self.matrix.T
                valid &= np.all(np.abs(reaction[:, fixed]) <= self.substep*self.friction[fixed]+1e-12, axis=1)
                take = valid & ~found
                v[take], found[take] = candidate[take], True
                if found.all():
                    break
            if not found.all():
                raise FloatingPointError('timed friction solve failed')
            x[:, :2] += self.substep*(x[:, 2:4]+v)/2
            x[:, 2:4] = v
            x[:, 4:] = u+self.decay*(x[:, 4:]-u)
        return x[0] if single else x

    def artifact(self):
        a = super().artifact()
        a.update(model_family='normalized_two_joint_timed_coulomb_v1',
                 state_order=['q_big', 'q_small', 'v_big', 'v_small', 'z_big', 'z_small'],
                 heading_matrix=[1., 1., 0., 0., 0., 0.],
                 input_convention='u[k] held on [t[k],t[k+1]); z[k] is node actuator state',
                 integrator='four 1ms implicit velocity steps, mean actuator force, trapezoidal position',
                 identification_method='400ms centered weak momentum with triangular actuator integrals',
                 actuator_initialization='steady u[0] prehistory assumption; propagated using past commands')
        return a


def fit_timed_stiction(trials, tau):
    rows = timed_rows(trials, tau)
    scale = np.maximum(rows[:, :2].std(axis=0), .01)
    fits = []
    for ratio in (.3, 3.):
        initial = np.array([np.log(ratio), .3, 0., 0., -2., -2., -1., -1.])
        result = least_squares(weak_residual, initial, args=(rows, scale), bounds=(LOW, HIGH),
                               loss='soft_l1', f_scale=.2, max_nfev=600)
        if result.success and np.isfinite(result.cost):
            fits.append(result)
    if not fits:
        raise ValueError('timed stiction fit failed')
    best = min(fits, key=lambda f: f.cost)
    condition = float(np.linalg.cond(best.jac))
    return TimedStickSlip(best.x, tau), dict(cost=float(best.cost), nfev=best.nfev, windows=len(rows),
        jacobian_rank=int(np.linalg.matrix_rank(best.jac)),
        jacobian_condition=condition if np.isfinite(condition) else None,
        at_bound_names=[PARAMETERS[i] for i in np.flatnonzero(np.minimum(best.x-LOW, HIGH-best.x) < .01)])


def evaluate_timed(model, trial, start, end, horizon):
    if start < HISTORY or end > len(trial['q']) or end-start < horizon or horizon < 1:
        raise ValueError('invalid prediction window')
    origins = np.arange(start, end-horizon+1, 10)
    q, v, y, u = (trial[k] for k in ('q', 'v', 'y', 'u'))
    anchor, heading = q[origins-1], y[origins-1, 2]
    nodes, _, _ = actuator_history(u, model.tau)
    x = np.column_stack((anchor, v[origins-1], nodes[origins-1]))
    errors = {key: [] for key in ('model', 'hold', 'velocity', 'measured_joint_sum')}
    for step in range(horizon):
        k = origins+step
        x = model.step(x, u[k-1])
        predicted = np.column_stack((np.rad2deg(x[:, :2]), heading+np.rad2deg((x[:, :2]-anchor).sum(axis=1))))
        constant = anchor+v[origins-1]*DT*(step+1)
        cv = np.column_stack((np.rad2deg(constant), heading+np.rad2deg((constant-anchor).sum(axis=1))))
        errors['model'].append(predicted-y[k])
        errors['hold'].append(y[origins-1]-y[k])
        errors['velocity'].append(cv-y[k])
        errors['measured_joint_sum'].append((heading+np.rad2deg((q[k]-anchor).sum(axis=1))-y[k, 2])[:, None])
    if not all(np.isfinite(e).all() for e in errors.values()):
        raise FloatingPointError('nonfinite prediction')
    return dict(horizon_ms=horizon*4, windows=len(origins), **{
        key: dict(rmse_deg=np.sqrt(np.mean(np.vstack(e)**2, axis=0)).tolist()) for key, e in errors.items()})


def select_timed(trials):
    candidates, options = [], []
    for tau in (0., .02, .05):
        model, diagnostic = fit_timed_stiction(trials, tau)
        scores = [evaluate_timed(model, t, TRAIN, TUNE, 50) for t in trials]
        score = float(np.mean([np.mean(np.square(s['model']['rmse_deg'])) for s in scores]))
        candidates.append(dict(tau=tau, tuning_score=score, fitting=diagnostic))
        options.append((score, model, diagnostic))
    _, model, diagnostic = min(options, key=lambda item: item[0])
    return model, diagnostic, candidates


def analyze(paths, allow_jitter):
    audit, trials = prepare(paths, allow_jitter)
    for trial in trials:
        trial['v'] = endpoint_velocity(trial['q'])
    model, diagnostic, candidates = select_timed(trials)
    validation = [dict(axis=t['info']['axis'], rolling=[evaluate_timed(model, t, TUNE, 5000, 50)],
                       free_run=evaluate_timed(model, t, 500, 5000, 4500)) for t in trials]
    failures = free_run_failures(validation)
    blockers = ['no_independent_vehicle_validation', 'closed_loop_bias_unresolved',
                '4ms_command_hold_assumption_not_verified_by_CAN_history',
                'actuator_initial_state_unknown', 'friction_sign_and_external_load_unresolved']
    if diagnostic['at_bound_names']:
        blockers.append('parameters_at_search_bounds')
    if diagnostic['jacobian_rank'] < 8:
        blockers.append('rank_deficient_parameter_fit')
    if failures:
        blockers.append('free_run_heading_not_better_than_hold')
    return dict(model_status='rejected_exploratory_candidate' if failures else 'experimental_timed_candidate',
                hardware_takeover_allowed=False, audit=audit, selected=model.artifact(), candidates=candidates,
                validation=validation, free_run_not_better_than_hold_axes=failures, blockers=blockers,
                split_ms=dict(train=[0, 7000], tune=[7000, 14000], exploratory_check=[14000, 20000]),
                caveats=['Original positive-direction pair only; reverse trial excluded.',
                         '400ms window and two-count friction threshold unchanged.',
                         'Friction and damping quadrature are approximations; actuator integrals are analytic.',
                         'Recorded 4ms snapshots do not establish zero-order hold between snapshots.',
                         'Long-run evaluation on previously inspected training sources is exploratory.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('captures', nargs=2, type=Path)
    parser.add_argument('--allow-one-ms-jitter', action='store_true')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output must not exist')
    result = analyze(args.captures, args.allow_one_ms_jitter)
    args.output.mkdir(exist_ok=False)
    for name, content in (('report.json', result), ('candidate.json', result['selected'])):
        (args.output/name).write_text(json.dumps(content, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps(dict(model_status=result['model_status'], selected_tau_s=result['selected']['actuator_tau_s'],
                          validation=result['validation'], blockers=result['blockers']), indent=2))


if __name__ == '__main__':
    main()
