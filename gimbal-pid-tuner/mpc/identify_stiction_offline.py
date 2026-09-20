"""New offline candidate: time-aligned velocity and set-valued Coulomb friction.

Old fitted artifacts and their validation code are intentionally unchanged.
"""

import argparse
import itertools
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from identify_coupled_offline import (Coupled, DT, HISTORY, TRAIN, TUNE, LOW, HIGH,
                                      PARAMETERS, parameters, prepare, evaluate, lagged_input)

ENCODER_RAD = 2*np.pi/8192


def endpoint_velocity(q):
    t = (np.arange(HISTORY)-(HISTORY-1))*DT
    weights = np.linalg.pinv(np.column_stack((np.ones(HISTORY), t, t*t)))[1]
    v = np.full_like(q, np.nan)
    for k in range(HISTORY-1, len(q)):
        v[k] = weights @ q[k-HISTORY+1:k+1]
    return v


class StickSlip(Coupled):
    def __post_init__(self):
        super().__post_init__()
        self.implicit = self.mass + DT*np.diag(self.damping)

    def step(self, x, u):
        # Backward-Euler Coulomb inclusion: zero velocity admits any friction
        # within +/-F. Enumerate the nine active sets of this two-variable QP.
        single = np.asarray(x).ndim == 1
        x = np.atleast_2d(x)
        u = np.broadcast_to(u, (len(x), 2))
        rhs = x[:, 2:] @ self.mass.T + DT*u*self.gain
        v = np.zeros((len(x), 2))
        found = np.zeros(len(x), dtype=bool)
        for signs in itertools.product((0, -1, 1), repeat=2):
            signs = np.array(signs)
            free = np.flatnonzero(signs)
            fixed = np.flatnonzero(signs == 0)
            candidate = np.zeros_like(v)
            if len(free):
                a = self.implicit[np.ix_(free, free)]
                force = rhs[:, free] - DT*self.friction[free]*signs[free]
                candidate[:, free] = np.linalg.solve(a, force.T).T
            valid = np.all(candidate[:, free]*signs[free] >= -1e-12, axis=1)
            reaction = rhs - candidate @ self.implicit.T
            valid &= np.all(np.abs(reaction[:, fixed]) <= DT*self.friction[fixed]+1e-12, axis=1)
            take = valid & ~found
            v[take] = candidate[take]
            found |= take
        if not found.all():
            raise FloatingPointError('friction active-set solve failed')
        result = np.column_stack((x[:, :2]+DT*v, v))
        return result[0] if single else result

    def artifact(self):
        a = super().artifact()
        a.update(model_family='normalized_two_joint_set_valued_coulomb_v1',
                 friction_law='v!=0: F*sign(v); v=0: friction in [-F,+F]',
                 integrator='4ms implicit Euler, exact two-variable active-set solve',
                 velocity_estimator='causal 40ms quadratic endpoint slope',
                 friction_smoothing_rad_s=None)
        return a


def interval_data(trials, tau):
    data, stationary = [], []
    for trial in trials:
        q, v = trial['q'], trial['v']
        u = lagged_input(trial['u'][:TRAIN], tau)
        for k in range(HISTORY-1, TRAIN-25, 10):
            end = k+25
            data.append(np.concatenate((v[end]-v[k], q[end]-q[k],
                                        DT*np.sign(v[k:end]).sum(axis=0),
                                        DT*u[k:end].sum(axis=0))))
            stationary.append(np.ptp(q[k:end+1], axis=0) <= 2*ENCODER_RAD)
    return np.array(data), np.array(stationary)


def residual(p, rows, stationary, scales):
    m, d, f, g = parameters(p)
    needed = rows[:, 6:8]*g - rows[:, :2] @ m.T - rows[:, 2:4]*d
    moving_error = needed - rows[:, 4:6]*f
    holding_error = np.maximum(np.abs(needed) - .1*f, 0)
    # Per-axis acceleration-equivalent normalization prevents mass shrinkage.
    error = np.where(stationary, holding_error, moving_error)
    return (error/(np.diag(m)*scales)).ravel()


def fit_stiction(trials, tau):
    rows, stationary = interval_data(trials, tau)
    scales = np.maximum(rows[:, :2].std(axis=0), .01)
    fits = []
    for ratio in (.3, 3.):
        initial = np.array([np.log(ratio), .3, 0., 0., -2., -2., -1., -1.])
        result = least_squares(residual, initial, args=(rows, stationary, scales),
                               bounds=(LOW, HIGH), loss='soft_l1', f_scale=.2, max_nfev=600)
        if result.success and np.isfinite(result.cost):
            fits.append(result)
    if not fits:
        raise ValueError('stiction fit failed')
    best = min(fits, key=lambda r: r.cost)
    return StickSlip(best.x, tau), dict(
        cost=float(best.cost), nfev=best.nfev,
        near_stationary_windows=stationary.sum(axis=0).tolist(),
        moving_windows=(~stationary).sum(axis=0).tolist(),
        at_bound_names=[PARAMETERS[i] for i in np.flatnonzero(np.minimum(best.x-LOW, HIGH-best.x) < .01)],
        jacobian_condition=float(np.linalg.cond(best.jac)))


def free_run_failures(validation):
    return [v['axis'] for v in validation
            if v['free_run']['model']['rmse_deg'][2] >= v['free_run']['hold']['rmse_deg'][2]]


def analyze(paths, allow_jitter):
    audit, trials = prepare(paths, allow_jitter)
    for trial in trials:
        trial['v'] = endpoint_velocity(trial['q'])
    choices, candidates = [], []
    for tau in (0., .02, .05):
        model, diagnostic = fit_stiction(trials, tau)
        scores = [evaluate(model, t, TRAIN, TUNE, 50) for t in trials]
        score = float(np.mean([np.mean(np.square(s['model']['rmse_deg'])) for s in scores]))
        candidates.append(dict(tau=tau, tuning_score=score, fitting=diagnostic))
        choices.append((score, model, diagnostic))
    _, model, diagnostic = min(choices, key=lambda c: c[0])
    validation = [dict(axis=t['info']['axis'],
                       rolling=[evaluate(model, t, TUNE, 5000, h) for h in (25, 50, 125)],
                       free_run=evaluate(model, t, 500, 5000, 4500)) for t in trials]
    blockers = ['new_candidate_not_independently_validated', 'closed_loop_bias_unresolved',
                '4ms_output_snapshots_not_complete_1ms_CAN_input_history',
                'static_holding_may_include_electrical_deadzone_or_external_load',
                'absolute_torque_scale_unknown']
    if diagnostic['at_bound_names']:
        blockers.append('parameters_at_search_bounds')
    failed_axes = free_run_failures(validation)
    if failed_axes:
        blockers.append('free_run_heading_not_better_than_hold')
    return dict(model_status='rejected_exploratory_candidate' if failed_axes else 'experimental_stiction_candidate',
                hardware_takeover_allowed=False, free_run_not_better_than_hold_axes=failed_axes,
                audit=audit, selected=model.artifact(), candidates=candidates, validation=validation,
                split_ms=dict(train=[0, 7000], tune=[7000, 14000], exploratory_check=[14000, 20000]),
                blockers=blockers,
                caveats=['Original two trials only; reverse validation trial not used.',
                         'Near stationary means <=2 encoder counts span over 100ms, not proof of zero physical speed.',
                         'Uses equal static/kinetic Coulomb bound as a first model, not identified Stribeck friction.',
                         'Different velocity estimator changes prediction initialization and baseline errors.',
                         'A causal endpoint estimate remains noisy; it does not restore missing CAN samples.',
                         'Whole-run checks on these original trials are not independent validation.'])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('captures', nargs=2, type=Path)
    p.add_argument('--allow-one-ms-jitter', action='store_true')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    result = analyze(args.captures, args.allow_one_ms_jitter)
    args.output.mkdir(exist_ok=False)
    for name, content in (('report.json', result), ('candidate.json', result['selected'])):
        (args.output/name).write_text(json.dumps(content, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({k: result[k] for k in ('selected', 'validation', 'blockers')}, indent=2))


if __name__ == '__main__':
    main()
