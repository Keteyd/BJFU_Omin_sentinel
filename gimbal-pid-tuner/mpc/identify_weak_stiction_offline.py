"""Experimental angle-weighted stiction identification, with no hardware I/O.

Weighted sums transfer discrete differences from quantized positions to a
smooth compact weight. No estimated endpoint acceleration/velocity enters
the momentum or damping terms. Uncertain friction signs remain intervals.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from identify_coupled_offline import (DT, TRAIN, TUNE, LOW, HIGH, PARAMETERS,
                                      parameters, prepare, evaluate, lagged_input)
from identify_stiction_offline import ENCODER_RAD, StickSlip, endpoint_velocity, free_run_failures

WINDOW = 100  # 400 ms, chosen for synthetic development, not vehicle tuning.
SIGN_RADIUS = 5  # 40 ms centered position change, training only.


def weak_rows(trials, tau):
    s = np.linspace(-1., 1., WINDOW+1)
    w = (1-s*s)**2
    # For j in [k,k+WINDOW], q[j+1]-2q[j]+q[j-1] and q[j+1]-q[j].
    acceleration = np.convolve(w, [1., -2., 1.])/DT
    displacement = np.r_[0., -w, 0.] + np.r_[0., 0., w]
    rows = []
    for trial in trials:
        q = trial['q'][:TRAIN]
        u = lagged_input(trial['u'][:TRAIN], tau)
        if len(q) != TRAIN or len(u) != TRAIN or not np.isfinite(q).all() or not np.isfinite(u).all():
            raise ValueError('finite 7 second training records required')
        # A difference exactly equal to two counts must remain uncertain after
        # a change of angular origin; floating-point subtraction can straddle it.
        sign_threshold = 2*ENCODER_RAD + 32*np.finfo(float).eps*max(1., float(np.abs(q).max()))
        for k in range(SIGN_RADIUS, TRAIN-WINDOW-SIGN_RADIUS, 10):
            indices = np.arange(k, k+WINDOW+1)
            delta = q[indices+SIGN_RADIUS]-q[indices-SIGN_RADIUS]
            signs = np.where(np.abs(delta) > sign_threshold, np.sign(delta), 0.)
            # Sign estimation is centered within TRAIN, never used to initialize
            # predictions. Zero here means uncertain, not physically stationary.
            local = q[k-1:k+WINDOW+2]-q[k-1]
            rows.append(np.r_[acceleration @ local, displacement @ local,
                              DT*w @ signs, DT*w @ u[indices], DT*w @ (signs == 0)])
    if not rows:
        raise ValueError('no weak training windows')
    return np.array(rows)


def weak_residual(p, rows, scales):
    m, d, f, g = parameters(p)
    needed = rows[:, 6:8]*g - rows[:, :2] @ m.T - rows[:, 2:4]*d - rows[:, 4:6]*f
    allowance = rows[:, 8:10]*f
    error = np.sign(needed)*np.maximum(np.abs(needed)-allowance, 0.)
    return (error/(np.diag(m)*scales)).ravel()


def fit_weak_stiction(trials, tau):
    rows = weak_rows(trials, tau)
    scales = np.maximum(rows[:, :2].std(axis=0), .01)
    fits = []
    for ratio in (.3, 3.):
        initial = np.array([np.log(ratio), .3, 0., 0., -2., -2., -1., -1.])
        result = least_squares(weak_residual, initial, args=(rows, scales),
                               bounds=(LOW, HIGH), loss='soft_l1', f_scale=.2, max_nfev=600)
        if result.success and np.isfinite(result.cost):
            fits.append(result)
    if not fits:
        raise ValueError('weak stiction fit failed')
    best = min(fits, key=lambda r: r.cost)
    condition = float(np.linalg.cond(best.jac))
    return StickSlip(best.x, tau), dict(
        method='400ms weighted discrete momentum, interval-valued uncertain friction signs',
        cost=float(best.cost), nfev=best.nfev, windows=len(rows),
        at_bound_names=[PARAMETERS[i] for i in np.flatnonzero(np.minimum(best.x-LOW, HIGH-best.x) < .01)],
        jacobian_rank=int(np.linalg.matrix_rank(best.jac)),
        jacobian_condition=condition if np.isfinite(condition) else None)


def analyze(paths, allow_jitter):
    audit, trials = prepare(paths, allow_jitter)
    for trial in trials:
        trial['v'] = endpoint_velocity(trial['q'])
    candidates, choices = [], []
    for tau in (0., .02, .05):
        model, diagnostic = fit_weak_stiction(trials, tau)
        scores = [evaluate(model, t, TRAIN, TUNE, 50) for t in trials]
        score = float(np.mean([np.mean(np.square(s['model']['rmse_deg'])) for s in scores]))
        candidates.append(dict(tau=tau, tuning_score=score, fitting=diagnostic))
        choices.append((score, model, diagnostic))
    _, model, diagnostic = min(choices, key=lambda c: c[0])
    validation = [dict(axis=t['info']['axis'],
                       rolling=[evaluate(model, t, TUNE, 5000, h) for h in (25, 50, 125)],
                       free_run=evaluate(model, t, 500, 5000, 4500)) for t in trials]
    failures = free_run_failures(validation)
    blockers = ['experimental_method_not_independently_validated', 'closed_loop_bias_unresolved',
                '4ms_output_snapshots_not_complete_1ms_CAN_input_history',
                'friction_sign_uncertainty_and_external_load_unresolved', 'absolute_torque_scale_unknown']
    if diagnostic['at_bound_names']:
        blockers.append('parameters_at_search_bounds')
    if diagnostic['jacobian_rank'] < 8:
        blockers.append('rank_deficient_parameter_fit')
    if failures:
        blockers.append('free_run_heading_not_better_than_hold')
    artifact = model.artifact()
    artifact['identification_method'] = diagnostic['method']
    return dict(model_status='rejected_exploratory_candidate' if failures else 'experimental_weak_stiction_candidate',
                hardware_takeover_allowed=False, selected=artifact, audit=audit,
                free_run_not_better_than_hold_axes=failures, candidates=candidates, validation=validation,
                split_ms=dict(train=[0, 7000], tune=[7000, 14000], exploratory_check=[14000, 20000]),
                blockers=blockers,
                caveats=['Original two trials only; reverse trial excluded.',
                         'Weighted fit uses only training angles and commands; centered sign estimates stay inside training.',
                         'Prediction initialization still uses causal 40ms quadratic endpoint velocity.',
                         'Two-count sign threshold does not bound all possible physical or measurement errors.',
                         'Already inspected original data and synthetic development are not independent vehicle validation.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('captures', nargs=2, type=Path)
    parser.add_argument('--allow-one-ms-jitter', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output must not exist')
    result = analyze(args.captures, args.allow_one_ms_jitter)
    args.output.mkdir(exist_ok=False)
    for name, data in (('report.json', result), ('candidate.json', result['selected'])):
        (args.output/name).write_text(json.dumps(data, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({k: result[k] for k in ('model_status', 'selected', 'validation', 'blockers')}, indent=2))


if __name__ == '__main__':
    main()
