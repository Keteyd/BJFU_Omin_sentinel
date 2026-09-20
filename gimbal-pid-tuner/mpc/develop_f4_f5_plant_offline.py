"""Develop command-to-joint candidates in the cable-free F4/F5 work domain.

F4 alone selects coefficients and structure. F5 is evaluated without fitting
or selection. Results remain development evidence because both captures were
already inspected before this analysis and commanded input is endogenous to
the existing PID loop. This module has no serial or hardware-control path.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


DT = .020
TRAIN = (3., 18.)
TUNE = (18., 26.)
TEST = (26., 31.)
ACTIVE = (3., 31.)
SETTLE = (31., 36.)
OUTPUTS = ('big_joint_deg', 'small_joint_deg')
INSTRUMENT_LAGS = 48
GRID = tuple((na, nb, delay, ridge, friction)
             for na in (2, 4, 8)
             for nb in (1, 2, 4)
             for delay in (0, 1, 2)
             for ridge in (1e-4, 1e-2, 1.)
             for friction in (False, True))
STATE_GRID = tuple((na, nb, delay, ridge, position, friction)
                   for na in (1, 2, 4)
                   for nb in (1, 2, 4)
                   for delay in (0, 1, 2)
                   for ridge in (1e-4, 1e-2, 1.)
                   for position in (False, True)
                   for friction in (False, True))


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest().upper()


def load_phase(directory, name):
    path = Path(directory)/(name+'.csv')
    values = np.genfromtxt(path, delimiter=',', names=True)
    get = lambda key: np.asarray(values[key], dtype=float)[::5]
    result = {
        'time': get('time_s'),
        'reference': np.column_stack((get('reference_big_deg'),
                                      get('reference_heading_deg'))),
        'q': np.column_stack((get('measured_big_joint_deg'),
                              get('measured_small_joint_deg'))),
        'heading': get('measured_heading_deg'),
        'u': np.column_stack((get('command_big'), get('command_small'))),
        'current': np.column_stack((get('current_big_raw'), get('current_small_raw'))),
        'source_csv': str(path.resolve()),
        'source_csv_sha256': sha256(path),
    }
    if len(result['time']) != 1800 or not all(
            np.isfinite(value).all() for key, value in result.items()
            if isinstance(value, np.ndarray)):
        raise ValueError(name+' does not contain a finite 36 s/20 ms grid')
    return result


def with_baseline_centered_current_input(data):
    result = dict(data)
    baseline = indices(data, (0., 3.))
    result['u'] = (data['current']-np.median(data['current'][baseline], axis=0))/1000
    return result


def indices(data, bounds):
    return np.flatnonzero((data['time'] >= bounds[0]-1e-9) &
                          (data['time'] < bounds[1]-1e-9))


def model_scales(data):
    pick = indices(data, TRAIN)
    return {
        'q_center': data['q'][pick].mean(axis=0),
        'q_scale': np.maximum(data['q'][pick].std(axis=0), .1),
        'u_center': data['u'][pick].mean(axis=0),
        'u_scale': np.maximum(data['u'][pick].std(axis=0), .01),
        'r_center': data['reference'][pick].mean(axis=0),
        'r_scale': np.maximum(data['reference'][pick].std(axis=0), .01),
    }


def normalized(data, scales):
    return {
        'q': (data['q']-scales['q_center'])/scales['q_scale'],
        'u': (data['u']-scales['u_center'])/scales['u_scale'],
        'r': (data['reference']-scales['r_center'])/scales['r_scale'],
    }


def causal_velocity(q, window=3):
    """Backward finite-window velocity; no future measurement leakage."""
    q = np.asarray(q, dtype=float)
    velocity = np.zeros_like(q)
    velocity[window:] = (q[window:]-q[:-window])/(window*DT)
    velocity[:window] = velocity[window]
    return velocity


def feature(q, u, k, na, nb, delay, friction):
    parts = [q[k-lag] for lag in range(1, na+1)]
    parts += [u[k-delay-lag] for lag in range(nb)]
    if friction:
        parts += [np.sign(q[k-1]-q[k-2])]
    parts += [np.ones(1)]
    return np.concatenate(parts)


def instruments(r, k, friction):
    parts = [r[k-lag] for lag in range(1, INSTRUMENT_LAGS+1)]
    if friction:
        parts += [np.sign(r[k-lag]-r[k-lag-1]) for lag in range(1, 13)]
    parts += [np.ones(1)]
    return np.concatenate(parts)


def spectral_radius(coefficient, na):
    companion = np.zeros((2*na, 2*na))
    companion[:2] = np.hstack([
        coefficient[2*lag:2*(lag+1)].T for lag in range(na)
    ])
    if na > 1:
        companion[2:, :-2] = np.eye(2*(na-1))
    return float(np.max(np.abs(np.linalg.eigvals(companion))))


def fit_candidate(data, method, na, nb, delay, ridge, friction, scales):
    signals = normalized(data, scales)
    pick = indices(data, TRAIN)
    history = max(na, delay+nb-1, INSTRUMENT_LAGS+1)
    pick = pick[pick >= history]
    x = np.asarray([feature(signals['q'], signals['u'], k, na, nb, delay, friction)
                    for k in pick])
    y = signals['q'][pick]
    penalty = np.eye(x.shape[1]); penalty[-1, -1] = 0
    first_stage = None
    if method == 'ols':
        coefficient = np.linalg.solve(
            x.T@x+len(x)*ridge*penalty, x.T@y)
    else:
        z = np.asarray([instruments(signals['r'], k, friction) for k in pick])
        z_scale = np.maximum(z.std(axis=0), 1e-9); z_scale[-1] = 1
        zn = z/z_scale
        projection = np.linalg.lstsq(zn, x, rcond=1e-10)[0]
        xhat = zn@projection
        centered = x[:, :-1]-x[:, :-1].mean(axis=0)
        residual = x[:, :-1]-xhat[:, :-1]
        r2 = 1-np.sum(residual**2, axis=0)/np.maximum(
            np.sum(centered**2, axis=0), 1e-12)
        singular = np.linalg.svd(xhat, compute_uv=False)
        rank = int(np.linalg.matrix_rank(xhat, tol=singular[0]*1e-8))
        first_stage = {
            'minimum_regressor_r2': float(np.min(r2)),
            'median_regressor_r2': float(np.median(r2)),
            'projected_rank': rank,
            'regressor_columns': int(x.shape[1]),
            'projected_condition': (float(singular[0]/singular[x.shape[1]-1])
                                    if rank >= x.shape[1] else None),
        }
        coefficient = np.linalg.solve(
            xhat.T@xhat+len(xhat)*ridge*penalty, xhat.T@y)
    return {
        'method': method, 'na': na, 'nb': nb, 'delay': delay,
        'ridge': ridge, 'friction': friction,
        'coefficient': coefficient,
        'spectral_radius_linear_part': spectral_radius(coefficient, na),
        'first_stage': first_stage,
    }


def simulate(data, model, scales, bounds):
    signals = normalized(data, scales)
    q = np.array(signals['q'], copy=True)
    pick = indices(data, bounds)
    first, last = int(pick[0]), int(pick[-1]+1)
    for k in range(first, last):
        q[k] = feature(q, signals['u'], k, model['na'], model['nb'],
                       model['delay'], model['friction'])@model['coefficient']
        if not np.isfinite(q[k]).all() or np.max(np.abs(q[k])) > 100:
            return None
    return q*scales['q_scale']+scales['q_center']


def error_metrics(prediction, measured):
    error = prediction-measured
    rmse = np.sqrt(np.mean(error**2, axis=0))
    std = np.maximum(np.std(measured, axis=0), .1)
    return {
        'rmse_deg': rmse.tolist(),
        'normalized_rmse_by_segment_std': (rmse/std).tolist(),
        'mean_error_deg': np.mean(error, axis=0).tolist(),
        'maximum_abs_error_deg': np.max(np.abs(error), axis=0).tolist(),
    }


def evaluate_segment(data, model, scales, bounds):
    pick = indices(data, bounds)
    prediction = simulate(data, model, scales, bounds)
    if prediction is None:
        return {'diverged': True}
    measured = data['q'][pick]
    held = np.repeat(data['q'][pick[0]-1:pick[0]], len(pick), axis=0)
    model_metrics = error_metrics(prediction[pick], measured)
    hold_metrics = error_metrics(held, measured)
    model_rmse = np.asarray(model_metrics['rmse_deg'])
    hold_rmse = np.asarray(hold_metrics['rmse_deg'])
    return {
        'diverged': False,
        'model': model_metrics,
        'hold': hold_metrics,
        'improvement_over_hold_fraction':
            (1-model_rmse/np.maximum(hold_rmse, 1e-12)).tolist(),
    }


def select_model(f4, method):
    scales = model_scales(f4)
    candidates = []
    for args in GRID:
        model = fit_candidate(f4, method, *args, scales)
        tune = evaluate_segment(f4, model, scales, TUNE)
        continuous = evaluate_segment(f4, model, scales, ACTIVE)
        first = model['first_stage']
        instrument_ok = (method == 'ols' or (first['projected_rank'] ==
                         first['regressor_columns'] and
                         first['projected_condition'] is not None and
                         first['projected_condition'] < 1e8 and
                         first['minimum_regressor_r2'] > .02))
        stable = (model['spectral_radius_linear_part'] <= 1.005 and
                  not tune['diverged'] and not continuous['diverged'])
        if stable and instrument_ok:
            score = float(np.mean(tune['model']['normalized_rmse_by_segment_std']))
            candidates.append((score, model))
    if not candidates:
        return None, scales, {'tested': len(GRID), 'eligible': 0}
    score, selected = min(candidates, key=lambda item: item[0])
    return selected, scales, {
        'tested': len(GRID), 'eligible': len(candidates),
        'selection_source': 'F4 18-26 s conditional forced response only',
        'selection_score_mean_normalized_rmse': score,
    }


def state_scales(data):
    scales = model_scales(data)
    velocity = causal_velocity(data['q'])
    pick = indices(data, TRAIN)
    scales['v_center'] = velocity[pick].mean(axis=0)
    scales['v_scale'] = np.maximum(velocity[pick].std(axis=0), .2)
    return scales


def state_feature(qn, vn, un, k, na, nb, delay, position, friction):
    parts = [vn[k-lag] for lag in range(1, na+1)]
    if position:
        parts += [qn[k-1]]
    parts += [un[k-delay-lag] for lag in range(nb)]
    if friction:
        parts += [np.sign(vn[k-1])]
    parts += [np.ones(1)]
    return np.concatenate(parts)


def state_radius(coefficient, na, position, scales):
    # Linear part of [q, v(k), ..., v(k-na+1)]. Friction and input are omitted.
    n = 2+2*na
    a = np.zeros((n, n))
    offset = 2*na
    qcoef = coefficient[offset:offset+2] if position else np.zeros((2, 2))
    q_actual = (np.diag(scales['v_scale']) @ qcoef.T @
                np.diag(1/scales['q_scale']))
    vblocks = []
    for lag in range(na):
        block = (np.diag(scales['v_scale']) @
                 coefficient[2*lag:2*(lag+1)].T @
                 np.diag(1/scales['v_scale']))
        vblocks.append(block)
    a[2:4, :2] = q_actual
    a[2:4, 2:] = np.hstack(vblocks)
    a[:2, :2] = np.eye(2)+DT*q_actual
    a[:2, 2:] = DT*np.hstack(vblocks)
    if na > 1:
        a[4:, 2:-2] = np.eye(2*(na-1))
    return float(np.max(np.abs(np.linalg.eigvals(a))))


def fit_state_candidate(data, method, na, nb, delay, ridge, position,
                        friction, scales):
    qn = (data['q']-scales['q_center'])/scales['q_scale']
    velocity = causal_velocity(data['q'])
    vn = (velocity-scales['v_center'])/scales['v_scale']
    un = (data['u']-scales['u_center'])/scales['u_scale']
    rn = (data['reference']-scales['r_center'])/scales['r_scale']
    pick = indices(data, TRAIN)
    history = max(na, delay+nb-1, INSTRUMENT_LAGS+1, 4)
    pick = pick[pick >= history]
    x = np.asarray([state_feature(qn, vn, un, k, na, nb, delay,
                                  position, friction) for k in pick])
    y = vn[pick]
    penalty = np.eye(x.shape[1]); penalty[-1, -1] = 0
    first_stage = None
    if method == 'ols':
        coefficient = np.linalg.solve(x.T@x+len(x)*ridge*penalty, x.T@y)
    else:
        z = np.asarray([instruments(rn, k, friction) for k in pick])
        z_scale = np.maximum(z.std(axis=0), 1e-9); z_scale[-1] = 1
        zn = z/z_scale
        xhat = zn@np.linalg.lstsq(zn, x, rcond=1e-10)[0]
        centered = x[:, :-1]-x[:, :-1].mean(axis=0)
        residual = x[:, :-1]-xhat[:, :-1]
        r2 = 1-np.sum(residual**2, axis=0)/np.maximum(
            np.sum(centered**2, axis=0), 1e-12)
        singular = np.linalg.svd(xhat, compute_uv=False)
        rank = int(np.linalg.matrix_rank(xhat, tol=singular[0]*1e-8))
        first_stage = {
            'minimum_regressor_r2': float(np.min(r2)),
            'median_regressor_r2': float(np.median(r2)),
            'projected_rank': rank,
            'regressor_columns': int(x.shape[1]),
            'projected_condition': (float(singular[0]/singular[x.shape[1]-1])
                                    if rank >= x.shape[1] else None),
        }
        coefficient = np.linalg.solve(
            xhat.T@xhat+len(xhat)*ridge*penalty, xhat.T@y)
    return {
        'method': method, 'na': na, 'nb': nb, 'delay': delay,
        'ridge': ridge, 'position': position, 'friction': friction,
        'coefficient': coefficient,
        'spectral_radius_linear_part': state_radius(
            coefficient, na, position, scales),
        'first_stage': first_stage,
    }


def simulate_state(data, model, scales, bounds):
    qn = (data['q']-scales['q_center'])/scales['q_scale']
    velocity = causal_velocity(data['q'])
    vn = (velocity-scales['v_center'])/scales['v_scale']
    un = (data['u']-scales['u_center'])/scales['u_scale']
    # Estimate the run-specific constant disturbance from the motion-free
    # 0--3 s prefix. This is available before excitation and avoids treating
    # sensor/load/holding-current offset as a universal constant.
    adapted_coefficient = np.array(model['coefficient'], copy=True)
    history = max(model['na'], model['delay']+model['nb']-1, 4)
    baseline = indices(data, (0., 3.))
    baseline = baseline[baseline >= history]
    without_intercept = np.asarray([
        state_feature(qn, vn, un, k, model['na'], model['nb'],
                      model['delay'], model['position'], model['friction'])[:-1]
        for k in baseline
    ])@adapted_coefficient[:-1]
    adapted_coefficient[-1] = np.median(vn[baseline]-without_intercept, axis=0)
    qn, vn = np.array(qn, copy=True), np.array(vn, copy=True)
    pick = indices(data, bounds)
    first, last = int(pick[0]), int(pick[-1]+1)
    for k in range(first, last):
        vn[k] = state_feature(qn, vn, un, k, model['na'], model['nb'],
                              model['delay'], model['position'],
                              model['friction'])@adapted_coefficient
        velocity_actual = vn[k]*scales['v_scale']+scales['v_center']
        q_previous = qn[k-1]*scales['q_scale']+scales['q_center']
        q_actual = q_previous+DT*velocity_actual
        qn[k] = (q_actual-scales['q_center'])/scales['q_scale']
        if (not np.isfinite(qn[k]).all() or not np.isfinite(vn[k]).all()
                or np.max(np.abs(q_actual)) > 100):
            return None
    return qn*scales['q_scale']+scales['q_center']


def evaluate_state_segment(data, model, scales, bounds):
    pick = indices(data, bounds)
    prediction = simulate_state(data, model, scales, bounds)
    if prediction is None:
        return {'diverged': True}
    measured = data['q'][pick]
    held = np.repeat(data['q'][pick[0]-1:pick[0]], len(pick), axis=0)
    model_metrics = error_metrics(prediction[pick], measured)
    hold_metrics = error_metrics(held, measured)
    model_rmse = np.asarray(model_metrics['rmse_deg'])
    hold_rmse = np.asarray(hold_metrics['rmse_deg'])
    return {
        'diverged': False, 'model': model_metrics, 'hold': hold_metrics,
        'improvement_over_hold_fraction':
            (1-model_rmse/np.maximum(hold_rmse, 1e-12)).tolist(),
    }


def adapted_state_intercept(data, model, scales):
    qn = (data['q']-scales['q_center'])/scales['q_scale']
    velocity = causal_velocity(data['q'])
    vn = (velocity-scales['v_center'])/scales['v_scale']
    un = (data['u']-scales['u_center'])/scales['u_scale']
    history = max(model['na'], model['delay']+model['nb']-1, 4)
    baseline = indices(data, (0., 3.)); baseline = baseline[baseline >= history]
    x = np.asarray([state_feature(qn, vn, un, k, model['na'], model['nb'],
                                  model['delay'], model['position'],
                                  model['friction'])[:-1] for k in baseline])
    return np.median(vn[baseline]-x@model['coefficient'][:-1], axis=0).tolist()


def rolling_state_metrics(data, model, scales, bounds=ACTIVE):
    active = indices(data, bounds)
    result = {}
    for horizon_s in (.1, .2, .5, 1.):
        steps = int(round(horizon_s/DT))
        errors, holds = [], []
        for first in range(int(active[0]), int(active[-1])-steps+1, 5):
            prediction = simulate_state(
                data, model, scales,
                (float(data['time'][first+1]),
                 float(data['time'][first+steps]+DT*.5)))
            if prediction is None:
                return {'diverged': True}
            target = data['q'][first+1:first+steps+1]
            errors.append(prediction[first+1:first+steps+1]-target)
            holds.append(data['q'][first]-target)
        error, hold = np.vstack(errors), np.vstack(holds)
        rmse = np.sqrt(np.mean(error**2, axis=0))
        hold_rmse = np.sqrt(np.mean(hold**2, axis=0))
        result[str(int(horizon_s*1000))+'_ms'] = {
            'model_rmse_deg': rmse.tolist(),
            'hold_rmse_deg': hold_rmse.tolist(),
            'improvement_over_hold_fraction':
                (1-rmse/np.maximum(hold_rmse, 1e-12)).tolist(),
        }
    return result


def select_state_model(f4, method):
    scales = state_scales(f4)
    candidates = []
    for args in STATE_GRID:
        model = fit_state_candidate(f4, method, *args, scales)
        tune = evaluate_state_segment(f4, model, scales, TUNE)
        continuous = evaluate_state_segment(f4, model, scales, ACTIVE)
        first = model['first_stage']
        instrument_ok = (method == 'ols' or (first['projected_rank'] ==
                         first['regressor_columns'] and
                         first['projected_condition'] is not None and
                         first['projected_condition'] < 1e8 and
                         first['minimum_regressor_r2'] > .02))
        stable = (model['spectral_radius_linear_part'] <= 1.005 and
                  not tune['diverged'] and not continuous['diverged'])
        if stable and instrument_ok:
            score = float(np.mean(tune['model']['normalized_rmse_by_segment_std']))
            candidates.append((score, model))
    if not candidates:
        return None, scales, {'tested': len(STATE_GRID), 'eligible': 0}
    score, selected = min(candidates, key=lambda item: item[0])
    return selected, scales, {
        'tested': len(STATE_GRID), 'eligible': len(candidates),
        'selection_source': 'F4 18-26 s conditional forced response only',
        'selection_score_mean_normalized_rmse': score,
    }


def serializable_model(model, scales):
    if model is None:
        return None
    result = {key: value for key, value in model.items() if key != 'coefficient'}
    result['coefficient'] = model['coefficient'].tolist()
    result['normalization'] = {key: value.tolist() for key, value in scales.items()}
    result['sample_period_ms'] = DT*1000
    return result


def fit_heading_observation(f4):
    pick = indices(f4, TRAIN)
    x = np.column_stack((f4['q'][pick], np.ones(len(pick))))
    coefficient = np.linalg.lstsq(x, f4['heading'][pick], rcond=None)[0]
    return coefficient


def heading_metrics(data, coefficient, bounds):
    pick = indices(data, bounds)
    prediction = np.column_stack((data['q'][pick], np.ones(len(pick))))@coefficient
    error = prediction-data['heading'][pick]
    held = np.repeat(data['heading'][pick[0]-1], len(pick))
    hold_error = held-data['heading'][pick]
    rmse = float(np.sqrt(np.mean(error**2)))
    hold_rmse = float(np.sqrt(np.mean(hold_error**2)))
    return {
        'rmse_deg': rmse,
        'mean_error_deg': float(np.mean(error)),
        'maximum_abs_error_deg': float(np.max(np.abs(error))),
        'hold_rmse_deg': hold_rmse,
        'improvement_over_hold_fraction': 1-rmse/max(hold_rmse, 1e-12),
    }


def analyze(source_dir):
    source_dir = Path(source_dir)
    upstream = json.loads((source_dir/'report.json').read_text(encoding='utf-8'))
    if upstream['analysis_status'] != 'completed_development_offline_comparison':
        raise ValueError('upstream cable-free analysis is not accepted')
    f4, f5 = load_phase(source_dir, 'F4_cable_free'), load_phase(source_dir, 'F5_cable_free')
    models = {}
    for method in ('ols', 'iv'):
        model, scales, search = select_model(f4, method)
        entry = {'search': search, 'model': serializable_model(model, scales)}
        if model is not None:
            entry['evaluation'] = {
                'F4_train': evaluate_segment(f4, model, scales, TRAIN),
                'F4_tune': evaluate_segment(f4, model, scales, TUNE),
                'F4_late_unseen_by_selection': evaluate_segment(f4, model, scales, TEST),
                'F4_continuous_3_31s': evaluate_segment(f4, model, scales, ACTIVE),
                'F4_settle': evaluate_segment(f4, model, scales, SETTLE),
                'F5_cross_amplitude_3_31s': evaluate_segment(f5, model, scales, ACTIVE),
                'F5_settle': evaluate_segment(f5, model, scales, SETTLE),
            }
        models[method] = entry

    state_models = {}
    for method in ('ols', 'iv'):
        model, scales, search = select_state_model(f4, method)
        entry = {'search': search, 'model': serializable_model(model, scales)}
        if model is not None:
            entry['baseline_adapted_intercept_normalized_velocity'] = {
                'F4': adapted_state_intercept(f4, model, scales),
                'F5': adapted_state_intercept(f5, model, scales),
            }
            entry['evaluation'] = {
                'F4_train': evaluate_state_segment(f4, model, scales, TRAIN),
                'F4_tune': evaluate_state_segment(f4, model, scales, TUNE),
                'F4_late_unseen_by_selection': evaluate_state_segment(f4, model, scales, TEST),
                'F4_continuous_3_31s': evaluate_state_segment(f4, model, scales, ACTIVE),
                'F4_settle': evaluate_state_segment(f4, model, scales, SETTLE),
                'F5_cross_amplitude_3_31s': evaluate_state_segment(f5, model, scales, ACTIVE),
                'F5_settle': evaluate_state_segment(f5, model, scales, SETTLE),
            }
            entry['F5_rolling_prediction'] = rolling_state_metrics(f5, model, scales)
        state_models[method] = entry

    current_f4 = with_baseline_centered_current_input(f4)
    current_f5 = with_baseline_centered_current_input(f5)
    current_state_models = {}
    for method in ('ols', 'iv'):
        model, scales, search = select_state_model(current_f4, method)
        entry = {'search': search, 'model': serializable_model(model, scales)}
        if model is not None:
            entry['baseline_adapted_intercept_normalized_velocity'] = {
                'F4': adapted_state_intercept(current_f4, model, scales),
                'F5': adapted_state_intercept(current_f5, model, scales),
            }
            entry['evaluation'] = {
                'F4_train': evaluate_state_segment(current_f4, model, scales, TRAIN),
                'F4_tune': evaluate_state_segment(current_f4, model, scales, TUNE),
                'F4_late_unseen_by_selection': evaluate_state_segment(current_f4, model, scales, TEST),
                'F4_continuous_3_31s': evaluate_state_segment(current_f4, model, scales, ACTIVE),
                'F4_settle': evaluate_state_segment(current_f4, model, scales, SETTLE),
                'F5_cross_amplitude_3_31s': evaluate_state_segment(current_f5, model, scales, ACTIVE),
                'F5_settle': evaluate_state_segment(current_f5, model, scales, SETTLE),
            }
            entry['F5_rolling_prediction'] = rolling_state_metrics(
                current_f5, model, scales)
        current_state_models[method] = entry

    observation = fit_heading_observation(f4)
    heading = {
        'coefficient_heading_from_big_small_and_intercept': observation.tolist(),
        'selection_source': 'F4 3-18 s only',
        'F4_late': heading_metrics(f4, observation, TEST),
        'F5_active': heading_metrics(f5, observation, ACTIVE),
        'warning': 'static local observation audit; tilted/offset geometry is not proven',
    }

    iv = state_models['iv']
    iv_ok = False
    if iv.get('evaluation'):
        check = iv['evaluation']['F5_cross_amplitude_3_31s']
        settle = iv['evaluation']['F5_settle']
        first = iv['model']['first_stage']
        iv_ok = bool(not check['diverged'] and
                     min(check['improvement_over_hold_fraction']) >= .20 and
                     not settle['diverged'] and
                     min(settle['improvement_over_hold_fraction']) >= 0 and
                     first['minimum_regressor_r2'] >= .05 and
                     iv['model']['spectral_radius_linear_part'] <= 1.001)
    return {
        'analysis_status': 'completed_F4_selected_F5_fixed_evaluation',
        'model_status': ('development_IV_candidate_meets_diagnostic_gates'
                         if iv_ok else 'open_loop_plant_not_identified'),
        'hardware_takeover_allowed': False,
        'operating_domain': {
            'primary': 'cable-free F4/F5, four-to-five times phase-E reference amplitude',
            'R1_role': 'cable and low-speed corner diagnostic; excluded from plant selection',
            'sample_period_ms': DT*1000,
        },
        'source': {
            'directory': str(source_dir.resolve()),
            'upstream_report_sha256': sha256(source_dir/'report.json'),
            'F4': {key: f4[key] for key in ('source_csv', 'source_csv_sha256')},
            'F5': {key: f5[key] for key in ('source_csv', 'source_csv_sha256')},
        },
        'selection_contract': {
            'fit': 'F4 3-18 s', 'select': 'F4 18-26 s',
            'internal_late_check': 'F4 26-31 s',
            'cross_amplitude_check': 'F5 3-31 s; never used for coefficient or structure selection',
            'position_ARX_candidate_grid_count_per_method': len(GRID),
            'fixed_kinematics_state_candidate_grid_count_per_method': len(STATE_GRID),
            'IV_diagnostic_gate': 'F5 active both joints >=20% better than hold, F5 settle no worse than hold, first-stage min R2 >=0.05, linear radius <=1.001',
            'formal_independence': False,
        },
        'command_to_joint_candidates': models,
        'fixed_kinematics_velocity_state_candidates': state_models,
        'feedback_current_to_joint_candidates': current_state_models,
        'heading_observation': heading,
        'blockers': [
            'software_command_is_endogenous_to_existing_PID_feedback',
            'F5_was_already_inspected_before_this_model_selection_contract',
            'feedback_current_is_not_calibrated_joint_torque',
            'no_new_independent_operating_domain_waveform',
        ],
        'baseline_adaptation': ('Each fixed-kinematics state candidate replaces only its constant '
                                'velocity-disturbance intercept using the causal 0--3 s prefix; '
                                'dynamic coefficients remain frozen from F4.'),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-dir', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output must not exist')
    report = analyze(args.source_dir)
    args.output.mkdir(parents=True)
    (args.output/'report.json').write_text(
        json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({key: report[key] for key in
                      ('analysis_status', 'model_status', 'hardware_takeover_allowed')},
                     indent=2))


if __name__ == '__main__':
    main()
