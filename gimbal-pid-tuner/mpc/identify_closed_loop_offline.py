"""Closed-loop and current-timing diagnostics for version-3 CAN traces.

Offline only. It cannot authorize, configure, or command vehicle hardware.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from yaw_can_trace_protocol import Decoder
from identify_can_trace_offline import DT_US, SAMPLES, prepare_trace_pair

DT = DT_US/1e6
TRAIN = 1750
TUNE = 3500
OUTPUTS = ('big_joint_deg', 'small_joint_deg', 'heading_deg')


def decode_bench(directory):
    directory = Path(directory)
    saved = json.loads((directory/'report.json').read_text(encoding='utf-8'))
    raw = (directory/'raw.bin').read_bytes()
    decoder = Decoder(saved['trial_id'])
    decoder.feed(raw)
    verified = decoder.report()
    if (not decoder.complete or decoder.crc_errors or decoder.discarded_bytes
            or decoder.profile['profile'] != 2 or len(decoder.rows) != 5001
            or verified['quality_issues'] or saved.get('error')):
        raise ValueError('invalid zero-output BENCH: '+str(directory))
    result = {'source': str(directory.resolve()),
              'raw_sha256': hashlib.sha256(raw).hexdigest(), 'axes': {}}
    for axis in ('big', 'small'):
        rows = decoder.rows
        if any(row[axis+'_integral_raw_us'] or row[axis+'_command_raw'] for row in rows):
            raise ValueError('BENCH contains nonzero command: '+axis)
        feedback_us = np.asarray([row[axis+'_feedback_us'] for row in rows], dtype=np.int64)
        current = np.asarray([row[axis+'_current'] for row in rows], dtype=float)
        fresh = np.r_[True, np.diff(feedback_us) > 0]
        t, value = feedback_us[fresh]/1e6, current[fresh]
        slope, intercept = np.polyfit(t-t[0], value, 1)
        result['axes'][axis] = {
            'samples': int(len(value)),
            'median_raw': float(np.median(value)),
            'mean_raw': float(np.mean(value)),
            'std_raw': float(np.std(value)),
            'p01_raw': float(np.percentile(value, 1)),
            'p99_raw': float(np.percentile(value, 99)),
            'linear_drift_raw_per_s': float(slope),
            'fitted_initial_raw': float(intercept),
        }
    return result


def first_order_signal(u, delay_samples, tau_s):
    u = np.asarray(u, dtype=float)
    if (u.ndim != 1 or not len(u) or delay_samples < 0 or tau_s < 0
            or not np.isfinite(u).all()):
        raise ValueError('finite scalar input and nonnegative delay/tau required')
    delayed = u[np.maximum(0, np.arange(len(u))-delay_samples)]
    if tau_s == 0:
        return delayed
    alpha = -np.expm1(-DT/tau_s)
    z = np.empty_like(delayed); z[0] = delayed[0]
    for k in range(1, len(z)):
        z[k] = z[k-1]+alpha*(delayed[k]-z[k-1])
    return z


def electrical_fit(trials, benches):
    """Fit command-to-feedback-current timing after independent BENCH bias removal."""
    models = {}
    for axis_index, axis in enumerate(('big', 'small')):
        options = []
        excitation = np.asarray([np.std(t['u'][:, axis_index]) for t in trials])
        fit_indices = np.flatnonzero(excitation >= max(.05, .1*excitation.max()))
        if not len(fit_indices):
            raise ValueError('no electrically excited trial for '+axis)
        scale = max(np.std(np.concatenate([trials[i]['current_raw'][:TRAIN, axis_index]
                                           for i in fit_indices])), 1.)
        for delay in range(4):
            for tau in (0., .004, .008, .016, .032):
                filtered = [first_order_signal(t['u'][:, axis_index]*1000, delay, tau) for t in trials]
                x = np.concatenate([filtered[i][:TRAIN] for i in fit_indices])[:, None]
                y = np.concatenate([trials[i]['current_raw'][:TRAIN, axis_index]-
                                    benches[i]['axes'][axis]['median_raw'] for i in fit_indices])
                # Residual offset is explicit: BENCH median is a measured bias, not forced to explain all DC current.
                design = np.column_stack((x, np.ones(len(x))))
                gain, residual_bias = np.linalg.lstsq(design, y, rcond=None)[0]
                tuning_errors = []
                for i in fit_indices:
                    trial = trials[i]
                    target = trial['current_raw'][TRAIN:TUNE, axis_index]-benches[i]['axes'][axis]['median_raw']
                    prediction = gain*filtered[i][TRAIN:TUNE]+residual_bias
                    tuning_errors.append(prediction-target)
                tuning_rmse = float(np.sqrt(np.mean(np.concatenate(tuning_errors)**2)))
                options.append((tuning_rmse/scale, delay, tau, float(gain), float(residual_bias), filtered))
        score, delay, tau, gain, residual_bias, filtered = min(options, key=lambda item: item[0])
        validations = []
        for i, trial in enumerate(trials):
            segments = {}
            for name, start, end in (('train', 0, TRAIN), ('tune', TRAIN, TUNE),
                                     ('test', TUNE, SAMPLES)):
                target = trial['current_raw'][start:end, axis_index]-benches[i]['axes'][axis]['median_raw']
                prediction = gain*filtered[i][start:end]+residual_bias
                error = prediction-target
                baseline = target-np.mean(target)
                segments[name] = {
                    'rmse_raw': float(np.sqrt(np.mean(error**2))),
                    'mean_error_raw': float(np.mean(error)),
                    'r2_vs_segment_mean': float(1-np.sum(error**2)/max(np.sum(baseline**2), 1e-12)),
                }
            validations.append({'trial_axis': trial['info']['axis'], 'segments': segments})
        training_command = np.concatenate([trials[i]['u'][:TRAIN, axis_index]*1000 for i in fit_indices])
        both_polarities = bool(training_command.min() < -50 and training_command.max() > 50)
        active_test_bias_shift = any(
            abs(validations[i]['segments']['test']['mean_error_raw']) >
            3*benches[i]['axes'][axis]['std_raw'] for i in fit_indices)
        models[axis] = {
            'status': ('one_sided_training_extrapolation' if not both_polarities else
                       'held_out_bias_shift' if active_test_bias_shift else 'diagnostic_fit'),
            'delay_samples': delay, 'effective_delay_ms': delay*DT*1000,
            'tau_ms': tau*1000, 'gain_current_raw_per_command_raw': gain,
            'residual_bias_raw_after_bench_subtraction': residual_bias,
            'full_record_command_std_software_units': excitation.tolist(),
            'fit_trial_axes': [trials[i]['info']['axis'] for i in fit_indices],
            'training_command_range_raw': [float(training_command.min()), float(training_command.max())],
            'training_has_both_polarities_above_50_raw': both_polarities,
            'selection_normalized_rmse': score, 'validation': validations,
            'warning': 'Effective delay is limited to 4ms resolution and includes command aggregation and feedback timing.',
        }
    return models


def fir_features(reference, memory, delay):
    reference = np.asarray(reference, dtype=float)
    if reference.ndim != 2 or reference.shape[1] != 2 or memory < 1 or delay < 0:
        raise ValueError('N by 2 reference and valid FIR structure required')
    delta = np.vstack((reference[0], np.diff(reference, axis=0)))
    x = np.zeros((len(reference), 2+2*memory))
    x[:, :2] = reference
    for lag in range(memory):
        source = np.arange(len(reference))-delay-lag
        valid = source >= 0
        x[valid, 2+2*lag:4+2*lag] = delta[source[valid]]
    return x


def scaled_ridge(x, y, ridge):
    xs = np.maximum(np.std(x, axis=0), 1e-9)
    ys = np.maximum(np.std(y, axis=0), 360/8192)
    xn, yn = x/xs, y/ys
    augmented = np.vstack((xn, np.sqrt(len(x)*ridge)*np.eye(x.shape[1])))
    target = np.vstack((yn, np.zeros((x.shape[1], y.shape[1]))))
    coefficient = np.linalg.lstsq(augmented, target, rcond=None)[0]
    return coefficient*ys/xs[:, None]


def error_metrics(prediction, measured, scale):
    error = np.asarray(prediction)-np.asarray(measured)
    return {'rmse_deg': np.sqrt(np.mean(error**2, axis=0)).tolist(),
            'max_abs_deg': np.max(np.abs(error), axis=0).tolist(),
            'normalized_rmse': float(np.sqrt(np.mean((error/scale)**2)))}


def closed_loop_fir(trials):
    """Identify the exogenous reference-to-output closed-loop map."""
    scales = np.maximum(np.std(np.vstack([t['y'][:TRAIN] for t in trials]), axis=0), .1)
    options = []
    for memory in (25, 50, 125, 250):
        for delay in (0, 1, 2):
            feature = [fir_features(t['reference'], memory, delay) for t in trials]
            for ridge in (1e-6, 1e-3, .1):
                coefficient = scaled_ridge(np.vstack([x[:TRAIN] for x in feature]),
                                           np.vstack([t['y'][:TRAIN] for t in trials]), ridge)
                score = np.mean([error_metrics(x[TRAIN:TUNE]@coefficient,
                                               t['y'][TRAIN:TUNE], scales)['normalized_rmse']**2
                                 for x, t in zip(feature, trials)])
                options.append((float(score), memory, delay, ridge, coefficient, feature))
    score, memory, delay, ridge, coefficient, feature = min(options, key=lambda item: item[0])
    validation = []
    for x, trial in zip(feature, trials):
        prediction = x@coefficient
        static = x[:, :2]@coefficient[:2]
        segments = {}
        for name, start, end in (('train', 0, TRAIN), ('tune', TRAIN, TUNE),
                                 ('test', TUNE, SAMPLES)):
            segments[name] = {'model': error_metrics(prediction[start:end], trial['y'][start:end], scales),
                              'static_reference': error_metrics(static[start:end], trial['y'][start:end], scales)}
        validation.append({'trial_axis': trial['info']['axis'], 'segments': segments})
    train_x = np.vstack([x[:TRAIN] for x in feature])
    s = np.linalg.svd(train_x/np.maximum(np.std(train_x, axis=0), 1e-9), compute_uv=False)
    return {
        'status': 'descriptive_closed_loop_map_only',
        'input': 'exogenous scheduled reference offset, split into big/small selected-axis channels',
        'output_names': OUTPUTS, 'memory_samples': memory, 'memory_ms': memory*DT*1000,
        'delay_samples': delay, 'ridge': ridge, 'tuning_score': score,
        'static_gain_deg_per_reference_deg': coefficient[:2].tolist(),
        'training_design_rank': int(np.linalg.matrix_rank(train_x)),
        'training_design_columns': int(train_x.shape[1]),
        'scaled_design_condition': float(s[0]/s[-1]) if s[-1] > 0 else None,
        'validation': validation,
        'coefficient': coefficient.tolist(),
        'warning': 'This identifies the existing PID closed loop, not an open-loop plant for MPC.',
    }


def iv_plant_audit(trials, na=2, nb=2, delay=1, instrument_lags=25):
    """Two-stage least-squares audit using scheduled reference lags as instruments."""
    history = max(na, delay+nb-1, instrument_lags-1)
    x_rows, z_rows, targets = [], [], []
    for trial_index, trial in enumerate(trials):
        q, current, reference = trial['q_deg'], trial['current_corrected']/1000, trial['reference']
        for k in range(history, TRAIN):
            dummy = np.zeros(len(trials)); dummy[trial_index] = 1
            x_rows.append(np.r_[*[q[k-lag] for lag in range(1, na+1)],
                                *[current[k-delay-lag] for lag in range(nb)], dummy])
            z_rows.append(np.r_[*[reference[k-lag] for lag in range(instrument_lags)], dummy])
            targets.append(q[k])
    x, z, target = map(np.asarray, (x_rows, z_rows, targets))
    x_scale = np.maximum(x.std(axis=0), 1e-9)
    z_scale = np.maximum(z.std(axis=0), 1e-9)
    xn, zn = x/x_scale, z/z_scale
    z_singular = np.linalg.svd(zn, compute_uv=False)
    projection_coefficient = np.linalg.lstsq(zn, xn, rcond=1e-10)[0]
    xhat = zn@projection_coefficient
    xhat_singular = np.linalg.svd(xhat, compute_uv=False)
    first_stage_r2 = []
    for column in range(x.shape[1]):
        residual = xn[:, column]-xhat[:, column]
        centered = xn[:, column]-xn[:, column].mean()
        first_stage_r2.append(float(1-np.sum(residual**2)/max(np.sum(centered**2), 1e-12)))
    projected_rank = int(np.linalg.matrix_rank(xhat, tol=xhat_singular[0]*1e-8))
    required_rank = x.shape[1]
    condition = float(xhat_singular[0]/xhat_singular[required_rank-1]) \
        if len(xhat_singular) >= required_rank and xhat_singular[required_rank-1] > 0 else None
    result = {
        'status': 'insufficient_instruments' if projected_rank < required_rank or condition is None
                  or condition > 1e8 or min(first_stage_r2) < .05 else 'exploratory_iv_fit',
        'structure': {'output_lags': na, 'current_lags': nb, 'current_delay_samples': delay,
                      'reference_instrument_lags': instrument_lags},
        'rows': int(len(x)), 'regressor_columns': required_rank,
        'instrument_columns': int(z.shape[1]),
        'instrument_rank_1e-8': int(np.sum(z_singular > z_singular[0]*1e-8)),
        'projected_regressor_rank_1e-8': projected_rank,
        'projected_regressor_condition': condition,
        'first_stage_r2': first_stage_r2,
        'warning': 'Reference is the only external instrument; slow single-profile excitation can be weak despite algebraic rank.',
    }
    if result['status'] != 'exploratory_iv_fit':
        return result
    coefficient_scaled = np.linalg.lstsq(xhat, target, rcond=1e-10)[0]
    coefficient = coefficient_scaled/x_scale[:, None]
    result['coefficient'] = coefficient.tolist()
    companion = np.zeros((2*na, 2*na))
    companion[:2] = coefficient[:2*na].T
    if na > 1:
        companion[2:, :-2] = np.eye(2*(na-1))
    result['autonomous_spectral_radius'] = float(np.max(np.abs(np.linalg.eigvals(companion))))
    # This is forced-response validation with recorded future current, not a full closed-loop simulation.
    validation = []
    for trial_index, trial in enumerate(trials):
        entries = {}
        for name, start, end in (('tune', TRAIN, TUNE), ('test', TUNE, SAMPLES)):
            predicted = np.array(trial['q_deg'], copy=True)
            dummy = np.zeros(len(trials)); dummy[trial_index] = 1
            diverged = False
            for k in range(start, end):
                feature = np.r_[*[predicted[k-lag] for lag in range(1, na+1)],
                                *[trial['current_corrected'][k-delay-lag]/1000 for lag in range(nb)], dummy]
                predicted[k] = feature@coefficient
                if not np.isfinite(predicted[k]).all() or np.max(np.abs(predicted[k])) > 1e5:
                    diverged = True; break
            entries[name] = {'diverged': diverged}
            if not diverged:
                scale = np.maximum(np.std(trial['q_deg'][:TRAIN], axis=0), .1)
                entries[name]['model'] = error_metrics(predicted[start:end], trial['q_deg'][start:end], scale)
                hold = np.broadcast_to(trial['q_deg'][start-1], (end-start, 2))
                entries[name]['hold'] = error_metrics(hold, trial['q_deg'][start:end], scale)
        validation.append({'trial_axis': trial['info']['axis'], 'segments': entries})
    result['forced_response_validation'] = validation
    failed = result['autonomous_spectral_radius'] > 1.000001 or any(
        segment.get('diverged') or segment['model']['normalized_rmse'] >= segment['hold']['normalized_rmse']
        for item in validation for segment in item['segments'].values())
    if failed:
        result['status'] = 'rejected_iv_fit'
    return result


def analyze(motion_paths):
    pair_audit, loaded = prepare_trace_pair(motion_paths)
    benches = [decode_bench(Path(path).parent/'bench') for path in motion_paths]
    trials = []
    for i, trial in enumerate(loaded):
        q_deg = np.rad2deg(trial['q']-trial['q'][0])
        heading = trial['y'][:, 2]-trial['y'][0, 2]
        reference = np.zeros((SAMPLES, 2))
        reference[:, trial['info']['axis']-1] = trial['reference_offset_deg']
        bias = np.array([benches[i]['axes'][axis]['median_raw'] for axis in ('big', 'small')])
        trials.append({**trial, 'q_deg': q_deg, 'y': np.column_stack((q_deg, heading)),
                       'reference': reference, 'current_corrected': trial['current_raw']-bias})
    electrical = electrical_fit(trials, benches)
    closed_loop = closed_loop_fir(trials)
    iv = iv_plant_audit(trials)
    blockers = ['closed_loop_map_is_not_open_loop_plant', 'feedback_current_not_torque_calibrated',
                'single_slow_profile_per_reference_channel', 'no_independent_version3_validation']
    if iv['status'] == 'insufficient_instruments':
        blockers.append('reference_instruments_insufficient_for_plant_identification')
    else:
        blockers.append('iv_plant_forced_response_validation_failed')
    if any(m['validation'][i]['segments']['test']['r2_vs_segment_mean'] < 0
           for m in electrical.values() for i, trial in enumerate(trials)
           if trial['info']['axis'] in m['fit_trial_axes']):
        blockers.append('electrical_model_fails_a_held_out_segment')
    if any(not m['training_has_both_polarities_above_50_raw'] for m in electrical.values()):
        blockers.append('electrical_training_missing_both_polarities')
    if any(m['status'] == 'held_out_bias_shift' for m in electrical.values()):
        blockers.append('electrical_bias_not_stationary_across_held_out_segment')
    implementation = Path(__file__).resolve()
    report = {
        'model_status': 'diagnostic_only_not_identified', 'hardware_takeover_allowed': False,
        'pair_audit': pair_audit, 'bench_bias': benches,
        'bench_bias_session_shift_raw': {
            axis: benches[1]['axes'][axis]['median_raw']-benches[0]['axes'][axis]['median_raw']
            for axis in ('big', 'small')},
        'current_timing': {
            'method': 'latest feedback timestamp at or before each 4ms grid node; no future interpolation',
            'per_trial_resampled_age_us': [t['info']['feedback_current_proxy']['resampled_feedback_age_us']
                                           for t in trials],
            'limitations': ['Only the latest feedback frame present at each trace snapshot is retained.',
                            'Command order inside a trace interval is unavailable; electrical delay resolution is about 4ms.'],
        },
        'command_to_current': electrical,
        'closed_loop_reference_model': closed_loop,
        'instrumental_variable_plant_audit': iv,
        'split_ms': {'train': [0, 7000], 'select': [7000, 14000], 'test': [14000, 20000]},
        'blockers': blockers,
        'implementation_sha256': {
            implementation.name: hashlib.sha256(implementation.read_bytes()).hexdigest(),
            'identify_can_trace_offline.py': hashlib.sha256(
                (implementation.parent/'identify_can_trace_offline.py').read_bytes()).hexdigest(),
        },
        'conclusion': 'Current timing/bias and the existing PID closed loop are diagnosable, but these two slow profiles do not establish an MPC plant.',
    }
    return report


def write_diagnostic_csv(report, motion_paths, output):
    """Export measured and fitted traces without changing the frozen JSON result."""
    _, loaded = prepare_trace_pair(motion_paths)
    coefficient = np.asarray(report['closed_loop_reference_model']['coefficient'])
    memory = report['closed_loop_reference_model']['memory_samples']
    delay = report['closed_loop_reference_model']['delay_samples']
    names = []
    for i, trial in enumerate(loaded):
        q_deg = np.rad2deg(trial['q']-trial['q'][0])
        heading = trial['y'][:, 2]-trial['y'][0, 2]
        measured = np.column_stack((q_deg, heading))
        reference = np.zeros((SAMPLES, 2))
        reference[:, trial['info']['axis']-1] = trial['reference_offset_deg']
        closed_prediction = fir_features(reference, memory, delay)@coefficient
        bias = np.array([report['bench_bias'][i]['axes'][axis]['median_raw']
                         for axis in ('big', 'small')])
        corrected = trial['current_raw']-bias
        electrical_prediction = np.empty_like(corrected)
        for axis_index, axis in enumerate(('big', 'small')):
            model = report['command_to_current'][axis]
            filtered = first_order_signal(trial['u'][:, axis_index]*1000,
                                          model['delay_samples'], model['tau_ms']/1000)
            electrical_prediction[:, axis_index] = (model['gain_current_raw_per_command_raw']*filtered+
                                                     model['residual_bias_raw_after_bench_subtraction'])
        table = np.column_stack((np.arange(SAMPLES)*DT, reference, measured, closed_prediction,
                                 trial['current_raw'], corrected, electrical_prediction,
                                 trial['feedback_age_us']))
        name = 'axis%d_diagnostics.csv' % trial['info']['axis']
        np.savetxt(output/name, table, delimiter=',', comments='', fmt='%.9g',
                   header='time_s,reference_big_deg,reference_small_deg,'
                          'measured_big_deg,measured_small_deg,measured_heading_deg,'
                          'closed_model_big_deg,closed_model_small_deg,closed_model_heading_deg,'
                          'feedback_big_current_raw,feedback_small_current_raw,'
                          'corrected_big_current_raw,corrected_small_current_raw,'
                          'electrical_model_corrected_big_current_raw,electrical_model_corrected_small_current_raw,'
                          'big_feedback_age_us,small_feedback_age_us')
        names.append(name)
    return names


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('captures', nargs=2, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output must not exist')
    result = analyze(args.captures)
    args.output.mkdir(exist_ok=False)
    result['diagnostic_csv'] = write_diagnostic_csv(result, args.captures, args.output)
    (args.output/'report.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({'model_status': result['model_status'],
                      'command_to_current': result['command_to_current'],
                      'closed_loop_reference_model': {k: result['closed_loop_reference_model'][k]
                          for k in ('status', 'memory_ms', 'delay_samples', 'ridge', 'tuning_score', 'validation')},
                      'instrumental_variable_plant_audit': result['instrumental_variable_plant_audit'],
                      'blockers': result['blockers']}, indent=2))
    return 2


if __name__ == '__main__':
    raise SystemExit(main())
