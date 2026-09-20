"""Fit the timed two-yaw exploratory model from CAN interval traces.

Offline only. The raw stream is verified before cumulative command integrals are
resampled onto a 4 ms grid. No serial access, controller output, or deployment.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from yaw_can_trace_protocol import BUILD, Decoder
from identify_coupled_offline import DT, TRAIN, TUNE
from identify_stiction_offline import endpoint_velocity, free_run_failures
from identify_timed_stiction_offline import select_timed, evaluate_timed

SAMPLES = 5000
DT_US = 4000
BASELINE_SAMPLES = 500


def joint_delta(raw):
    angle = np.unwrap(np.asarray(raw, dtype=float)*(2*np.pi/8192))
    return np.rad2deg(angle-angle[0])


def resample_integrated_input(boundary_us, areas_raw_us, *, samples=SAMPLES,
                              dt_us=DT_US, start_us=None):
    """Preserve cumulative area while producing uniform interval means.

    Splitting a measured interval distributes its area uniformly because the
    command order inside that interval was intentionally not logged.
    """
    boundary_us = np.asarray(boundary_us, dtype=np.int64)
    areas_raw_us = np.asarray(areas_raw_us, dtype=float)
    if (samples < 2 or dt_us <= 0 or boundary_us.ndim != 1
            or areas_raw_us.shape != (len(boundary_us), 2)
            or not len(boundary_us) or boundary_us[0] < 0
            or (boundary_us[0] == 0 and np.any(areas_raw_us[0] != 0))
            or np.any(np.diff(boundary_us) <= 0) or not np.isfinite(areas_raw_us).all()):
        raise ValueError('finite monotonic trace boundaries and N by 2 areas required')
    start_us = int(boundary_us[0] if start_us is None else start_us)
    grid = start_us+np.arange(samples, dtype=np.int64)*dt_us
    if grid[0] < boundary_us[0] or grid[-1] > boundary_us[-1]:
        raise ValueError('uniform grid lies outside measured trace')
    cumulative = np.cumsum(areas_raw_us, axis=0)
    if boundary_us[0] == 0:
        source_t = boundary_us
    else:
        source_t = np.r_[0, boundary_us]
        cumulative = np.vstack((np.zeros(2), cumulative))
    on_grid = np.column_stack([np.interp(grid, source_t, cumulative[:, axis])
                               for axis in range(2)])
    # integral_raw_us / dt_us gives raw command; /1000 restores Motor.output units.
    means = np.diff(on_grid, axis=0)/dt_us/1000
    return grid, np.vstack((means, means[-1]))


def baseline_centered_current_proxy(current_raw, *, baseline_samples=BASELINE_SAMPLES):
    """Return a scale-conditioned current deviation for a diagnostic fit."""
    current_raw = np.asarray(current_raw, dtype=float)
    if (current_raw.ndim != 2 or current_raw.shape[1] != 2
            or baseline_samples < 1 or baseline_samples > len(current_raw)
            or not np.isfinite(current_raw).all()):
        raise ValueError('finite N by 2 current and valid baseline length required')
    baseline = np.median(current_raw[:baseline_samples], axis=0)
    return (current_raw-baseline)/1000, baseline


def causal_resample(sample_us, values, grid_us):
    """Hold the latest timestamped measurement without using a future sample."""
    sample_us = np.asarray(sample_us, dtype=np.int64)
    values = np.asarray(values, dtype=float)
    grid_us = np.asarray(grid_us, dtype=np.int64)
    if (sample_us.ndim != 1 or values.shape[0] != len(sample_us)
            or grid_us.ndim != 1 or not len(sample_us) or not len(grid_us)
            or np.any(np.diff(sample_us) < 0) or np.any(np.diff(grid_us) <= 0)
            or not np.isfinite(values).all()):
        raise ValueError('ordered timestamped samples and grid required')
    # Repeated timestamps mean no new feedback. Keep the last copy.
    fresh = np.r_[np.diff(sample_us) > 0, True]
    times, observed = sample_us[fresh], values[fresh]
    indices = np.searchsorted(times, grid_us, side='right')-1
    if np.any(indices < 0):
        raise ValueError('grid begins before first measurement')
    return observed[indices], grid_us-times[indices]


def load_trace(directory):
    directory = Path(directory)
    saved = json.loads((directory/'report.json').read_text(encoding='utf-8'))
    raw = (directory/'raw.bin').read_bytes()
    decoder = Decoder(saved['trial_id'])
    for offset in range(0, len(raw), 4096):
        decoder.feed(raw[offset:offset+4096])
    verified = decoder.report()
    if (saved.get('error') or not decoder.complete or decoder.crc_errors
            or decoder.discarded_bytes or decoder.metadata['phase'] != 5
            or decoder.metadata['reason'] or decoder.profile['profile'] != 1
            or decoder.profile['version'] != 3 or decoder.profile['build'] != BUILD):
        raise ValueError('not a complete version-3 motion capture')
    if decoder.metadata != saved['metadata'] or decoder.profile != saved['profile']:
        raise ValueError('raw metadata/profile differs from saved report')
    if set(verified['quality_issues'])-{'irregular_sample_interval'}:
        raise ValueError('unacceptable trace quality: '+str(verified['quality_issues']))
    rows = decoder.rows
    trace_us = np.asarray([row['trace_us'] for row in rows], dtype=np.int64)
    interval_us = np.asarray([row['interval_us'] for row in rows], dtype=np.int64)
    if (len(rows) != 5001 or interval_us[0] != trace_us[0]
            or np.any(np.diff(trace_us) != interval_us[1:])):
        raise ValueError('trace interval boundaries are inconsistent')
    for axis in ('big', 'small'):
        summary = verified['can_summary'][axis]
        counts = summary['interval_counter_sums']
        if (summary['counter_saturated'] or summary['incomplete_coverage_intervals_after_first']
                or any(counts[key] for key in ('failed', 'errors', 'aborted'))
                or counts['queued'] != counts['completed']+summary['terminal_pending']):
            raise ValueError('CAN evidence is incomplete or faulty for '+axis)
    areas = np.asarray([[row['big_integral_raw_us'], row['small_integral_raw_us']]
                        for row in rows], dtype=float)
    grid_us, u = resample_integrated_input(trace_us, areas)
    source_t = trace_us/1e6
    grid_t = grid_us/1e6
    big = joint_delta([row['big_raw'] for row in rows])
    small = joint_delta([row['small_raw'] for row in rows])
    configuration = decoder.metadata['configuration']
    heading = np.asarray([row['yaw_deg'] for row in rows])-configuration['small_heading_anchor_deg']
    measured = np.column_stack((big, small, heading))
    y = np.column_stack([np.interp(grid_t, source_t, measured[:, axis]) for axis in range(3)])
    snapshot = np.asarray([[row['big_command_raw'], row['small_command_raw']]
                           for row in rows], dtype=float)/1000
    snapshot = np.column_stack([np.interp(grid_t, source_t, snapshot[:, axis]) for axis in range(2)])
    current_columns, feedback_age_columns = [], []
    for axis in ('big', 'small'):
        sampled, age = causal_resample(
            [row[axis+'_feedback_us'] for row in rows],
            [row[axis+'_current'] for row in rows], grid_us)
        current_columns.append(sampled); feedback_age_columns.append(age)
    current = np.column_stack(current_columns)
    feedback_age = np.column_stack(feedback_age_columns)
    current_proxy, current_baseline = baseline_centered_current_proxy(current)
    if not all(np.isfinite(value).all() for value in (y, u, snapshot, current)):
        raise ValueError('nonfinite resampled trace')
    source_area = np.sum(areas[1:], axis=0)/1000
    grid_area = np.sum(u[:-1]*DT_US, axis=0)
    audit = {
        'source': str(directory.resolve()),
        'raw_sha256': hashlib.sha256(raw).hexdigest(),
        'trial_id': decoder.metadata['id'],
        'axis': decoder.metadata['axis'],
        'build': decoder.metadata['build'],
        'quality_issues': verified['quality_issues'],
        'trace_interval_us': {'minimum': int(interval_us[1:].min()),
                              'maximum': int(interval_us[1:].max()),
                              'mean': float(interval_us[1:].mean())},
        'can_summary': verified['can_summary'],
        'resampling': 'angles linearly interpolated by trace_us; command cumulative area differenced on 4ms grid',
        # Source includes edge pieces outside the uniform grid, so equality is not expected.
        'source_area_software_command_us_after_first': source_area.tolist(),
        'uniform_grid_area_software_command_us': grid_area.tolist(),
        'grid_start_us': int(grid_us[0]),
        'grid_end_us': int(grid_us[-1]),
        'input_warning': 'attempted CAN command interval mean, not confirmed motor torque; intra-interval order unavailable',
        'feedback_current_proxy': {
            'baseline_median_raw': current_baseline.tolist(),
            'definition': '(latest feedback at/before grid time - first-2s median) / 1000',
            'resampled_feedback_age_us': {
                axis: {'minimum': int(feedback_age[:, index].min()),
                       'maximum': int(feedback_age[:, index].max()),
                       'mean': float(feedback_age[:, index].mean())}
                for index, axis in enumerate(('big', 'small'))},
            'warning': 'causal diagnostic proxy only; not calibrated torque and not interval integrated',
        },
        'interval_integral_vs_snapshot_input': {
            axis: {'difference_rms_software_units': float(np.sqrt(np.mean((u[:, index]-snapshot[:, index])**2))),
                   'difference_max_abs_software_units': float(np.max(np.abs(u[:, index]-snapshot[:, index]))),
                   'integral_input_std_software_units': float(np.std(u[:, index]))}
            for index, axis in enumerate(('big', 'small'))},
    }
    return audit, dict(info=audit, configuration=configuration, y=y,
                       q=np.deg2rad(y[:, :2]), u=u, snapshot_u=snapshot,
                       current_raw=current, current_proxy_u=current_proxy,
                       feedback_age_us=feedback_age,
                       reference_offset_deg=np.interp(grid_t, source_t,
                           [row['offset_cdeg']/100 for row in rows]),
                       grid_us=grid_us)


def prepare_trace_pair(paths):
    loaded = [load_trace(path) for path in paths]
    audits = [item[0] for item in loaded]
    trials = [item[1] for item in loaded]
    if len(trials) != 2 or {item['info']['axis'] for item in trials} != {1, 2}:
        raise ValueError('need one completed big and one completed small trace')
    if len({item['info']['raw_sha256'] for item in trials}) != 2:
        raise ValueError('duplicate traces')
    changing = {'amplitude_deg', 'big_anchor_deg', 'small_heading_anchor_deg',
                'small_joint_start_deg', 'pitch_target_rad', 'imu_roll_start_deg', 'imu_pitch_start_deg'}
    a, b = (item['configuration'] for item in trials)
    if a.keys() != b.keys() or any(a[key] != b[key] for key in a if key not in changing):
        raise ValueError('PID, limits, directions or other fixed configuration changed')
    inputs = np.vstack([trial['u']-trial['u'].mean(axis=0) for trial in trials])
    scale = inputs.std(axis=0)
    if np.any(scale <= 1e-8) or np.linalg.matrix_rank(inputs/scale) != 2:
        raise ValueError('two-axis input excitation is rank deficient')
    for trial in trials:
        trial['v'] = endpoint_velocity(trial['q'])
    return {'trials': audits, 'fixed_configuration_matches': True,
            'input_rank': int(np.linalg.matrix_rank(inputs/scale)),
            'standardized_input_singular_values': np.linalg.svd(inputs/scale, compute_uv=False).tolist()}, trials


def fit_and_score(trials, input_key):
    selected_trials = [{**trial, 'u': trial[input_key]} for trial in trials]
    model, diagnostic, candidates = select_timed(selected_trials)
    validation = [
        {'axis': trial['info']['axis'],
         'rolling': [evaluate_timed(model, trial, TUNE, SAMPLES, 50)],
         'free_run': evaluate_timed(model, trial, 500, SAMPLES, SAMPLES-500)}
        for trial in selected_trials]
    return model, diagnostic, candidates, validation, free_run_failures(validation)


def analyze(paths):
    audit, trials = prepare_trace_pair(paths)
    model, diagnostic, candidates, validation, failures = fit_and_score(trials, 'u')
    _, old_diagnostic, old_candidates, old_validation, old_failures = fit_and_score(trials, 'snapshot_u')
    _, current_diagnostic, current_candidates, current_validation, current_failures = fit_and_score(
        trials, 'current_proxy_u')
    blockers = ['no_independent_version3_reverse_validation', 'closed_loop_identification_bias_unresolved',
                'feedback_current_not_calibrated_to_joint_torque',
                'intra_interval_command_order_unavailable_for_actuator_lag',
                'actuator_initial_state_unknown']
    if diagnostic['at_bound_names']:
        blockers.append('parameters_at_search_bounds')
    if diagnostic['jacobian_rank'] < 8:
        blockers.append('rank_deficient_parameter_fit')
    if failures:
        blockers.append('free_run_heading_not_better_than_hold')
    selected = model.artifact()
    source_dir = Path(__file__).resolve().parent
    implementation = {name: hashlib.sha256((source_dir/name).read_bytes()).hexdigest()
                      for name in ('identify_can_trace_offline.py', 'identify_timed_stiction_offline.py',
                                   'identify_stiction_offline.py', 'identify_weak_stiction_offline.py')}
    return {
        'model_status': 'rejected_exploratory_candidate' if failures else 'experimental_can_trace_candidate',
        'hardware_takeover_allowed': False,
        'audit': audit,
        'selected': selected,
        'candidate_sha256': hashlib.sha256(json.dumps(selected, sort_keys=True,
                                                      separators=(',', ':')).encode()).hexdigest(),
        'implementation_sha256': implementation,
        'fitting': diagnostic,
        'candidates': candidates,
        'validation': validation,
        'free_run_not_better_than_hold_axes': failures,
        'snapshot_input_control': {'fitting': old_diagnostic, 'candidates': old_candidates,
                                   'validation': old_validation,
                                   'free_run_not_better_than_hold_axes': old_failures},
        'feedback_current_proxy_control': {
            'status': 'diagnostic_only_not_calibrated_torque',
            'fitting': current_diagnostic,
            'candidates': current_candidates,
            'validation': current_validation,
            'free_run_not_better_than_hold_axes': current_failures,
        },
        'split_ms': {'train': [0, 7000], 'select': [7000, 14000],
                     'exploratory_check': [14000, 20000]},
        'blockers': blockers,
        'method': 'Timed weak-momentum model using 4ms resampling that preserves measured attempted-command area.',
        'caveats': [
            'Positive-direction big/small pair only; no version-3 independent reverse capture yet.',
            'Recorded future attempted commands are used for prediction; no future measured angles are used.',
            'Interval mean preserves command area, but cannot reconstruct command order inside each interval.',
            'Angle interpolation and endpoint velocity estimation remain sensitive to encoder quantization.',
            'Current is retained as a diagnostic and is not used as calibrated force in this fit.',
            'This is an offline candidate only; no firmware, PID, serial, MPC or motor command is produced.',
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('captures', nargs=2, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output must not exist')
    result = analyze(args.captures)
    args.output.mkdir(exist_ok=False)
    (args.output/'report.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    (args.output/'candidate.json').write_text(json.dumps(result['selected'], indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({'model_status': result['model_status'],
                      'selected_tau_s': result['selected']['actuator_tau_s'],
                      'validation': result['validation'],
                      'blockers': result['blockers']}, indent=2))
    return 0 if result['model_status'] != 'rejected_exploratory_candidate' else 2


if __name__ == '__main__':
    raise SystemExit(main())
