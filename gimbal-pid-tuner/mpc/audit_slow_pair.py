"""Offline-only streaming capture audit. No parameter writes or model deployment."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from yaw_slow_protocol import Decoder


def joint_delta(raw):
    angle = np.unwrap(np.asarray(raw, dtype=float) * (2 * np.pi / 8192))
    return np.rad2deg(angle - angle[0])


def timing(ticks):
    elapsed = (np.asarray(ticks, dtype=np.int64) - int(ticks[0])) % (2**32)
    dt = np.diff(elapsed)
    if len(ticks) != 5001 or np.any(dt <= 0) or elapsed[-1] != 20000:
        raise ValueError('incomplete/nonmonotonic 20-second capture')
    grid_error = elapsed - np.arange(len(ticks)) * 4
    values, counts = np.unique(dt, return_counts=True)
    return elapsed / 1000, dict(
        interval_counts_ms={str(k): int(v) for k, v in zip(values, counts)},
        max_grid_error_ms=int(np.max(np.abs(grid_error))),
        irregular_indices=(np.flatnonzero(dt != 4) + 1).tolist(),
        exploratory_timing_acceptable=bool(np.all((dt >= 3) & (dt <= 5))
                                         and np.max(np.abs(grid_error)) <= 1))


def bounds(values):
    return dict(min=float(np.min(values)), max=float(np.max(values)))


def integrate_rate(t, rate):
    rate = np.asarray(rate, dtype=float)
    return np.concatenate(([0.], np.cumsum(np.diff(t) * (rate[1:] + rate[:-1]) / 2)))


def load(directory):
    directory = Path(directory)
    report = json.loads((directory / 'report.json').read_text(encoding='utf-8'))
    raw = (directory / 'raw.bin').read_bytes()
    d = Decoder(report['trial_id'])
    d.feed(raw)
    verified = d.report()
    if (report.get('error') or not d.complete or d.metadata['phase'] != 5
            or d.metadata['reason'] or d.profile['profile'] != 1):
        raise ValueError('not a completed motion capture')
    if d.metadata != report['metadata'] or d.profile != report['profile']:
        raise ValueError('raw metadata/profile differs from report')
    if set(verified['quality_issues']) - {'irregular_sample_interval'}:
        raise ValueError('unacceptable capture quality: ' + str(verified['quality_issues']))
    rows = d.rows
    if rows[0]['tick_ms'] != d.metadata['start_ms']:
        raise ValueError('start timestamp mismatch')
    t, clock = timing([r['tick_ms'] for r in rows])
    if not clock['exploratory_timing_acceptable']:
        raise ValueError('timing exceeds explicit exploratory audit tolerance')
    c = d.metadata['configuration']
    big = joint_delta([r['big_raw'] for r in rows])
    small = joint_delta([r['small_raw'] for r in rows])
    heading = np.array([r['yaw_deg'] for r in rows]) - c['small_heading_anchor_deg']
    y = np.column_stack((big, small, heading))
    u = np.array([[r['big_command_raw'], r['small_command_raw']] for r in rows]) / 1000
    if not np.isfinite(y).all() or not np.isfinite(u).all():
        raise ValueError('nonfinite observations')
    # This is only a geometry diagnostic under fixed-base, aligned-axis assumptions.
    # It is not a dynamic plant model or a calibration to apply to firmware.
    relative = y - y[t < 2].mean(axis=0)
    closure = relative[:, 2] - relative[:, 0] - relative[:, 1]
    # The live small-yaw speed loop uses gyro Z. This sampled integral is a
    # diagnostic, not a replacement for the full-rate attitude estimator.
    gyro_z = np.rad2deg(np.array([r['gyro_z_rad_s'] for r in rows]))
    integrated_z = integrate_rate(t, gyro_z)
    integrated_z -= integrated_z[t < 2].mean()
    plateaus = []
    for start, end in ((0, 2), (6, 7), (13, 14), (19, 20)):
        mask = (t >= start) & (t < end)
        plateaus.append(dict(seconds=[start, end],
                             relative_mean_deg=relative[mask].mean(axis=0).tolist(),
                             heading_minus_joint_sum_deg=float(closure[mask].mean()),
                             gyro_z_integral_deg=float(integrated_z[mask].mean()),
                             gyro_z_mean_dps=float(gyro_z[mask].mean())))
    audit = dict(source=str(directory.resolve()), sha256=hashlib.sha256(raw).hexdigest(),
                 trial_id=d.metadata['id'], axis=d.metadata['axis'], build=d.metadata['build'],
                 raw_quality_issues=verified['quality_issues'], timing=clock,
                 output_order=['big_joint_delta', 'small_joint_delta', 'imu_heading_delta'],
                 response_range_deg=[bounds(y[:, i]) for i in range(3)],
                 command_range_software_units=[bounds(u[:, i]) for i in range(2)],
                 plateaus=plateaus, aligned_fixed_base_closure_range_deg=bounds(closure),
                 gyro_z_integral_minus_heading_range_deg=bounds(integrated_z-relative[:, 2]),
                 input_unit_warning='Software command, not measured torque or current')
    return audit, y, u, t, c


def review(directories):
    trials = [load(path) for path in directories]
    audits = [v[0] for v in trials]
    if {a['axis'] for a in audits} != {1, 2} or len(trials) != 2:
        raise ValueError('need one big and one small capture')
    if len({a['sha256'] for a in audits}) != 2 or len({a['build'] for a in audits}) != 1:
        raise ValueError('duplicate capture or mismatched builds')
    changing = {'amplitude_deg', 'big_anchor_deg', 'small_heading_anchor_deg',
                'small_joint_start_deg', 'pitch_target_rad', 'imu_roll_start_deg', 'imu_pitch_start_deg'}
    a, b = trials[0][-1], trials[1][-1]
    if a.keys() != b.keys() or any(a[k] != b[k] for k in a if k not in changing):
        raise ValueError('PID, limits or other fixed configuration changed')
    pose_difference = {k: float(b[k]-a[k]) for k in
                       ('small_joint_start_deg', 'imu_roll_start_deg', 'imu_pitch_start_deg', 'pitch_target_rad')}
    inputs = np.vstack([v[2][:-1] - v[2][:-1].mean(axis=0) for v in trials])
    scales = inputs.std(axis=0)
    if np.any(scales <= 1e-8):
        raise ValueError('missing input excitation')
    singular = np.linalg.svd(inputs / scales, compute_uv=False)
    # Fit only a descriptive angle relation with an independent offset for each trial.
    x, h = [], []
    for index, (_, y, _, _, _) in enumerate(trials):
        offsets = np.zeros((len(y)-1, 2))
        offsets[:, index] = 1
        x.append(np.column_stack((y[:-1, :2], offsets)))
        h.append(y[:-1, 2])
    x, h = np.vstack(x), np.concatenate(h)
    coefficient = np.linalg.lstsq(x, h, rcond=None)[0]
    residual = h - x @ coefficient
    return dict(hardware_takeover_allowed=False, model_status='not_identified', trials=audits,
                pose_difference=pose_difference, fixed_configuration_matches=True,
                standardized_input_singular_values=singular.tolist(),
                input_rank=int(np.linalg.matrix_rank(inputs)),
                angle_relation_diagnostic=dict(
                    big_coefficient=float(coefficient[0]), small_coefficient=float(coefficient[1]),
                    per_trial_offsets_deg=coefficient[2:].tolist(),
                    rmse_deg=float(np.sqrt(np.mean(residual**2))),
                    warning='Descriptive fit only; not inertia, proof of fixed base, or firmware calibration'),
                next_step='Verify fixed-base/axis geometry before physical-model interpretation; independent validation still required')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('captures', nargs=2, type=Path)
    p.add_argument('--output', required=True, type=Path)
    args = p.parse_args()
    result = review(args.captures)
    with args.output.open('x', encoding='utf-8') as f:
        json.dump(result, f, indent=2, allow_nan=False)
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
