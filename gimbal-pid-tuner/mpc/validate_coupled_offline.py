"""Read-only independent capture validation. Never fit or export a controller."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from audit_slow_pair import load
from identify_coupled_offline import Coupled, VS, causal_velocity, evaluate
from identify_slow_offline import uniform_samples


def frozen_model(artifact):
    if (artifact.get('angle_scales') != [1., 1.]
            or artifact.get('heading_matrix') != [1., 1., 0., 0.]
            or artifact.get('state_order') != ['q_big', 'q_small', 'v_big', 'v_small']
            or artifact.get('coordinates') != 'q_big relative to base; q_small relative to big; radians'
            or artifact.get('hardware_takeover_allowed') is not False
            or artifact.get('friction_smoothing_rad_s') != VS):
        raise ValueError('unsupported model coordinates or angle/force conventions')
    m = np.asarray(artifact['normalized_mass'], dtype=float)
    vectors = [np.asarray(artifact[k], dtype=float) for k in ('damping', 'coulomb_friction', 'command_gain')]
    if (m.shape != (2, 2) or not np.isfinite(m).all() or m[0, 0] != 1.
            or not np.allclose(m, m.T, rtol=0, atol=1e-12)
            or np.any(np.linalg.eigvalsh(m) <= 0)
            or any(v.shape != (2,) or not np.isfinite(v).all() or np.any(v <= 0) for v in vectors)):
        raise ValueError('invalid normalized dynamics')
    rho = m[0, 1] / (.95*np.sqrt(m[1, 1]))
    tau = float(artifact['actuator_tau_s'])
    if not abs(rho) < 1 or not np.isfinite(tau) or tau < 0:
        raise ValueError('invalid mass coupling or actuator response')
    p = np.concatenate(([np.log(m[1, 1]), np.arctanh(rho)], *[np.log(v) for v in vectors]))
    model = Coupled(p, tau)
    if not np.allclose(model.mass, m, rtol=1e-12, atol=1e-12):
        raise ValueError('model reconstruction changed mass')
    return model


def check_configuration(reference, actual):
    mutable = {'amplitude_deg', 'big_anchor_deg', 'small_heading_anchor_deg', 'small_joint_start_deg',
               'pitch_target_rad', 'imu_roll_start_deg', 'imu_pitch_start_deg'}
    if reference.keys() != actual.keys():
        raise ValueError('configuration fields differ')
    different = [k for k in reference if k not in mutable and reference[k] != actual[k]]
    if different:
        raise ValueError('fixed control configuration changed: ' + ', '.join(different))
    return {k: float(actual[k]-reference[k]) for k in sorted(mutable & reference.keys())}


def validate(model_directory, capture, allow_jitter=False):
    directory = Path(model_directory)
    training_bytes = (directory/'report.json').read_bytes()
    artifact_bytes = (directory/'candidate.json').read_bytes()
    training = json.loads(training_bytes)
    artifact = json.loads(artifact_bytes)
    if artifact != training['selected']:
        raise ValueError('candidate differs from frozen training report')
    model = frozen_model(artifact)
    info, y, u, t, config = load(capture)
    if info['sha256'] in {v['sha256'] for v in training['audit']['trials']}:
        raise ValueError('capture was already used to train/select this candidate')
    source = json.loads((Path(capture)/'report.json').read_text(encoding='utf-8'))
    if source['profile']['reverse'] != 1 or info['axis'] != 2 or config['amplitude_deg'] != 10.:
        raise ValueError('this verification expects the independent reverse small-yaw 10deg trial')
    reference = next(v for v in training['audit']['trials'] if v['axis'] == info['axis'])
    original, _, _, _, reference_config = load(reference['source'])
    if original['sha256'] != reference['sha256'] or info['build'] != original['build']:
        raise ValueError('training provenance or firmware mismatch')
    differences = check_configuration(reference_config, config)
    if info['raw_quality_issues'] and not allow_jitter:
        raise ValueError('explicit reviewed one-ms jitter permission required')
    y, u = uniform_samples(t, y, u)
    q = np.deg2rad(y[:, :2])
    trial = dict(y=y, q=q, v=causal_velocity(q), u=u)
    sections = {}
    for name, start, end in (('whole_record', 11, 5000), ('motion_section', 500, 4000),
                              ('return_and_settle', 3500, 5000)):
        sections[name] = [evaluate(model, trial, start, end, h) for h in (25, 50, 125)]
    sections['free_run_from_2s'] = [evaluate(model, trial, 500, 5000, 4500)]
    blockers = list(training['blockers'])
    # No deployment gate is relaxed just because one validation trial completes.
    blockers.append('independent_big_axis_validation_still_missing')
    if any(abs(differences[k]) > tol for k, tol in
           (('small_joint_start_deg', 2.), ('imu_roll_start_deg', .5), ('imu_pitch_start_deg', .5),
            ('pitch_target_rad', .01))):
        blockers.append('validation_pose_outside_local_training_tolerance')
    for metric in sections['motion_section']:
        mse = lambda key: float(np.mean(np.square(metric[key]['rmse_deg'])))
        if mse('model') >= min(mse('hold'), mse('velocity')):
            blockers.append('independent_motion_prediction_not_better_than_baselines')
    return dict(model_status='independent_validation_only_not_refitted', hardware_takeover_allowed=False,
                candidate_sha256=hashlib.sha256(artifact_bytes).hexdigest(),
                training_report_sha256=hashlib.sha256(training_bytes).hexdigest(),
                capture=info, configuration_difference=differences,
                fixed_configuration_matches=True, independent_of_training_sources=True,
                sections=sections, blockers=sorted(set(blockers)),
                caveats=['Candidate and training report were only read; no fitting performed.',
                         'Recorded future inputs used; no future output measurements inside prediction.',
                         'Windows overlap and all belong to one independent trial.',
                         'Historical training blockers retained, not interpreted as new test results.'])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('model_directory', type=Path)
    p.add_argument('capture', type=Path)
    p.add_argument('--allow-one-ms-jitter', action='store_true')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    result = validate(args.model_directory, args.capture, args.allow_one_ms_jitter)
    with args.output.open('x', encoding='utf-8') as f:
        json.dump(result, f, indent=2, allow_nan=False)
    print(json.dumps({k: result[k] for k in ('model_status', 'candidate_sha256', 'configuration_difference',
                                           'sections', 'blockers')}, indent=2))


if __name__ == '__main__':
    main()
