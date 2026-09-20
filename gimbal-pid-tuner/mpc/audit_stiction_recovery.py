"""Reproducible synthetic audit of the existing stiction fitter; offline only.

The 1 ms reference uses a separate coordinate-descent friction solver. Even
successful recovery here does not validate the model family on the vehicle.
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from identify_coupled_offline import DT, TRAIN, TUNE, evaluate, lagged_input
from identify_stiction_offline import (ENCODER_RAD, StickSlip, endpoint_velocity,
                                      fit_stiction, interval_data, residual)

TRUTH = np.array([np.log(.7), .4, np.log(.6), np.log(.3),
                  np.log(.12), np.log(.08), np.log(.8), np.log(.5)])
SAMPLES = 5000


def reference_step(x, drive, mass, damping, friction, gain, dt):
    """Independent implicit friction solve by cyclic scalar soft thresholding."""
    a = mass + dt*np.diag(damping)
    rhs = mass @ x[2:] + dt*gain*drive
    velocity = x[2:].copy()
    for _ in range(200):
        previous = velocity.copy()
        for j in range(2):
            force = rhs[j] - a[j, 1-j]*velocity[1-j]
            velocity[j] = np.sign(force)*max(abs(force)-dt*friction[j], 0)/a[j, j]
        if np.max(np.abs(velocity-previous)) < 1e-13:
            return np.r_[x[:2]+dt*velocity, velocity]
    raise FloatingPointError('reference friction solver did not converge')


def excitation(samples, phase):
    t = np.arange(samples)*DT
    u = np.column_stack((1.4*np.sin(2*np.pi*.71*t+phase)+.6*np.sin(2*np.pi*1.83*t),
                         1.6*np.cos(2*np.pi*1.13*t+phase)+.5*np.sin(2*np.pi*2.37*t)))
    # Smooth startup and two nonzero-command holding intervals. These are
    # invented open-loop inputs, never a proposed hardware excitation.
    u *= np.clip((t-.8)/.4, 0, 1)[:, None]
    u[(t >= 5) & (t < 6)] = [.06, -.06]
    u[(t >= 12) & (t < 13)] = [-.06, .06]
    return u


def simulate(phase, tau, *, reference=False, samples=SAMPLES):
    model = StickSlip(TRUTH.copy(), tau)
    u = excitation(samples, phase)
    x = np.zeros((samples, 4))
    if reference:
        z = u[0].copy()
        dt = DT/4
        alpha = 1. if tau == 0 else -np.expm1(-dt/tau)
        for k in range(samples-1):
            state = x[k].copy()
            for _ in range(4):
                z += alpha*(u[k]-z)
                state = reference_step(state, z, model.mass, model.damping,
                                       model.friction, model.gain, dt)
            x[k+1] = state
    else:
        z = lagged_input(u, tau)
        for k in range(samples-1):
            x[k+1] = model.step(x[k], z[k])
    return dict(q=x[:, :2], v=x[:, 2:], u=u)


def observations(source, *, quantized=False, oracle_velocity=False):
    q = source['q'].copy()
    if quantized:
        q = np.rint(q/ENCODER_RAD)*ENCODER_RAD
    return dict(q=q, v=source['v'].copy() if oracle_velocity else endpoint_velocity(q),
                u=source['u'].copy(),
                y=np.rad2deg(np.column_stack((q, q.sum(axis=1)))))


def parameter_errors(model, truth):
    return {name: dict(truth=getattr(truth, attr).tolist(),
                       fitted=getattr(model, attr).tolist(),
                       max_relative_error=float(np.max(np.abs(
                           (getattr(model, attr)-getattr(truth, attr))/getattr(truth, attr)))))
            for name, attr in (('mass', 'mass'), ('damping', 'damping'),
                               ('friction', 'friction'), ('command_gain', 'gain'))}


def audit_case(name, sources, tau, *, quantized=False, oracle_velocity=False, method='interval'):
    if method == 'weak':
        from identify_weak_stiction_offline import fit_weak_stiction
        fitter = fit_weak_stiction
    else:
        fitter = fit_stiction
    trials = [observations(s, quantized=quantized, oracle_velocity=oracle_velocity) for s in sources]
    truth = StickSlip(TRUTH.copy(), tau)
    candidates, options = [], []
    for candidate_tau in (0., .02, .05):
        model, diagnostic = fitter(trials, candidate_tau)
        metrics = [evaluate(model, t, TRAIN, TUNE, 50) for t in trials]
        score = float(np.mean([np.mean(np.square(m['model']['rmse_deg'])) for m in metrics]))
        candidates.append(dict(tau_s=candidate_tau, tuning_score_deg2=score, fitting=diagnostic))
        options.append((score, model))
    _, selected = min(options, key=lambda item: item[0])
    if method == 'weak':
        from identify_weak_stiction_offline import weak_rows, weak_residual
        rows = weak_rows(trials, tau)
        scales = np.maximum(rows[:, :2].std(axis=0), .01)
        truth_residual = weak_residual(TRUTH, rows, scales)
    else:
        rows, stationary = interval_data(trials, tau)
        scales = np.maximum(rows[:, :2].std(axis=0), .01)
        truth_residual = residual(TRUTH, rows, stationary, scales)
    errors = parameter_errors(selected, truth)
    # Predeclared diagnostic tolerances, not vehicle acceptance thresholds.
    recovery_passed = (selected.tau == tau and
                       all(e['max_relative_error'] <= .20 for e in errors.values()))
    checks = [dict(rolling_200ms=evaluate(selected, t, TUNE, SAMPLES, 50),
                   free_run_2_to_20s=evaluate(selected, t, 500, SAMPLES, SAMPLES-500),
                   truth_free_run=evaluate(truth, t, 500, SAMPLES, SAMPLES-500)) for t in trials]
    return dict(name=name, method=method, hardware_takeover_allowed=False,
                quantized_encoder=quantized, oracle_velocity=oracle_velocity,
                true_tau_s=tau, selected_tau_s=selected.tau,
                parameter_recovery_passed=recovery_passed,
                parameter_errors=errors, candidates=candidates,
                truth_normalized_residual_rms=float(np.sqrt(np.mean(truth_residual**2))),
                checks=checks)


def analyze(method='interval'):
    ideal = [simulate(p, 0.) for p in (0., .83)]
    reference = [simulate(p, .02, reference=True) for p in (0., .83)]
    cases = []
    for name, sources, tau, quantized, oracle in (
            ('shared_4ms_integrator_exact_velocity', ideal, 0., False, True),
            ('shared_4ms_integrator_endpoint_velocity', ideal, 0., False, False),
            ('independent_1ms_integrator_lag20ms', reference, .02, False, False),
            ('independent_1ms_integrator_lag20ms_encoder8192', reference, .02, True, False)):
        print(f'Auditing {method}: {name}', flush=True)
        cases.append(audit_case(name, sources, tau, quantized=quantized, oracle_velocity=oracle, method=method))
    directory = Path(__file__).resolve().parent
    hashes = {name: hashlib.sha256((directory/name).read_bytes()).hexdigest()
              for name in ('audit_stiction_recovery.py', 'identify_stiction_offline.py',
                           'identify_coupled_offline.py', 'identify_weak_stiction_offline.py')}
    return dict(model_status='synthetic_audit_only', hardware_takeover_allowed=False,
                all_parameter_recovery_passed=all(c['parameter_recovery_passed'] for c in cases),
                protocol=dict(method=method, samples=SAMPLES, sample_period_s=DT, phases=[0., .83],
                              split_samples=dict(train=[0, TRAIN], select=[TRAIN, TUNE], check=[TUNE, SAMPLES]),
                              candidate_tau_s=[0., .02, .05], max_parameter_relative_error=.20,
                              exact_tau_selection_required=True, source_sha256=hashes),
                truth=StickSlip(TRUTH.copy(), .02).artifact(), cases=cases,
                limitations=['Invented open-loop two-input excitation, not historical vehicle data or a hardware trial.',
                             'First two cases share the fitted model integrator and are only internal consistency checks.',
                             'Independent solver still uses the same assumed Coulomb model family.',
                             '20 ms actuator first-order lag is not a pure transport delay; transport delay is not tested.',
                             'No closed-loop feedback bias, missing input updates, measurement age or external loads simulated.',
                             'Quantized synthetic heading is the sum of encoders, not a simulated independent IMU.',
                             'Long-run checks use recorded future commands and may include fitted time segments.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--method', choices=('interval', 'weak'), default='interval')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output must not exist')
    result = analyze(args.method)
    with args.output.open('x', encoding='utf-8') as stream:
        stream.write(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(args.output),
                          cases=[dict(name=c['name'], passed=c['parameter_recovery_passed'],
                                      selected_tau_s=c['selected_tau_s'],
                                      relative_errors={k: v['max_relative_error'] for k, v in c['parameter_errors'].items()})
                                 for c in result['cases']]), indent=2))
    return 0 if result['all_parameter_recovery_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
