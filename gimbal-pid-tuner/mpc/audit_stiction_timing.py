"""Factorial timing diagnosis plus frozen synthetic validation. No hardware I/O."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from audit_stiction_recovery import TRUTH, SAMPLES, excitation, reference_step, simulate, observations, parameter_errors
from identify_coupled_offline import DT, TRAIN
from identify_stiction_offline import StickSlip
from identify_weak_stiction_offline import fit_weak_stiction
from identify_timed_stiction_offline import fit_timed_stiction, select_timed, evaluate_timed


def reference_source(phase, tau, substeps, samples=SAMPLES):
    """Refine the previous independent reference without changing its rules."""
    if not isinstance(substeps, int) or substeps < 1:
        raise ValueError('positive integer reference substep count required')
    truth = StickSlip(TRUTH.copy(), tau)
    u = excitation(samples, phase)
    x = np.zeros((samples, 4))
    z = u[0].copy()
    dt = DT/substeps
    alpha = 1. if tau == 0 else -np.expm1(-dt/tau)
    for k in range(samples-1):
        state = x[k].copy()
        for _ in range(substeps):
            z += alpha*(u[k]-z)
            state = reference_step(state, z, truth.mass, truth.damping, truth.friction, truth.gain, dt)
        x[k+1] = state
    return dict(q=x[:, :2], v=x[:, 2:], u=u)


def digest(source):
    h = hashlib.sha256()
    for name in ('q', 'v', 'u'):
        h.update(name.encode('ascii'))
        h.update(np.asarray(source[name], dtype='<f8').tobytes())
    return h.hexdigest()


def summarize(model, diagnostic, tau):
    errors = parameter_errors(model, StickSlip(TRUTH, tau))
    return dict(parameter_errors=errors, fitting=diagnostic,
                max_relative_error=max(e['max_relative_error'] for e in errors.values()),
                parameter_recovery_passed=all(e['max_relative_error'] <= .20 for e in errors.values()))


def analyze():
    attribution = []
    for generator in ('legacy_shared_4ms', 'independent_1ms', 'independent_0.25ms'):
        for tau in (0., .02):
            print(f'Attribution: {generator}, tau={tau}', flush=True)
            sources = [simulate(p, tau, samples=TRAIN) if generator == 'legacy_shared_4ms'
                       else reference_source(p, tau, 4 if generator == 'independent_1ms' else 16, TRAIN)
                       for p in (0., .83)]
            for quantized in (False, True):
                trials = [observations(s, quantized=quantized) for s in sources]
                results = {}
                for name, fitter in (('previous_weak', fit_weak_stiction), ('aligned', fit_timed_stiction)):
                    model, diagnostic = fitter(trials, tau)
                    results[name] = summarize(model, diagnostic, tau)
                attribution.append(dict(generator=generator, known_tau_s=tau, quantized=quantized,
                                        source_sha256=[digest(s) for s in sources], results=results))

    # Model selection sees only the original two synthetic development phases.
    # Generate the separate validation source only after freezing selection.
    sources = [reference_source(p, .02, 4) for p in (0., .83)]
    training = [observations(s, quantized=True) for s in sources]
    print('Selecting 0/20/50ms using development records only', flush=True)
    model, diagnostic, candidates = select_timed(training)
    frozen = json.dumps(model.artifact(), sort_keys=True, allow_nan=False)
    frozen_hash = hashlib.sha256(frozen.encode()).hexdigest()
    errors = summarize(model, diagnostic, .02)

    validation_source = reference_source(3.07, .02, 16)
    source_hash = digest(validation_source)
    if source_hash in [digest(s) for s in sources]:
        raise ValueError('validation source duplicates training')
    observed = observations(validation_source, quantized=True)
    oracle = observations(validation_source, quantized=True, oracle_velocity=True)
    checks = {}
    for name, trial in (('causal_measured_velocity', observed), ('oracle_velocity_diagnostic', oracle)):
        checks[name] = dict(rolling_200ms=evaluate_timed(model, trial, 500, SAMPLES, 50),
                           free_run=evaluate_timed(model, trial, 500, SAMPLES, SAMPLES-500))
    if json.dumps(model.artifact(), sort_keys=True, allow_nan=False) != frozen:
        raise ValueError('frozen model changed during validation')
    # Fixed diagnostic gate: every predicted joint and heading must beat a hold
    # baseline on this synthetic trajectory. This is not a vehicle requirement.
    free = checks['causal_measured_velocity']['free_run']
    prediction_passed = bool(np.all(np.array(free['model']['rmse_deg']) < free['hold']['rmse_deg']))
    recovery_passed = model.tau == .02 and errors['parameter_recovery_passed']
    directory = Path(__file__).resolve().parent
    names = ('audit_stiction_timing.py', 'identify_timed_stiction_offline.py', 'audit_stiction_recovery.py',
             'identify_stiction_offline.py', 'identify_weak_stiction_offline.py', 'identify_coupled_offline.py')
    return dict(model_status='synthetic_timing_audit_only', hardware_takeover_allowed=False,
                synthetic_gate_passed=recovery_passed and prediction_passed,
                protocol=dict(max_parameter_relative_error=.20, tau_candidates_s=[0., .02, .05],
                              true_tau_s=.02, training_phases=[0., .83], validation_phase=3.07,
                              train_samples=[0, TRAIN], select_samples=[TRAIN, 3500],
                              prediction_gate='all joint/heading free-run RMSE below hold baseline',
                              source_sha256={name: hashlib.sha256((directory/name).read_bytes()).hexdigest() for name in names}),
                attribution=attribution, selected=model.artifact(), candidates=candidates,
                recovery=errors, correct_tau_selected=model.tau == .02,
                frozen_validation=dict(artifact_sha256=frozen_hash, artifact_unchanged=True,
                                       training_source_sha256=[digest(s) for s in sources],
                                       validation_source_sha256=source_hash, checks=checks,
                                       prediction_gate_passed=prediction_passed),
                limitations=['Attribution fits use known tau to isolate factors; only the final selection estimates it.',
                             'Legacy shared 4ms generator is a retrospective self-consistency check, not physical truth.',
                             'Independent reference retains previous right-endpoint force and position quadrature; 0.25ms checks refinement.',
                             'One new phase and finer step only; same assumed friction model and invented open-loop inputs.',
                             'Validation phase 3.07 is excluded from fitting and candidate selection.',
                             'Pure delays, feedback bias, external loads and missing within-4ms inputs remain untested.',
                             'Steady initial actuator state and quantized encoder-sum heading are synthetic assumptions.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output must not exist')
    report = analyze()
    with args.output.open('x', encoding='utf-8') as stream:
        stream.write(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(args.output), synthetic_gate_passed=report['synthetic_gate_passed'],
                          selected_tau_s=report['selected']['actuator_tau_s'],
                          recovery=report['recovery'], frozen_validation=report['frozen_validation']), indent=2))
    return 0 if report['synthetic_gate_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
