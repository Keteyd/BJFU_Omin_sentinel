import unittest
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np

from audit_stiction_recovery import TRUTH, observations, reference_step, simulate, parameter_errors
from identify_coupled_offline import DT, TRAIN
from identify_stiction_offline import ENCODER_RAD, StickSlip
from identify_weak_stiction_offline import weak_rows, weak_residual, fit_weak_stiction


class WeakStictionTests(unittest.TestCase):
    def test_independent_reference_matches_diagonal_analytic_solution(self):
        mass = np.diag([1., .7])
        d, f, g = np.array([.6, .3]), np.array([.12, .08]), np.array([.8, .5])
        x, u = np.array([.1, -.2, .4, -.1]), np.array([1.1, -.8])
        rhs = mass @ x[2:]+DT*g*u
        v = np.sign(rhs)*np.maximum(np.abs(rhs)-DT*f, 0)/np.diag(mass+DT*np.diag(d))
        np.testing.assert_allclose(reference_step(x, u, mass, d, f, g, DT), np.r_[x[:2]+DT*v, v])

    def test_independent_reference_coupled_friction_balance(self):
        model = StickSlip(TRUTH, 0.)
        rng = np.random.default_rng(128)
        for x, u in zip(rng.normal(size=(20, 4)), rng.normal(size=(20, 2))):
            nxt = reference_step(x, u, model.mass, model.damping, model.friction, model.gain, .001)
            reaction = model.mass @ (x[2:]-nxt[2:])/.001 + model.gain*u - model.damping*nxt[2:]
            for j in range(2):
                if abs(nxt[j+2]) > 1e-10:
                    self.assertAlmostEqual(reaction[j], model.friction[j]*np.sign(nxt[j+2]), places=8)
                else:
                    self.assertLessEqual(abs(reaction[j]), model.friction[j]+1e-8)

    def test_quantization_is_bounded_and_does_not_mutate_truth(self):
        source = simulate(.47, 0., samples=250)
        original = source['q'].copy()
        trial = observations(source, quantized=True)
        self.assertLessEqual(np.abs(trial['q']-original).max(), ENCODER_RAD/2+1e-15)
        trial['u'][:] = 42
        trial['q'][:] = 42
        np.testing.assert_array_equal(source['q'], original)
        self.assertFalse(np.all(source['u'] == 42))

    def test_weak_moments_equal_discrete_momentum_for_constant_positive_motion(self):
        model = StickSlip(TRUTH, 0.)
        velocity = np.array([.5, .7])
        q = np.arange(TRAIN)[:, None]*DT*velocity
        u = np.tile((model.damping*velocity+model.friction)/model.gain, (TRAIN, 1))
        rows = weak_rows([dict(q=q, u=u)], 0.)
        np.testing.assert_allclose(weak_residual(TRUTH, rows, np.ones(2)), 0., atol=1e-10)

    def test_fit_features_ignore_future_and_velocity_estimates(self):
        source = simulate(.47, .02, samples=TRAIN+200)
        trial = observations(source, quantized=True)
        expected = weak_rows([trial], .02)
        trial['q'][TRAIN:] = np.nan
        trial['u'][TRAIN:] = np.nan
        trial['v'][:] = np.nan
        np.testing.assert_array_equal(weak_rows([trial], .02), expected)
        trial['q'][:TRAIN] += [8., -9.]
        np.testing.assert_allclose(weak_rows([trial], .02), expected, atol=1e-10)

    def test_stationary_commands_are_friction_intervals(self):
        source = dict(q=np.zeros((TRAIN, 2)), u=np.tile([.06, -.06], (TRAIN, 1)))
        rows = weak_rows([source], 0.)
        np.testing.assert_allclose(weak_residual(TRUTH, rows, np.ones(2)), 0.)
        source['u'][:] = [2., -2.]
        self.assertGreater(np.linalg.norm(weak_residual(TRUTH, weak_rows([source], 0.), np.ones(2))), 0.)

    def test_unquantized_recovery_on_separate_phase(self):
        source = observations(simulate(1.71, 0., samples=TRAIN))
        fitted, diagnostic = fit_weak_stiction([source], 0.)
        errors = parameter_errors(fitted, StickSlip(TRUTH, 0.))
        self.assertLess(max(e['max_relative_error'] for e in errors.values()), .20)
        self.assertFalse(diagnostic['at_bound_names'])
        self.assertFalse(fitted.artifact()['hardware_takeover_allowed'])

    def test_unexcited_data_reports_rank_loss_and_never_authorizes_hardware(self):
        source = dict(q=np.zeros((TRAIN, 2)), u=np.zeros((TRAIN, 2)))
        fitted, diagnostic = fit_weak_stiction([source], 0.)
        self.assertEqual(diagnostic['jacobian_rank'], 0)
        self.assertIsNone(diagnostic['jacobian_condition'])
        self.assertFalse(fitted.artifact()['hardware_takeover_allowed'])

    def test_invalid_training_data_is_rejected(self):
        for length, value in ((TRAIN-1, 0.), (TRAIN, np.nan), (TRAIN, np.inf)):
            with self.assertRaises(ValueError):
                weak_rows([dict(q=np.full((length, 2), value), u=np.zeros((length, 2)))], 0.)

    def test_existing_evidence_cannot_be_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'report.json'
            output.write_text('preserved', encoding='utf-8')
            script = Path(__file__).with_name('audit_stiction_recovery.py')
            result = subprocess.run([sys.executable, str(script), '--output', str(output)],
                                    capture_output=True, text=True, timeout=20)
            self.assertEqual(result.returncode, 2)
            self.assertIn('output must not exist', result.stderr)
            self.assertEqual(output.read_text(encoding='utf-8'), 'preserved')


if __name__ == '__main__':
    unittest.main()
