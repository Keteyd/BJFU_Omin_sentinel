import unittest
from unittest.mock import patch

import numpy as np
from scipy.integrate import quad

from audit_stiction_recovery import TRUTH, simulate, observations, parameter_errors
from identify_coupled_offline import DT, TRAIN
from identify_stiction_offline import StickSlip
from identify_weak_stiction_offline import weak_residual
from identify_timed_stiction_offline import (lag_coefficients, actuator_history, triangular_input,
                                            timed_rows, TimedStickSlip, fit_timed_stiction, evaluate_timed)
from audit_stiction_timing import reference_source


class TimedStictionTests(unittest.TestCase):
    def test_refined_reference_preserves_previous_one_ms_generator(self):
        previous = simulate(.47, .02, reference=True, samples=500)
        current = reference_source(.47, .02, 4, samples=500)
        for name in ('q', 'v', 'u'):
            np.testing.assert_array_equal(current[name], previous[name])

    def test_lag_nodes_and_integrals_match_analytic_step(self):
        u = np.zeros((30, 2))
        u[10:] = 1.
        nodes, mean, falling = actuator_history(u, .02)
        np.testing.assert_array_equal(nodes[:11], 0.)
        for k in range(10, len(u)):
            t = (k-10)*DT
            self.assertAlmostEqual(nodes[k, 0], 1-np.exp(-t/.02), places=13)
            expected_mean = quad(lambda s: 1-np.exp(-(t+s)/.02), 0, DT)[0]/DT
            expected_fall = quad(lambda s: (DT-s)*(1-np.exp(-(t+s)/.02)), 0, DT)[0]/DT**2
            self.assertAlmostEqual(mean[k, 0], expected_mean, places=13)
            self.assertAlmostEqual(falling[k, 0], expected_fall, places=13)

    def test_triangle_matches_continuous_quadrature(self):
        u = np.zeros((30, 2))
        u[10:] = 1.
        value = triangular_input(u, .02)
        for k in (9, 10, 11, 19):
            t = (k-10)*DT
            def integrand(s):
                z = 0. if t+s < 0 else 1-np.exp(-(t+s)/.02)
                return (DT-abs(s))*z/DT**2
            expected = quad(integrand, -DT, 0)[0]+quad(integrand, 0, DT)[0]
            self.assertAlmostEqual(value[k, 0], expected, places=12)

    def test_zero_lag_triangle_averages_neighboring_commands(self):
        u = np.arange(60, dtype=float).reshape(30, 2)
        np.testing.assert_allclose(triangular_input(u, 0.)[1:], (u[:-1]+u[1:])/2)
        np.testing.assert_allclose(triangular_input(u, 0.)[0], u[0])

    def test_lag_history_is_causal(self):
        u = np.zeros((30, 2))
        before = actuator_history(u, .02)
        u[20:] = 100.
        after = actuator_history(u, .02)
        np.testing.assert_array_equal(before[0][:21], after[0][:21])
        np.testing.assert_array_equal(before[1][:20], after[1][:20])
        np.testing.assert_array_equal(triangular_input(u, .02)[:20], 0.)

    def test_extreme_lag_and_invalid_inputs(self):
        decay, mean, falling = lag_coefficients(1e9, DT)
        self.assertAlmostEqual(decay, 1.)
        self.assertAlmostEqual(mean, 1.)
        self.assertAlmostEqual(falling, .5)
        for tau in (-1., np.nan, np.inf):
            with self.assertRaises(ValueError):
                actuator_history(np.zeros((20, 2)), tau)
        for u in (np.zeros((0, 2)), np.zeros((20, 3)), np.full((20, 2), np.nan)):
            with self.assertRaises(ValueError):
                actuator_history(u, .02)

    def test_centered_momentum_matches_analytic_accelerating_motion(self):
        model = StickSlip(TRUTH, 0.)
        t = np.arange(TRAIN)[:, None]*DT
        a, v0 = np.array([.2, .1]), np.array([.5, .7])
        q = .5*t*t*a+t*v0
        u = (model.mass @ a + model.damping*(v0+(t+DT/2)*a)+model.friction)/model.gain
        rows = timed_rows([dict(q=q, u=u)], 0.)
        np.testing.assert_allclose(weak_residual(TRUTH, rows, np.ones(2)), 0., atol=1e-10)

    def test_quantized_independent_reference_recovery(self):
        trials = [observations(simulate(p, .02, reference=True, samples=TRAIN), quantized=True)
                  for p in (1.71, 2.4)]
        fitted, diagnostic = fit_timed_stiction(trials, .02)
        errors = parameter_errors(fitted, StickSlip(TRUTH, .02))
        self.assertLess(max(e['max_relative_error'] for e in errors.values()), .20)
        self.assertFalse(diagnostic['at_bound_names'])
        self.assertFalse(fitted.artifact()['hardware_takeover_allowed'])

    def test_training_stays_before_split_and_ignores_velocity_estimates(self):
        source = observations(simulate(.47, .02, samples=TRAIN+100), quantized=True)
        expected = timed_rows([source], .02)
        source['q'][TRAIN:] = np.nan
        source['u'][TRAIN:] = np.nan
        source['v'][:] = np.nan
        np.testing.assert_array_equal(timed_rows([source], .02), expected)
        source['q'][:TRAIN] += [8., -9.]
        np.testing.assert_allclose(timed_rows([source], .02), expected, atol=1e-10)

    def test_future_measurements_cannot_change_predictions(self):
        trial = observations(simulate(.47, .02, samples=100))
        model = TimedStickSlip(TRUTH, .02)
        original = model.step
        predictions = []
        def record(x, u):
            value = original(x, u)
            predictions.append(value.copy())
            return value
        with patch.object(model, 'step', side_effect=record):
            evaluate_timed(model, trial, 20, 40, 20)
            expected = np.array(predictions)
            predictions.clear()
            for name in ('q', 'v', 'y'):
                trial[name][20:] += 100.
            evaluate_timed(model, trial, 20, 40, 20)
        np.testing.assert_array_equal(predictions, expected)

    def test_batched_six_state_step_and_energy_dissipation(self):
        model = TimedStickSlip(TRUTH, .02)
        x = np.array([0., 0., .5, -.7, 0., 0.])
        batched = model.step(np.tile(x, (3, 1)), [0., 0.])
        np.testing.assert_allclose(batched, np.tile(model.step(x, [0., 0.]), (3, 1)))
        for _ in range(100):
            before = x[2:4] @ model.mass @ x[2:4]
            x = model.step(x, [0., 0.])
            self.assertLessEqual(x[2:4] @ model.mass @ x[2:4], before+1e-12)
        self.assertEqual(len(model.artifact()['state_order']), 6)
        with self.assertRaises(ValueError):
            model.step(np.zeros(4), [0., 0.])


if __name__ == '__main__':
    unittest.main()
