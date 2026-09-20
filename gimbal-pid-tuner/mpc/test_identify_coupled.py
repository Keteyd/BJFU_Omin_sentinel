import unittest

import numpy as np

from identify_coupled_offline import Coupled, DT, causal_velocity, evaluate, lagged_input, parameters, velocity_residual, fit_model, TRAIN


class CoupledTests(unittest.TestCase):
    def test_recover_known_coupled_synthetic_plant(self):
        p = np.array([np.log(.7), .4, np.log(.6), np.log(.3),
                      np.log(.03), np.log(.02), np.log(.8), np.log(.5)])
        truth = Coupled(p, 0)
        t = np.arange(TRAIN) * DT
        u = np.column_stack((np.sin(2*np.pi*.8*t)+.3*np.sin(2*np.pi*1.7*t),
                             np.cos(2*np.pi*1.2*t)+.4*np.sin(2*np.pi*2.1*t)))
        x = np.zeros((TRAIN, 4))
        for k in range(TRAIN-1):
            x[k+1] = truth.step(x[k], u[k])
        fitted, diagnostic = fit_model([dict(q=x[:, :2], v=x[:, 2:], u=u)], 0)
        np.testing.assert_allclose(fitted.mass, truth.mass, rtol=.005, atol=.001)
        np.testing.assert_allclose(fitted.gain, truth.gain, rtol=.005, atol=.001)
        self.assertFalse(diagnostic['at_bound'])

    def test_mass_shrink_cannot_hide_unexplained_velocity(self):
        rows = np.array([[1., 2., 0., 0., 0., 0., 0., 0.]])
        for ratio in (-4., 0., 4.):
            p = np.array([ratio, .3, 0., 0., -4., -4., 0., 0.])
            np.testing.assert_allclose(velocity_residual(p, rows, np.ones(2)), [1., 2.])

    def test_positive_definite_mass_and_fixed_angle_scale(self):
        for ratio in (-4., 0., 4.):
            for rho in (-3., 0., 3.):
                p = np.array([ratio, rho, 0., 0., -4., -4., 0., 0.])
                self.assertTrue(np.all(np.linalg.eigvalsh(parameters(p)[0]) > 0))
                self.assertEqual(Coupled(p, 0).artifact()['angle_scales'], [1., 1.])

    def test_velocity_is_causal_and_correct_for_ramp(self):
        q = np.column_stack((np.arange(100)*DT, -2*np.arange(100)*DT))
        expected = causal_velocity(q)
        np.testing.assert_allclose(expected[10:], np.tile([1., -2.], (90, 1)), atol=1e-12)
        q[70:] += 1000
        np.testing.assert_allclose(causal_velocity(q)[10:70], expected[10:70])

    def test_lag_is_causal(self):
        u = np.zeros((100, 2))
        u[50:] = 1
        z = lagged_input(u, .02)
        np.testing.assert_equal(z[:50], 0)
        self.assertTrue(np.all((z[50:] > 0) & (z[50:] < 1)))

    def test_no_future_output_leakage(self):
        q = np.zeros((100, 2))
        trial = dict(q=q, v=causal_velocity(q), y=np.zeros((100, 3)), u=np.zeros((100, 2)))
        model = Coupled(np.array([0., .3, 0., 0., -4., -4., 0., 0.]), 0)
        result = evaluate(model, trial, 20, 45, 25)
        np.testing.assert_allclose(result['model']['rmse_deg'], 0)
        trial['y'][20:45] = 10
        result = evaluate(model, trial, 20, 45, 25)
        np.testing.assert_allclose(result['model']['rmse_deg'], 10)

    def test_zero_input_damping_reduces_energy(self):
        model = Coupled(np.array([0., .3, 0., 0., -8., -8., 0., 0.]), 0)
        x = np.array([[0., 0., 1., -.5]])
        e0 = x[0, 2:] @ model.mass @ x[0, 2:]
        for _ in range(100):
            x = model.step(x, np.zeros((1, 2)))
        self.assertLess(x[0, 2:] @ model.mass @ x[0, 2:], e0)


if __name__ == '__main__':
    unittest.main()
