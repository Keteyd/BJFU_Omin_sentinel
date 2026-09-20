import unittest

import numpy as np

from mpc.develop_f4_f5_plant_offline import (
    ACTIVE, causal_velocity, evaluate_state_segment, fit_state_candidate,
    indices, state_scales, with_baseline_centered_current_input)


class OperatingDomainPlantHelpersTest(unittest.TestCase):
    def test_causal_velocity_does_not_use_future_samples(self):
        q = np.zeros((20, 2))
        changed = q.copy(); changed[15:] = 100
        before = causal_velocity(q)
        after = causal_velocity(changed)
        np.testing.assert_allclose(before[:15], after[:15])

    def test_current_input_uses_only_baseline_median(self):
        time = np.arange(1800)*.02
        current = np.column_stack((np.full(1800, 12.), np.full(1800, -8.)))
        current[200:] += [5., -3.]
        data = {'time': time, 'current': current, 'u': np.zeros((1800, 2))}
        converted = with_baseline_centered_current_input(data)
        np.testing.assert_allclose(converted['u'][:150], 0)
        np.testing.assert_allclose(converted['u'][200], [.005, -.003])

    def test_fixed_kinematics_model_recovers_stable_synthetic_dynamics(self):
        rng = np.random.default_rng(12)
        samples = 1800
        time = np.arange(samples)*.02
        u = rng.normal(size=(samples, 2))
        q = np.zeros((samples, 2)); v = np.zeros_like(q)
        a = np.array([[.82, .04], [.03, .78]])
        b = np.array([[.45, .08], [.05, .38]])
        for k in range(4, samples):
            v[k] = a@v[k-1]+b@u[k]
            q[k] = q[k-1]+.02*v[k]
        data = {
            'time': time, 'q': q, 'u': u, 'reference': u,
            'current': u, 'heading': q.sum(axis=1),
        }
        scales = state_scales(data)
        # Four input taps are needed because the observed velocity is a
        # three-interval causal average of the integrated synthetic velocity.
        model = fit_state_candidate(data, 'ols', 1, 4, 0, 1e-8,
                                    False, False, scales)
        result = evaluate_state_segment(data, model, scales, ACTIVE)
        self.assertFalse(result['diverged'])
        self.assertLess(max(result['model']['rmse_deg']), .03)
        self.assertGreater(min(result['improvement_over_hold_fraction']), .9)

    def test_time_segments_are_half_open(self):
        data = {'time': np.arange(1800)*.02}
        pick = indices(data, (3., 18.))
        self.assertEqual((pick[0], pick[-1], len(pick)), (150, 899, 750))


if __name__ == '__main__':
    unittest.main()
