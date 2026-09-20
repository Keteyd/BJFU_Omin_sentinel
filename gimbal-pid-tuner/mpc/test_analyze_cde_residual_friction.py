import unittest

import numpy as np

from analyze_cde_residual_friction import features, smooth_velocity


class ResidualFeatureCausalityTests(unittest.TestCase):
    def test_velocity_does_not_use_future_measurements(self):
        baseline = np.zeros((30, 3))
        changed = baseline.copy()
        changed[20, 0] = 1.0

        baseline_velocity = smooth_velocity(baseline)
        changed_velocity = smooth_velocity(changed)

        np.testing.assert_array_equal(changed_velocity[:20], baseline_velocity[:20])
        self.assertNotEqual(changed_velocity[20, 0], baseline_velocity[20, 0])

    def test_friction_features_lag_measured_state_by_one_sample(self):
        baseline = {
            'reference': np.zeros((30, 2)),
            'measured': np.zeros((30, 3)),
            'command': np.zeros((30, 2)),
            'current': np.zeros((30, 2)),
        }
        changed = {name: value.copy() for name, value in baseline.items()}
        changed['measured'][20, 0] = 1.0

        baseline_features = features(baseline, 'friction_state')
        changed_features = features(changed, 'friction_state')

        np.testing.assert_array_equal(changed_features[:21], baseline_features[:21])
        self.assertTrue(np.any(changed_features[21] != baseline_features[21]))


if __name__ == '__main__':
    unittest.main()
