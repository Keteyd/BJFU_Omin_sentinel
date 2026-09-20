import unittest

import numpy as np

from mpc.analyze_cable_free_scaling import (direction_stats, metrics,
                                             safe_corr, target_outputs)


class CableFreeScalingHelpersTest(unittest.TestCase):
    def test_heading_reference_maps_to_planar_small_joint(self):
        reference = np.array([[1., 3.], [-2., 1.]])
        np.testing.assert_allclose(
            target_outputs(reference), [[1., 2., 3.], [-2., 3., 1.]])

    def test_direction_proxy_reports_gap_and_midpoint(self):
        result = direction_stats(
            np.array([10., 14., -2., -6., 99.]),
            np.array([1., 2., -1., -2., 0.]), threshold=.2)
        self.assertEqual(result['positive_samples'], 2)
        self.assertEqual(result['negative_samples'], 2)
        self.assertAlmostEqual(result['half_direction_gap'], 8.)
        self.assertAlmostEqual(result['direction_midpoint'], 4.)

    def test_metrics_use_prediction_minus_measurement(self):
        result = metrics(np.array([[2., 0.], [4., 2.]]),
                         np.array([[1., 1.], [2., 1.]]))
        np.testing.assert_allclose(result['mean_error_deg'], [1.5, 0.])
        np.testing.assert_allclose(result['rmse_deg'], [np.sqrt(2.5), 1.])

    def test_constant_signal_correlation_is_undefined(self):
        self.assertIsNone(safe_corr(np.ones(5), np.arange(5)))


if __name__ == '__main__':
    unittest.main()
