import unittest

import numpy as np

from identify_can_trace_offline import (baseline_centered_current_proxy, causal_resample,
                                        resample_integrated_input)


class CanTraceInputTests(unittest.TestCase):
    def test_causal_resample_never_uses_future_feedback(self):
        values, ages = causal_resample([100, 350, 350, 800], [1, 2, 3, 4],
                                       [200, 400, 700, 900])
        np.testing.assert_array_equal(values, [1, 3, 3, 4])
        np.testing.assert_array_equal(ages, [100, 50, 350, 100])
        with self.assertRaises(ValueError):
            causal_resample([100, 350], [1, 2], [50, 400])

    def test_current_proxy_uses_only_baseline_median_and_scales(self):
        raw = np.array([[100, -20], [110, -10], [90, -30], [210, 70]], dtype=float)
        proxy, baseline = baseline_centered_current_proxy(raw, baseline_samples=3)
        np.testing.assert_array_equal(baseline, [100, -20])
        np.testing.assert_allclose(proxy[-1], [.11, .09])

    def test_constant_irregular_intervals_preserve_uniform_mean(self):
        boundaries = np.array([1000, 4800, 9100, 13000, 17000, 21000])
        durations = np.diff(np.r_[0, boundaries])
        commands = np.array([250., -125.])
        areas = durations[:, None]*commands
        grid, means = resample_integrated_input(boundaries, areas, samples=5,
                                                dt_us=4000, start_us=1000)
        np.testing.assert_array_equal(grid, [1000, 5000, 9000, 13000, 17000])
        np.testing.assert_allclose(means, np.tile(commands/1000, (5, 1)))

    def test_step_area_is_split_across_uniform_boundary_without_loss(self):
        boundaries = np.array([1000, 5000, 7000, 11000, 15000])
        # Raw command is 0 through 5ms, then 1000 through 15ms.
        areas = np.array([[0, 0], [0, 0], [0, 0],
                          [4_000_000, -4_000_000], [4_000_000, -4_000_000]])
        _, means = resample_integrated_input(boundaries, areas, samples=4,
                                             dt_us=4000, start_us=1000)
        np.testing.assert_allclose(means[:3], [[0, 0], [.5, -.5], [1, -1]])
        self.assertAlmostEqual(float(means[:3, 0].sum()*.004), .006)

    def test_zero_origin_boundary_with_zero_initial_area_is_supported(self):
        boundaries = np.array([0, 4000, 8000, 12000])
        areas = np.array([[0, 0], [4_000_000, -2_000_000],
                          [4_000_000, -2_000_000], [4_000_000, -2_000_000]])
        grid, means = resample_integrated_input(boundaries, areas, samples=4)
        np.testing.assert_array_equal(grid, boundaries)
        np.testing.assert_allclose(means, [[1, -.5], [1, -.5],
                                           [1, -.5], [1, -.5]])

        areas[0, 0] = 1
        with self.assertRaises(ValueError):
            resample_integrated_input(boundaries, areas, samples=4)

    def test_invalid_or_out_of_range_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            resample_integrated_input([1000, 900], np.zeros((2, 2)), samples=1)
        with self.assertRaises(ValueError):
            resample_integrated_input([1000, 5000], np.zeros((2, 2)), samples=1)
        with self.assertRaises(ValueError):
            resample_integrated_input([1000, 5000], np.zeros((2, 2)), samples=3)


if __name__ == '__main__':
    unittest.main()
