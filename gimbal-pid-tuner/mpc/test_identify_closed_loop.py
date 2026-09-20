import unittest

import numpy as np

from identify_closed_loop_offline import DT, SAMPLES, electrical_fit, first_order_signal


class ClosedLoopIdentificationTests(unittest.TestCase):
    def test_first_order_signal_is_causal(self):
        u = np.zeros(8); u[3:] = 1
        z = first_order_signal(u, delay_samples=1, tau_s=DT)
        self.assertEqual(z[3], 0.)
        self.assertGreater(z[4], 0.)
        altered = u.copy(); altered[-1] = 99
        np.testing.assert_allclose(first_order_signal(altered, 1, DT)[:-1], z[:-1])

    def test_electrical_fit_recovers_synthetic_delay_tau_gain_and_bias(self):
        k = np.arange(SAMPLES)
        trials = []
        benches = []
        for trial_axis, phase in ((1, 0.), (2, .7)):
            u = np.column_stack((np.sin(k*.013+phase), np.cos(k*.011+phase)))
            current = np.column_stack((
                1.7*first_order_signal(u[:, 0]*1000, 1, .008)+123+4,
                .8*first_order_signal(u[:, 1]*1000, 1, .008)-77-6))
            trials.append({'u': u, 'current_raw': current, 'info': {'axis': trial_axis}})
            benches.append({'axes': {'big': {'median_raw': 123., 'std_raw': 1.},
                                      'small': {'median_raw': -77., 'std_raw': 1.}}})
        fitted = electrical_fit(trials, benches)
        for axis, gain, residual in (('big', 1.7, 4.), ('small', .8, -6.)):
            self.assertEqual(fitted[axis]['delay_samples'], 1)
            self.assertEqual(fitted[axis]['tau_ms'], 8.)
            self.assertAlmostEqual(fitted[axis]['gain_current_raw_per_command_raw'], gain, places=9)
            self.assertAlmostEqual(fitted[axis]['residual_bias_raw_after_bench_subtraction'], residual, places=8)


if __name__ == '__main__':
    unittest.main()
