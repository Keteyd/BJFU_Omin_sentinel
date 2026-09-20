import unittest

import numpy as np

from audit_slow_pair import joint_delta, timing, integrate_rate
from identify_slow_offline import uniform_samples


class AuditTests(unittest.TestCase):
    def test_integrate_rate_uses_actual_intervals(self):
        t = np.array([0., .004, .009, .012])
        np.testing.assert_allclose(integrate_rate(t, [2., 2., 2., 2.]), 2*t)

    def test_encoder_wrap(self):
        np.testing.assert_allclose(joint_delta([8190, 8191, 0, 1]),
                                   np.arange(4) * 360 / 8192, atol=1e-10)

    def test_uniform_and_clock_wrap(self):
        ticks = (np.arange(5001) * 4 + 0xfffffff0) % (2**32)
        t, report = timing(ticks)
        self.assertEqual(t[-1], 20)
        self.assertEqual(report['interval_counts_ms'], {'4': 5000})

    def test_one_ms_shift_preserved(self):
        ticks = np.arange(5001) * 4
        ticks[802:-1] += 1
        _, report = timing(ticks)
        self.assertEqual(report['interval_counts_ms'], {'3': 1, '4': 4998, '5': 1})
        self.assertEqual(report['irregular_indices'], [802, 5000])
        self.assertTrue(report['exploratory_timing_acceptable'])

    def test_larger_jitter_not_accepted(self):
        ticks = np.arange(5001) * 4
        ticks[100] += 2
        self.assertFalse(timing(ticks)[1]['exploratory_timing_acceptable'])
        ticks[100] = ticks[99]
        with self.assertRaises(ValueError):
            timing(ticks)

    def test_resampling_holds_commands_without_terminal_output(self):
        t = np.arange(5001) * .004
        t[802:-1] += .001
        y = np.column_stack((t, 2*t, 3*t))
        u = np.column_stack((np.arange(5001), -np.arange(5001)))
        angles, commands = uniform_samples(t, y, u)
        self.assertEqual(angles.shape, (5000, 3))
        self.assertAlmostEqual(angles[802, 0], 3.208)
        self.assertEqual(commands[802, 0], 801)
        self.assertLess(commands[-1, 0], 5000)


if __name__ == '__main__':
    unittest.main()
