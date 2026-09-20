import unittest

import numpy as np

from identify_coupled_offline import DT, causal_velocity
from identify_stiction_offline import StickSlip, endpoint_velocity, residual, free_run_failures


class StictionTests(unittest.TestCase):
    def test_free_run_failure_is_explicit(self):
        validation = [dict(axis=i, free_run=dict(model=dict(rmse_deg=[0, 0, error]),
                                               hold=dict(rmse_deg=[0, 0, 1])))
                      for i, error in enumerate((.5, 1., 2.), 1)]
        self.assertEqual(free_run_failures(validation), [2, 3])

    def model(self):
        return StickSlip(np.array([0., .3, 0., 0., 0., 0., 0., 0.]), 0)

    def test_endpoint_velocity_removes_quadratic_time_shift_without_future(self):
        t = np.arange(100)*DT
        q = np.column_stack((t*t, -t*t))
        old, new = causal_velocity(q), endpoint_velocity(q)
        np.testing.assert_allclose(new[10:, 0], 2*t[10:], atol=1e-12)
        np.testing.assert_allclose(new[10:, 0]-old[10:, 0], .04, atol=1e-12)
        q[80:] += 100
        np.testing.assert_allclose(endpoint_velocity(q)[10:80], new[10:80])

    def test_rest_with_nonzero_command_and_breakaway(self):
        m = self.model()
        rest = np.zeros(4)
        np.testing.assert_allclose(m.step(rest, [.8, -.8]), rest, atol=1e-14)
        moving = m.step(rest, [2., 0.])
        self.assertGreater(moving[2], 0)
        self.assertEqual(moving[3], 0)

    def test_coupled_friction_equilibrium(self):
        m = self.model()
        rng = np.random.default_rng(19)
        x, u = rng.normal(size=(50, 4)), rng.normal(size=(50, 2))
        nxt = m.step(x, u)
        reaction = (x[:, 2:]-nxt[:, 2:]) @ m.mass.T/DT + u*m.gain-nxt[:, 2:]*m.damping
        moving = np.abs(nxt[:, 2:]) > 1e-9
        np.testing.assert_allclose(reaction[moving], np.broadcast_to(m.friction, reaction.shape)[moving]
                                   *np.sign(nxt[:, 2:][moving]), atol=1e-10)
        self.assertTrue(np.all(np.abs(reaction[~moving]) <= 1+1e-10))

    def test_no_input_energy_dissipates(self):
        m = self.model()
        x = np.array([0., 0., 1., -.5])
        for _ in range(400):
            before = x[2:] @ m.mass @ x[2:]
            x = m.step(x, [0., 0.])
            self.assertLessEqual(x[2:] @ m.mass @ x[2:], before+1e-12)
        np.testing.assert_allclose(x[2:], 0, atol=1e-12)

    def test_stationary_rows_use_inequality_not_zero_friction(self):
        m = self.model()
        rows = np.array([[0., 0., 0., 0., 0., 0., .08, -.08]])
        np.testing.assert_allclose(residual(m.p, rows, np.ones((1, 2), dtype=bool), np.ones(2)), 0)
        rows[0, 6] = .2
        self.assertGreater(residual(m.p, rows, np.ones((1, 2), dtype=bool), np.ones(2))[0], 0)


if __name__ == '__main__':
    unittest.main()
