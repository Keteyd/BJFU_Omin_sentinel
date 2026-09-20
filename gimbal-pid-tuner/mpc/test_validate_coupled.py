import copy
import unittest

import numpy as np

from identify_coupled_offline import Coupled
from validate_coupled_offline import check_configuration, frozen_model


class FrozenTests(unittest.TestCase):
    def setUp(self):
        self.model = Coupled(np.array([-.5, .4, 0., -.5, -3., -4., -.2, -.8]), .02)

    def test_roundtrip_has_same_dynamics_and_does_not_mutate(self):
        artifact = self.model.artifact()
        before = copy.deepcopy(artifact)
        restored = frozen_model(artifact)
        np.testing.assert_allclose(restored.ad, self.model.ad, rtol=1e-12, atol=1e-12)
        x, u = np.array([.1, -.2, .5, -.3]), np.array([.3, -.1])
        np.testing.assert_allclose(restored.step(x, u), self.model.step(x, u), atol=1e-12)
        self.assertEqual(artifact, before)

    def test_bad_units_or_dynamics_rejected(self):
        for key, value in (('angle_scales', [.85, 1]), ('normalized_mass', [[1, 2], [2, 1]]),
                           ('command_gain', [0, 1]), ('actuator_tau_s', float('nan')),
                           ('friction_smoothing_rad_s', 1)):
            a = self.model.artifact()
            a[key] = value
            with self.assertRaises(ValueError):
                frozen_model(a)

    def test_configuration_only_allows_pose_reference_changes(self):
        a = dict(big_angle_kp=1., small_joint_start_deg=0., amplitude_deg=10.)
        b = dict(a, small_joint_start_deg=1.)
        self.assertEqual(check_configuration(a, b)['small_joint_start_deg'], 1.)
        b['big_angle_kp'] = 2
        with self.assertRaises(ValueError):
            check_configuration(a, b)


if __name__ == '__main__':
    unittest.main()
