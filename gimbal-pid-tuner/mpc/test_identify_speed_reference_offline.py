import unittest

import numpy as np

from identify_speed_reference_offline import (ACTIVE, DT, endpoint_metrics, fit,
                                                spectral_radius)
from validate_speed_reference_offline import apply_gate


class SpeedReferenceIdentificationTests(unittest.TestCase):
    def test_recovers_stable_coupled_speed_dynamics(self):
        rng = np.random.default_rng(7)
        count = 1800
        reference = rng.normal(size=(count, 2))
        measured = np.zeros((count, 2))
        a = np.array([[.72, .04], [-.03, .61]])
        b = np.array([[.24, .05], [.02, .31]])
        for k in range(1, count):
            measured[k] = a@measured[k-1]+b@reference[k]
        data = {'time': np.arange(1, count+1)*DT,
                'reference': reference, 'measured': measured}
        scales = {'output_baseline': np.zeros(2),
                  'output_scale': measured.std(axis=0),
                  'input_scale': reference.std(axis=0)}
        structure = (1, 1, 0, 1e-5)
        coefficient = fit(data, structure, scales, ACTIVE)
        self.assertLess(spectral_radius(coefficient, 1), .9)
        metrics = endpoint_metrics(data, coefficient, structure, scales, (17., 24.), 10)
        self.assertGreater(min(metrics['improvement_over_hold_fraction'].values()), .95)

    def test_frozen_gate_requires_each_axis_and_metric(self):
        gate = {
            'primary_horizon_ms': 200,
            'minimum_improvement_over_hold_fraction': {
                'big_motor_speed_dps': .2, 'small_inertial_heading_rate_dps': .5},
            'maximum_rmse_dps': {
                'big_motor_speed_dps': 6., 'small_inertial_heading_rate_dps': 6.},
        }
        metrics = {'200': {
            'improvement_over_hold_fraction': {
                'big_motor_speed_dps': .3, 'small_inertial_heading_rate_dps': .6},
            'rmse_dps': {
                'big_motor_speed_dps': 5., 'small_inertial_heading_rate_dps': 5.},
        }}
        self.assertTrue(apply_gate(metrics, gate)[1])
        metrics['200']['rmse_dps']['small_inertial_heading_rate_dps'] = 6.01
        checks, passed = apply_gate(metrics, gate)
        self.assertFalse(passed)
        self.assertFalse(checks['small_inertial_heading_rate_dps_rmse'])


if __name__ == '__main__':
    unittest.main()
