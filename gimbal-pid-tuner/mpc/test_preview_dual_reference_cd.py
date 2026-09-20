import copy
import json
import unittest

import numpy as np

from preview_dual_reference_cd import DEFAULT_PLAN, audit, reference, time_grid, validate


class DualReferenceCDTests(unittest.TestCase):
    def setUp(self):
        self.plan = json.loads(DEFAULT_PLAN.read_text(encoding='utf-8'))

    def test_draft_is_offline_and_wire_count_is_consistent(self):
        report = audit(self.plan)
        self.assertFalse(report['hardware_execution_authorized'])
        self.assertFalse(report['field_execution_allowed'])
        self.assertTrue(report['firmware_ready'])
        self.assertEqual(report['nominal_records'], 9001)
        self.assertTrue(report['preview_limits_passed'])

    def test_boundaries_budgets_and_both_polarities(self):
        t = time_grid(self.plan)
        for phase in ('C', 'D'):
            values = reference(self.plan, phase, t)
            np.testing.assert_allclose(values[[0, 750, 2250, -1]], 0, atol=1e-12)
            self.assertLessEqual(np.max(np.abs(values[:, 0])), 3)
            self.assertLessEqual(np.max(np.abs(values[:, 1])), 2)
            self.assertTrue(np.all(values.max(axis=0) > .2))
            self.assertTrue(np.all(values.min(axis=0) < -.2))

    def test_operator_approved_full_budget_amplitudes_are_frozen(self):
        self.assertEqual(
            self.plan['amplitude_selection']['name'],
            'full_budget_low_frequency_enhancement',
        )
        expected = {
            'big': [1.45, 0.86, 0.45, 0.18, 0.06],
            'small': [0.97, 0.605, 0.28, 0.11, 0.035],
        }
        for axis, amplitudes in expected.items():
            actual = self.plan['multisine']['axes'][axis]['component_amplitudes_deg']
            self.assertEqual(actual, amplitudes)
            self.assertAlmostEqual(
                sum(actual),
                self.plan['multisine']['axes'][axis]['peak_budget_deg'],
            )

    def test_C_D_and_axis_correlations_are_bounded(self):
        report = audit(self.plan)
        limit = self.plan['preview_limits']
        for phase in ('C', 'D'):
            self.assertLessEqual(abs(report['phase_sets'][phase]['active_cross_axis_correlation']),
                                 limit['maximum_abs_cross_axis_correlation'])
        for value in report['same_axis_C_D_correlation'].values():
            self.assertLessEqual(abs(value), limit['maximum_abs_same_axis_C_D_correlation'])

    def test_rejects_authorization_overlap_and_bad_independent_role(self):
        for mutate in (
                lambda p: p.__setitem__('hardware_execution_authorized', True),
                lambda p: p['multisine']['axes']['small']['harmonics_per_22s'].__setitem__(0, 4),
                lambda p: p['offline_acceptance_gates'].__setitem__('D_role', 'fit and validate'),
        ):
            plan = copy.deepcopy(self.plan); mutate(plan)
            with self.assertRaises(ValueError):
                validate(plan)

    def test_invalid_phase_or_time_is_rejected(self):
        with self.assertRaises(ValueError):
            reference(self.plan, 'A', np.array([0.]))
        with self.assertRaises(ValueError):
            reference(self.plan, 'C', np.array([-0.1]))


if __name__ == '__main__':
    unittest.main()
