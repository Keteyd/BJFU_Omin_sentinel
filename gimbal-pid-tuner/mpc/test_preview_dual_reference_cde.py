import hashlib
import json
import unittest

from preview_dual_reference_cde import DEFAULT_PLAN, PHASES, audit


class CDEReferencePreviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan = json.loads(DEFAULT_PLAN.read_text(encoding='utf-8'))
        cls.report = audit(cls.plan)

    def test_plan_is_frozen_and_hash_is_stable(self):
        self.assertEqual(self.plan['status'], 'FROZEN_E_INDEPENDENT_VALIDATION_PLAN')
        self.assertEqual(
            hashlib.sha256(DEFAULT_PLAN.read_bytes()).hexdigest().upper(),
            'F59EA206EBCBFE0914C10CE6619B49F5EF2E6257625EACA229A6BD9AAB0A84BE')

    def test_all_reference_limits_pass(self):
        self.assertEqual(PHASES, ('C', 'D', 'E'))
        self.assertTrue(self.report['preview_limits_passed'])
        e = self.report['phase_sets']['E']
        self.assertGreaterEqual(e['axes']['big']['peak_abs_deg'], 2.7)
        self.assertGreaterEqual(e['axes']['small']['peak_abs_deg'], 1.6)
        self.assertLessEqual(abs(e['active_cross_axis_correlation']), 0.1)
        self.assertLessEqual(e['axes']['big']['maximum_abs_rate_dps'], 12.0)
        self.assertLessEqual(e['axes']['small']['maximum_abs_rate_dps'], 12.0)
        self.assertLessEqual(e['axes']['big']['maximum_abs_acceleration_dps2'], 200.0)
        self.assertLessEqual(e['axes']['small']['maximum_abs_acceleration_dps2'], 200.0)

    def test_e_is_distinct_from_both_prior_phase_sets(self):
        for pair in ('C_E', 'D_E'):
            for value in self.report['same_axis_pairwise_correlation'][pair].values():
                self.assertLessEqual(abs(value), 0.4)


if __name__ == '__main__':
    unittest.main()
