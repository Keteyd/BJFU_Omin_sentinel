import copy
import json
import unittest

import numpy as np

from preview_dual_reference_excitation import DEFAULT_PLAN, audit, reference, time_grid, validate


class DualReferencePreviewTests(unittest.TestCase):
    def setUp(self):
        self.plan = json.loads(DEFAULT_PLAN.read_text(encoding='utf-8'))
        self.time = time_grid(self.plan)

    def test_offline_status_and_existing_wire_budget(self):
        result = audit(self.plan)
        self.assertIs(result['hardware_execution_authorized'], False)
        self.assertIs(result['firmware_ready'], False)
        self.assertIs(result['field_execution_allowed'], False)
        self.assertEqual(result['records_per_trial'], 5001)
        self.assertEqual(result['record_bytes'], 152)
        self.assertLess(result['sample_uart_8n1_utilization'], .9)

    def test_zero_holds_and_peak_budgets(self):
        for phase_set in ('A', 'B'):
            values = reference(self.plan, phase_set, self.time)
            self.assertTrue(np.all(values[:501] == 0))
            self.assertTrue(np.all(values[4000:] == 0))
            for i, axis in enumerate(('big', 'small')):
                self.assertLessEqual(np.max(np.abs(values[:, i])),
                                     self.plan['axes'][axis]['peak_budget_deg']+1e-12)

    def test_both_polarities_and_low_axis_correlation(self):
        result = audit(self.plan)
        for phase in result['phase_sets'].values():
            self.assertLess(abs(phase['reference_correlation_during_excitation']), .15)
            for axis in ('big', 'small'):
                stats = phase['axes'][axis]
                self.assertGreater(stats['positive_fraction_above_threshold'], .1)
                self.assertGreater(stats['negative_fraction_above_threshold'], .1)
                self.assertGreaterEqual(stats['zero_crossings'], 3)

    def test_independent_repeat_phase_sets(self):
        result = audit(self.plan)
        self.assertLess(abs(result['repeat_similarity']['big']), .8)
        self.assertLess(abs(result['repeat_similarity']['small']), .8)

    def test_rejects_overlap_and_authorization(self):
        bad = copy.deepcopy(self.plan)
        bad['axes']['small']['harmonics_per_14s'][0] = 2
        with self.assertRaisesRegex(ValueError, 'disjoint'):
            validate(bad)
        bad = copy.deepcopy(self.plan)
        bad['hardware_execution_authorized'] = True
        with self.assertRaisesRegex(ValueError, 'unauthorized'):
            validate(bad)

    def test_rejects_invalid_time_or_phase(self):
        with self.assertRaises(ValueError):
            reference(self.plan, 'C', self.time)
        with self.assertRaises(ValueError):
            reference(self.plan, 'A', np.asarray([-0.001]))


if __name__ == '__main__':
    unittest.main()
