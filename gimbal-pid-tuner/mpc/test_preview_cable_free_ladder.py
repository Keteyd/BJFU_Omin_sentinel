import json
import unittest

from preview_cable_free_ladder import (DEFAULT_PLAN, SOURCE_PLAN, calculate)


class CableFreeLadderPreviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan = json.loads(DEFAULT_PLAN.read_text(encoding='utf-8'))
        cls.source = json.loads(SOURCE_PLAN.read_text(encoding='utf-8'))
        cls.report, _, _ = calculate(cls.plan, cls.source)

    def test_ladder_is_exactly_one_four_and_five_times_phase_e(self):
        self.assertEqual([self.report['phases'][name]['scale']
                          for name in ('R1', 'F4', 'F5')], [1., 4., 5.])
        base = self.report['phases']['R1']
        for name, scale in (('F4', 4.), ('F5', 5.)):
            for key, value in base.items():
                if key not in ('scale',):
                    self.assertAlmostEqual(self.report['phases'][name][key],
                                           value*scale, places=6)

    def test_five_times_keeps_nominal_travel_reserve(self):
        f5 = self.report['phases']['F5']
        guards = self.plan['unchanged_runtime_guards']
        self.assertLess(f5['big_peak_deg'], guards['big_relative_travel_abort_deg'])
        self.assertLess(f5['heading_peak_deg'], guards['heading_relative_travel_abort_deg'])
        self.assertLess(f5['planar_implied_small_joint_peak_deg'],
                        guards['small_relative_travel_abort_deg'])

    def test_ten_times_is_rejected_by_two_travel_guards(self):
        ten = self.plan['rejected_10x_preview']
        guards = self.plan['unchanged_runtime_guards']
        self.assertGreater(ten['big_peak_deg'], guards['big_relative_travel_abort_deg'])
        self.assertGreater(ten['planar_implied_small_joint_peak_deg'],
                           guards['small_relative_travel_abort_deg'])


if __name__ == '__main__':
    unittest.main()
