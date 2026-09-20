import copy
import json
import math
import unittest

from preview_slow_excitation import DEFAULT_PLAN, audit, reference, validate


class SlowPreviewTests(unittest.TestCase):
    def setUp(self):
        self.plan = json.loads(DEFAULT_PLAN.read_text(encoding='utf-8'))

    def test_resource_budget_and_unauthorized(self):
        result = audit(self.plan)
        self.assertIs(result['hardware_execution_authorized'], False)
        self.assertEqual(result['records'], 5001)
        self.assertEqual(result['full_capture_bytes'], 240048)
        self.assertGreater(result['existing_8n1_uart_sample_utilization'], 2)
        self.assertLess(result['proposed_8n1_uart_sample_utilization'], .6)

    def test_peaks_holds_and_zero_endpoints(self):
        for axis, amplitude in self.plan['peak_target_deg'].items():
            for t, target in zip(self.plan['knots_ms'], self.plan['normalized_targets']):
                self.assertEqual(reference(self.plan, t, axis), (amplitude*target, 0., 0.))
            self.assertEqual(reference(self.plan, 5500, axis), (amplitude, 0., 0.))
            self.assertEqual(reference(self.plan, 12500, axis), (-amplitude, 0., 0.))

    def test_full_profile_derivative_bounds_and_continuity(self):
        result = audit(self.plan)
        for axis in ('big', 'small'):
            values = [reference(self.plan, t, axis) for t in range(0, 20001, 4)]
            bound = result['axes'][axis]
            self.assertAlmostEqual(max(abs(v[0]) for v in values), bound['peak_target_deg'])
            self.assertAlmostEqual(max(abs(v[1]) for v in values), bound['max_target_rate_dps'])
            self.assertLessEqual(max(abs(v[2]) for v in values), bound['max_target_acceleration_dps2']+1e-10)
            for knot in self.plan['knots_ms'][1:-1]:
                before, after = reference(self.plan, knot-.0001, axis), reference(self.plan, knot+.0001, axis)
                for a, b in zip(before, after):
                    self.assertLess(abs(a-b), .0001)

    def test_numerical_derivatives(self):
        for t in (2500, 3200, 8000, 9500, 15000):
            y, v, a = reference(self.plan, t, 'big')
            left, right = reference(self.plan, t-.01, 'big'), reference(self.plan, t+.01, 'big')
            self.assertAlmostEqual((right[0]-left[0])/.00002, v, places=6)
            self.assertAlmostEqual((right[1]-left[1])/.00002, a, places=6)

    def test_reverse_trial_mirrors_reference(self):
        for t in range(0, 20001, 4):
            self.assertEqual(reference(self.plan, t, 'small', -1),
                             tuple(-v for v in reference(self.plan, t, 'small')))

    def test_old_start_pose_has_insufficient_corridor(self):
        self.plan['small_joint_start_abs_max_deg'] = 8.4375
        with self.assertRaisesRegex(ValueError, 'corridor'):
            validate(self.plan)

    def test_draft_cannot_be_authorized(self):
        self.plan['hardware_execution_authorized'] = True
        with self.assertRaisesRegex(ValueError, 'unauthorized'):
            validate(self.plan)

    def test_reject_invalid_profile(self):
        for value in (math.nan, math.inf, -1., 0.):
            plan = copy.deepcopy(self.plan)
            plan['peak_target_deg']['big'] = value
            with self.assertRaises(ValueError):
                validate(plan)
        for t in (-1, 20001, math.nan):
            with self.assertRaises(ValueError):
                reference(self.plan, t, 'big')
        self.plan['knots_ms'][2] = 2000
        with self.assertRaises(ValueError):
            validate(self.plan)


if __name__ == '__main__':
    unittest.main()
