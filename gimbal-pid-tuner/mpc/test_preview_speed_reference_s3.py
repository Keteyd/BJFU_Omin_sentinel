import json
import unittest

from preview_speed_reference_s3 import DEFAULT_PLAN, preview


class SpeedReferenceS3DesignTest(unittest.TestCase):
    def test_reference_is_bounded_zero_travel_and_frequency_disjoint(self):
        report = preview()
        self.assertTrue(report["frequencies_disjoint_from_S1_S2"])
        for axis, budget in (("big", 30.), ("small", 60.)):
            metrics = report["reference_metrics_at_4ms"][axis]
            self.assertLessEqual(metrics["peak_abs_speed_dps"], budget)
            self.assertLess(abs(metrics["net_position_deg"]), 1e-6)
            self.assertEqual(metrics["start_dps"], 0.)
            self.assertEqual(metrics["end_dps"], 0.)

    def test_gate_is_time_coverage_based_and_keeps_S2_numeric_thresholds(self):
        plan = json.loads(DEFAULT_PLAN.read_text(encoding="utf-8"))
        data = plan["prospective_S3_acceptance_gate"]["data_quality"]
        model = plan["prospective_S3_acceptance_gate"]["model_quality"]
        self.assertIn("no minimum count", data["raw_record_count_policy"])
        self.assertEqual(data["time_domain_coverage"]["source_interval_us_maximum"], 10000)
        self.assertEqual(model["primary_horizon_ms"], 200)
        self.assertEqual(model["minimum_improvement_over_hold_fraction"], {
            "big_motor_speed_dps": .2, "small_inertial_heading_rate_dps": .5})
        self.assertEqual(model["maximum_rmse_dps"], {
            "big_motor_speed_dps": 6., "small_inertial_heading_rate_dps": 6.})


if __name__ == "__main__":
    unittest.main()
