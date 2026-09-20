import json
import re
import unittest
from pathlib import Path

import numpy as np

from design_speed_reference_mpc import BUILD, design


class SpeedReferenceMpcDesignTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = Path(__file__).resolve().parents[2]
        cls.model = cls.root / "NoMachineTemp/speed-reference-model-59490702-s1/report.json"
        cls.result = design(cls.model)

    def test_frozen_identity_and_dimensions(self):
        self.assertEqual(self.result["build"], f"0x{BUILD:08X}")
        self.assertEqual(
            self.result["model_report_sha256"],
            "1E81D3F94F3C81E3CE5A9A04481AABFCFE0270A8753BC4D3CE2C7A7725A2D53D",
        )
        gain = np.asarray(self.result["controller"]["first_move_feedback_gain"])
        self.assertEqual(gain.shape, (2, 14))
        self.assertTrue(np.isfinite(gain).all())

    def test_frozen_model_scenarios_respect_mechanical_soft_limits(self):
        for scenario in self.result["deterministic_model_scenarios"]:
            self.assertLess(scenario["peak_abs_small_joint_deg"], 56.677734375)
            self.assertLess(abs(scenario["final_heading_error_deg"]), 0.01)
            self.assertLess(abs(scenario["final_small_joint_deg"]), 0.01)

    def test_committed_json_and_firmware_gain_match_design(self):
        committed = json.loads(
            (Path(__file__).with_name("speed_reference_mpc_deployment_59490903.json"))
            .read_text(encoding="utf-8")
        )
        np.testing.assert_allclose(
            committed["controller"]["first_move_feedback_gain"],
            self.result["controller"]["first_move_feedback_gain"],
            rtol=0,
            atol=1e-12,
        )
        source = (
            self.root
            / "AGVSentinel_v10090/AGVSentinel_gimbal/Src/Modules/module_yaw_mpc.c"
        ).read_text(encoding="utf-8")
        block = source.split("static const float s_gain", 1)[1].split("};", 1)[0]
        values = [float(x) for x in re.findall(r"([-+]?\d+\.\d+)f", block)]
        np.testing.assert_allclose(
            np.asarray(values).reshape(2, 14),
            self.result["controller"]["first_move_feedback_gain"],
            rtol=0,
            atol=5e-9,
        )


if __name__ == "__main__":
    unittest.main()
