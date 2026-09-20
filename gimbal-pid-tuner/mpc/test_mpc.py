from types import SimpleNamespace
import csv
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from controller import MPC
from audit_capture import audit
from model import HEADING, SOFT_MAX, SOFT_MIN, synthetic_model
from simulate import run_case, scenarios


class MPCTests(unittest.TestCase):
    def test_invalid_configuration(self):
        for dt in (0, -1, np.nan, np.inf):
            with self.assertRaises(ValueError):
                synthetic_model(dt=dt)
        for horizon in (0, 1, 2.5, 201):
            with self.assertRaises(ValueError):
                MPC(synthetic_model(), horizon=horizon)

    def test_coupled_second_order_plant(self):
        model = synthetic_model()
        # One joint drive accelerates both relative coordinates.
        self.assertLess(model.b[2, 1], 0)
        self.assertGreater(model.b[3, 1], 0)
        self.assertGreater(model.b[0, 0], 0)
        self.assertIn("UNIDENTIFIED", model.provenance)

    def test_limits_match_firmware_landmarks(self):
        self.assertAlmostEqual(np.rad2deg(SOFT_MIN), -56.677734375)
        self.assertAlmostEqual(np.rad2deg(SOFT_MAX), 29.080078125)

    def test_zero_equilibrium(self):
        result = MPC(synthetic_model()).solve(np.zeros(6), 0.)
        self.assertTrue(result.accepted, result.status)
        np.testing.assert_allclose(result.command, 0., atol=1e-8)

    def test_invalid_and_stale_return_no_command(self):
        mpc = MPC(synthetic_model())
        for kwargs in (dict(state=[np.nan] * 6), dict(state=[0] * 5),
                       dict(sample_age_s=.5), dict(sample_age_s=-1),
                       dict(heading_reference=np.inf), dict(previous_input=[31, 0]),
                       dict(heading_reference=1e308),
                       dict(deadline_s=0)):
            arguments = dict(state=np.zeros(6), heading_reference=0.)
            arguments.update(kwargs)
            with self.subTest(kwargs=kwargs):
                result = mpc.solve(**arguments)
                self.assertFalse(result.accepted)
                self.assertIsNone(result.command)

    def test_outside_limits_not_silently_clipped(self):
        state = np.zeros(6); state[1] = SOFT_MAX + .001
        result = MPC(synthetic_model()).solve(state, 0.)
        self.assertEqual(result.status, "outside_soft_limits")
        self.assertIsNone(result.command)

    def test_unbrakeable_outward_motion_is_rejected(self):
        state = np.zeros(6); state[1] = SOFT_MAX - .0001; state[3] = 20.
        result = MPC(synthetic_model()).solve(state, HEADING @ state)
        self.assertFalse(result.accepted)
        self.assertIsNone(result.command)

    def test_failure_never_reuses_previous_solution(self):
        mpc = MPC(synthetic_model())
        self.assertTrue(mpc.solve(np.zeros(6), .05).accepted)
        for status in ("solved inaccurate", "maximum iterations reached", "run time limit reached",
                       "primal infeasible", "dual infeasible"):
            fake = SimpleNamespace(info=SimpleNamespace(status=status), x=np.zeros(mpc.size))
            with patch.object(mpc.solver, "solve", return_value=fake):
                result = mpc.solve(np.zeros(6), .05)
                self.assertFalse(result.accepted)
                self.assertIsNone(result.command)

    def test_residual_and_nonfinite_solution_rejection(self):
        mpc = MPC(synthetic_model())
        for value in (np.full(mpc.size, np.nan), np.full(mpc.size, 100.)):
            fake = SimpleNamespace(info=SimpleNamespace(status="solved"), x=value)
            with patch.object(mpc.solver, "solve", return_value=fake):
                self.assertFalse(mpc.solve(np.zeros(6), 0.).accepted)

    def test_full_turn_is_not_wrapped_backwards(self):
        mpc = MPC(synthetic_model())
        x = np.zeros(6); x[0] = 2 * np.pi + .02
        r = mpc.solve(x, x[0] + .05, big_anchor=x[0])
        self.assertTrue(r.accepted, r.status)
        self.assertGreater(HEADING @ r.states[-1], x[0])
        self.assertLess(abs(r.states[-1, 0] - x[0]), .5)

    def test_closed_loop_regressions(self):
        for name, initial, target, scale in scenarios():
            with self.subTest(name=name):
                _, result = run_case(name, initial, target, scale)
                self.assertLess(abs(result["final_error_deg"]), 1.)
                self.assertGreaterEqual(result["small_range_deg"][0], np.rad2deg(SOFT_MIN) - .002)
                self.assertLessEqual(result["small_range_deg"][1], np.rad2deg(SOFT_MAX) + .002)
                self.assertLessEqual(result["peak_inputs"][0], 30.0001)
                self.assertLessEqual(result["peak_inputs"][1], 6.0001)
                if name == "positive_boundary":
                    self.assertLess(result["final_small_deg"], np.rad2deg(SOFT_MAX) - 4.)
                elif name == "negative_boundary":
                    self.assertGreater(result["final_small_deg"], np.rad2deg(SOFT_MIN) + 4.)

    def test_big_motion_cost_reduces_motion(self):
        target = lambda t: np.deg2rad(10)
        _, quiet = run_case("quiet", np.zeros(6), target, big_motion_weight=25.)
        _, loose = run_case("loose", np.zeros(6), target, big_motion_weight=0.)
        self.assertLess(quiet["max_big_displacement_deg"], loose["max_big_displacement_deg"])

    def test_zero_big_input_audit_blocks_identification(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "samples.csv"
            fields = ["valid", "tick_ms", "segment", "sequence", "big_command", "small_command"]
            with path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                for i in range(20):
                    writer.writerow(dict(valid=1, tick_ms=i * 2, segment=0, sequence=i,
                                         big_command=0, small_command=np.sin(i)))
            result = audit(path)
            self.assertEqual(result["input_rank"], 1)
            self.assertFalse(result["model_identified"])
            self.assertFalse(result["hardware_takeover_allowed"])
            self.assertTrue(result["blockers"])


if __name__ == "__main__":
    unittest.main()
