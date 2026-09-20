import unittest
from unittest.mock import patch
import struct

import yaw_limits_cli as cli


def sample(t, small=8191, big=500, age=0, active=0, monitor=1, version=1):
    return cli.decode(cli.make_frame(0x2A, struct.pack(
        "<4H4B", small, big, age, age, 3, active, monitor, version)), t)


class YawLandmarkTests(unittest.TestCase):
    def peak_samples(self):
        rows = self.observation_samples()
        latest = None
        for i, row in enumerate(rows):
            if i % 2 == 0:
                latest = cli.decode_peaks(cli.make_frame(0x2F, struct.pack(
                    "<6H", 100, 1024, 500, 1200, 4000, 100)), row["time_s"])
                row["big_yaw_peak_windows"] = [latest]
            row["big_yaw_peaks"] = latest
        return rows

    def test_peak_packet_and_window_summary(self):
        rows = self.peak_samples()
        result = cli.summarize(rows, 3, observe=True, require_peaks=True)
        self.assertTrue(result["accepted"], result)
        self.assertEqual(result["mcu_peaks"]["windows"], 30)
        self.assertEqual(result["mcu_peaks"]["active_task_samples"], 3000)
        self.assertEqual(result["mcu_peaks"]["max_window_span_deg"], 45)
        self.assertEqual(result["mcu_peaks"]["peak_error_deg"], 5)
        self.assertEqual(result["mcu_peaks"]["peak_raw_speed_rpm"], 12)
        self.assertEqual(result["mcu_peaks"]["peak_effort"], 4)
        decoded = cli.decode_peaks(cli.make_frame(0x2F, struct.pack(
            "<6H", 0xC064, 1, 2, 3, 4, 100)), 0)
        self.assertEqual(decoded["task_samples"], 100)
        self.assertTrue(decoded["saturated"])
        self.assertTrue(decoded["invalid"])

    def test_require_peaks_rejects_missing_stale_invalid_or_lost_windows(self):
        self.assertFalse(cli.summarize(self.observation_samples(), 3,
                                      observe=True, require_peaks=True)["accepted"])
        for defect in ("stale", "invalid", "lost", "truncated"):
            rows = self.peak_samples()
            if defect == "stale":
                rows[20]["big_yaw_peaks"]["time_s"] = 0
            elif defect == "invalid":
                rows[20]["big_yaw_peaks"]["invalid"] = True
            elif defect == "lost":
                for row in rows[20:26]:
                    row["big_yaw_peak_windows"] = []
            else:
                for row in rows[40:]:
                    row["big_yaw_peak_windows"] = []
            self.assertFalse(cli.summarize(rows, 3, observe=True,
                                          require_peaks=True)["accepted"], defect)
        with self.assertRaises(ValueError):
            cli.summarize(self.samples(), 3, require_peaks=True)

    def test_collect_retains_all_peak_windows_in_read_batch(self):
        clock = [0.0]
        peak = cli.make_frame(0x2F, struct.pack("<6H", 100, 20, 30, 40, 50, 100))
        encoder = cli.make_frame(0x2A, struct.pack("<4H4B", 500, 600, 0, 0, 3, 1, 1, 1))
        def ready(*args):
            clock[0] += 0.04
            return [7], [], []
        with patch.object(cli.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(cli.select, "select", side_effect=ready), \
             patch.object(cli.os, "read", return_value=peak + peak + encoder), \
             patch.object(cli.os, "write", return_value=16):
            rows = cli.collect(7, 0.2)
        self.assertTrue(rows)
        self.assertTrue(all(len(row["big_yaw_peak_windows"]) == 2 for row in rows))

    def observation_samples(self):
        rows = self.samples()
        for i, row in enumerate(rows):
            row["yaw_active"] = True
            row["big_raw_count"] = (8000 + i * 300) % 8192
            row["soft_limits"] = cli.decode_limits(cli.make_frame(
                0x2B, struct.pack("<4h4B", 0, -5667, 2908, 0, 1, 0, 1, 1)), row["time_s"])
            row["coordinator"] = cli.decode_coordinator(cli.make_frame(
                0x2C, struct.pack("<4h4B", -25, 150, 100, 200, 2, 1, 0, 1)), row["time_s"])
        return rows

    def test_coordinator_packet(self):
        decoded = cli.decode_coordinator(cli.make_frame(
            0x2C, struct.pack("<4h4B", -25, 150, 100, -200, 3, 1, 1, 1)), 0.1)
        self.assertEqual(decoded["big_error_deg"], -0.25)
        self.assertEqual(decoded["big_speed_ref_rpm"], 1.5)
        self.assertEqual(decoded["big_speed_rpm"], 1.0)
        self.assertEqual(decoded["big_effort"], -0.2)
        self.assertEqual(decoded["mode"], 3)
        self.assertEqual(decoded["relief_direction"], -1)
        self.assertTrue(decoded["active"])

    def test_observation_allows_motion_but_is_not_landmark(self):
        rows = self.observation_samples()
        result = cli.summarize(rows, 3, observe=True)
        self.assertTrue(result["accepted"])
        self.assertEqual(result["capture_mode"], "observation")
        self.assertGreater(result["big_net_delta_deg"], 720)
        self.assertEqual(result["coordinator_modes_seen"], [2])
        self.assertFalse(cli.summarize(rows, 3)["accepted"])

    def test_observation_rejects_missing_stale_guard_packets(self):
        for key in ("soft_limits", "coordinator"):
            rows = self.observation_samples()
            del rows[20][key]
            self.assertFalse(cli.summarize(rows, 3, observe=True)["accepted"])
            rows = self.observation_samples()
            rows[20][key]["time_s"] = 0
            self.assertFalse(cli.summarize(rows, 3, observe=True)["accepted"])

    def test_asymmetric_soft_limit_packet(self):
        packet = cli.make_frame(0x2B, struct.pack(
            "<4h4B", -300, -5667, 2908, -50, 1, 3, 0, 1))
        result = cli.decode_limits(packet, 0.25)
        self.assertEqual(result["joint_deg"], -3)
        self.assertEqual(result["minimum_deg"], -56.67)
        self.assertEqual(result["maximum_deg"], 29.08)
        self.assertEqual(result["speed_ref_rpm"], -5)
        self.assertTrue(result["valid"])
        self.assertEqual(result["status"], 3)
        self.assertFalse(result["big_yaw_enabled"])

    def samples(self):
        return [sample(round(0.05 * i, 4), small=(8191 + i % 2) % 8192)
                for i in range(1, 60)]

    def test_stationary_across_encoder_zero(self):
        result = cli.summarize(self.samples(), 3)
        self.assertTrue(result["accepted"])
        self.assertLess(result["small_span_deg"], 0.05)

    def test_reject_enabled_stale_and_old_protocol(self):
        for key, value in (("yaw_active", True), ("small_age_ms", 101),
                           ("feedback_valid_mask", 1), ("monitor_only", False),
                           ("protocol_version", 0), ("small_raw_count", 8192)):
            with self.subTest(key=key):
                rows = self.samples()
                rows[20][key] = value
                self.assertFalse(cli.summarize(rows, 3)["accepted"])

    def test_reject_motion_and_gaps(self):
        rows = self.samples()
        rows[20]["small_raw_count"] = 50
        self.assertFalse(cli.summarize(rows, 3)["accepted"])
        rows = self.samples()
        self.assertFalse(cli.summarize(rows[:20] + rows[30:], 3)["accepted"])
        self.assertFalse(cli.summarize([], 3)["accepted"])

    def test_only_monitor_commands_even_with_yaw_enabled(self):
        clock = [0.0]
        writes = []
        frame = cli.make_frame(0x2A, struct.pack("<4H4B", 500, 600, 0, 0, 3, 1, 1, 1))

        def ready(*args):
            clock[0] += 0.04
            return [7], [], []

        def write(fd, data):
            writes.append(data)
            return len(data)

        with patch.object(cli.time, "monotonic", side_effect=lambda: clock[0]), \
             patch.object(cli.select, "select", side_effect=ready), \
             patch.object(cli.os, "read", return_value=frame), \
             patch.object(cli.os, "write", side_effect=write):
            rows = cli.collect(7, 3)
        self.assertTrue(writes)
        self.assertTrue(all(w == cli.MONITOR_FRAME for w in writes))
        self.assertFalse(cli.summarize(rows, 3)["accepted"])


if __name__ == "__main__":
    unittest.main()
