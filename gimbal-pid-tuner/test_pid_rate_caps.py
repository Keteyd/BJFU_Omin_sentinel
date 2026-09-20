"""Offline codec tests. No serial device is opened."""

import argparse
import math
import struct
import unittest

from pid_tune_cli import encode_optional_cap, tune_payload


class RateCapTests(unittest.TestCase):
    def test_unlimited_and_zero_are_distinct(self):
        for scale in (100, 10000):
            self.assertEqual(encode_optional_cap(-1, scale), 0xFFFF)
            self.assertEqual(encode_optional_cap(0, scale), 0)

    def test_existing_positive_caps(self):
        self.assertEqual(encode_optional_cap(24, 100), 2400)
        self.assertEqual(encode_optional_cap(0.3, 10000), 3000)

    def test_invalid_does_not_become_unlimited(self):
        for value in (-2, -0.5, math.nan, math.inf, 655.35, 655.349):
            with self.assertRaises(ValueError):
                encode_optional_cap(value, 100)

    def test_payload_and_zero_effort_abort(self):
        args = argparse.Namespace(angle_kp=2.6, speed_kp=0.6, speed_limit=-1,
                                  filter_alpha=0.15, manual_step=-1, effort_limit=6)
        self.assertEqual(struct.unpack("<6H", tune_payload(args)),
                         (2600, 600, 65535, 1500, 65535, 6000))
        self.assertEqual(struct.unpack("<6H", tune_payload(args, effort_limit=0))[-1], 0)


if __name__ == "__main__":
    unittest.main()
