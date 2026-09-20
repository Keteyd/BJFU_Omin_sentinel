"""Cross-check the browser's fixed packet fixture against the deployed Python codec."""
import struct
import unittest

from pitch_tune_cli import make_frame, extract_frames
from big_yaw_tune_cli import TuneTransaction
from unittest.mock import patch


class BrowserWireTests(unittest.TestCase):
    def test_browser_tune_matches_cli_transaction(self):
        browser = bytes([255,45,160,15,188,2,160,15,44,1,210,4,183,1,132,13])
        reply = make_frame(0x2E, struct.pack("<5H2B", 4000,700,4000,300,58792,1,0x93))
        with patch("big_yaw_tune_cli.secrets.randbelow", return_value=1233):
            tune = TuneTransaction({"angle_kp":4.0})
            self.assertEqual(tune.receive(reply), browser)
        self.assertEqual(extract_frames(bytearray(browser)), [browser])


if __name__ == "__main__":
    unittest.main()
