import csv
import json
import tempfile
import unittest
from pathlib import Path

from review_cablefree_capture import review


FIELDS = ('big_raw', 'small_raw', 'yaw_deg', 'big_mean_attempted_command_raw',
          'small_mean_attempted_command_raw', 'big_feedback_rpm',
          'small_feedback_rpm')


class CableFreeCaptureReviewTests(unittest.TestCase):
    def capture(self, *, small_command=3000, quality=None):
        temporary = tempfile.TemporaryDirectory()
        path = Path(temporary.name)
        report = {
            'download_complete': True,
            'quality_issues': quality or [],
            'crc_errors': 0,
            'firmware_stream_quality_latched': False,
            'big_stop_reason_name': 'none',
            'ladder_phase_name': 'F4',
            'last_status': {'phase': 5, 'reason': 0},
        }
        (path/'report.json').write_text(json.dumps(report), encoding='utf-8')
        rows = [
            dict(big_raw=100, small_raw=200, yaw_deg=10,
                 big_mean_attempted_command_raw=0,
                 small_mean_attempted_command_raw=0,
                 big_feedback_rpm=0, small_feedback_rpm=0),
            dict(big_raw=200, small_raw=300, yaw_deg=12,
                 big_mean_attempted_command_raw=5000,
                 small_mean_attempted_command_raw=small_command,
                 big_feedback_rpm=10, small_feedback_rpm=10),
        ]
        with (path/'samples.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS)
            writer.writeheader(); writer.writerows(rows)
        return temporary, path

    def test_clean_capture_with_margin_allows_optional_f5(self):
        temporary, path = self.capture()
        with temporary:
            result = review(path)
        self.assertTrue(result['f5_allowed_after_F4'])

    def test_small_command_above_eighty_percent_stops_before_f5(self):
        temporary, path = self.capture(small_command=5000)
        with temporary:
            result = review(path)
        self.assertFalse(result['f5_allowed_after_F4'])

    def test_quality_issue_stops_before_f5(self):
        temporary, path = self.capture(quality=['incomplete_stream'])
        with temporary:
            result = review(path)
        self.assertFalse(result['structural_quality_passed'])
        self.assertFalse(result['f5_allowed_after_F4'])


if __name__ == '__main__':
    unittest.main()
