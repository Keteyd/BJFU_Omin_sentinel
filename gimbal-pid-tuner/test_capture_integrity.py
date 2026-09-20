import contextlib
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest

from audit_capture_integrity import audit, main, timing_summary
from test_yaw_slow import fixture, info, status
from yaw_slow_protocol import Decoder


class TimingTests(unittest.TestCase):
    def test_wrap_is_not_clock_reversal(self):
        r = timing_summary([0xfffffffc, 0, 4], 4, 0xfffffffc)
        self.assertEqual(r['uint32_wraps'], 1)
        self.assertEqual(r['elapsed_ms'], 8)
        self.assertEqual(r['irregular_intervals'], 0)
        self.assertTrue(r['first_tick_matches_metadata'])

    def test_jitter_preserves_actual_intervals(self):
        r = timing_summary([100, 104, 109, 112], 4, 100)
        self.assertEqual(r['interval_histogram_ms'], {'3': 1, '4': 1, '5': 1})
        self.assertEqual(r['max_grid_error_ms'], 1)
        self.assertEqual(r['irregular_intervals'], 2)

    def test_duplicate_backward_empty_and_single(self):
        r = timing_summary([10, 10, 9], 4)
        self.assertEqual(r['duplicate_timestamps'], 1)
        self.assertEqual(r['backwards_or_ambiguous_intervals'], 1)
        self.assertIsNone(r['elapsed_ms'])
        self.assertIsNone(timing_summary([], 4)['elapsed_ms'])
        self.assertIsNone(timing_summary([10], 4)['strictly_forward'])
        with self.assertRaises(ValueError):
            timing_summary([-1], 4)


class IntegrityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.raw = info()+fixture()

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.report = dict(trial_id=1, records=5001, download_complete=True, crc_errors=0)

    def capture(self, raw=None):
        (self.root/'raw.bin').write_bytes(self.raw if raw is None else raw)
        (self.root/'report.json').write_text(json.dumps(self.report), encoding='utf-8')
        return audit(self.root)

    def test_valid_capture_and_source_unchanged(self):
        r = self.capture()
        self.assertEqual(r['findings'], [])
        self.assertEqual(r['raw_sha256'], hashlib.sha256(self.raw).hexdigest())
        self.assertEqual(r['raw_bytes'], r['bytes_consumed'])
        self.assertEqual(r['frame_counts']['0x3d'], 6*5001)
        self.assertFalse(r['hardware_access'])
        self.assertEqual((self.root/'raw.bin').read_bytes(), self.raw)

    def test_leading_noise_and_trailing_partial_status_are_visible(self):
        r = self.capture(b'noise'+self.raw+status()[:7])
        self.assertTrue(r['download_complete'])
        self.assertEqual(r['discarded_bytes'], 5)
        self.assertEqual(r['pending_bytes'], 7)
        self.assertIn('unparsed_bytes_remaining', r['findings'])

    def test_truncated_measurements_fail(self):
        r = self.capture(self.raw[:-16])
        self.assertFalse(r['download_complete'])
        self.assertEqual(r['pending_fragment_groups'], 1)

    def test_crc_failure_reports_byte_offset_and_hashes_entire_file(self):
        raw = bytearray(self.raw)
        raw[14] ^= 1
        r = self.capture(raw)
        self.assertEqual(r['first_crc_error_offset'], 0)
        self.assertEqual(r['crc_errors'], 1)
        self.assertFalse(r['download_complete'])
        self.assertEqual(r['raw_sha256'], hashlib.sha256(raw).hexdigest())
        self.assertLess(r['bytes_submitted_to_decoder'], r['raw_bytes'])

    def test_claim_mismatch_and_invalid_id(self):
        self.report['records'] = 12
        r = self.capture()
        self.assertIn('stored_summary_differs_from_raw', r['findings'])
        self.report['trial_id'] = None
        with self.assertRaises(ValueError):
            self.capture()

    def test_output_is_exclusive_and_warning_exit_code(self):
        self.capture(b'noise'+self.raw)
        output = self.root/'audit.json'
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(main([str(self.root), '--output', str(output)]), 2)
        before = output.read_bytes()
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as exc:
            main([str(self.root), '--output', str(output)])
        self.assertEqual(exc.exception.code, 1)
        self.assertEqual(output.read_bytes(), before)

    def test_counters_independent_of_chunk_boundaries(self):
        raw = b'noise'+self.raw+status()[:7]
        results = []
        for size in (127, 4096):
            d = Decoder(1)
            for i in range(0, len(raw), size):
                d.feed(raw[i:i+size])
            results.append((d.bytes_received, d.bytes_consumed, d.discarded_bytes, len(d.buffer), d.frame_counts))
        self.assertEqual(*results)


if __name__ == '__main__':
    unittest.main()
