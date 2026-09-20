import hashlib
import json
import math
import struct
import unittest
from pathlib import Path

from yaw_capture_protocol import frame
from yaw_slow_cli import parse_args
from yaw_s3_trace_protocol import (BUILD, HOST_DECODER_REVISION, NOMINAL_RECORDS,
                                   PHASE_OPS, PLAN_SHA256, Decoder, request,
                                   s3_capture_timing_summary,
                                   speed_reference_matches, speed_wave)


ROOT = Path(__file__).resolve().parent
PLAN = ROOT/'mpc/speed_reference_s3_plan.json'


class SpeedReferenceS3TraceTests(unittest.TestCase):
    def test_frozen_plan_hash_and_identity(self):
        self.assertEqual(hashlib.sha256(PLAN.read_bytes()).hexdigest().upper(), PLAN_SHA256)
        plan = json.loads(PLAN.read_text(encoding='utf-8'))
        self.assertEqual(plan['planned_firmware_build'], '0x59490801')
        self.assertEqual(plan['nominal_records'], NOMINAL_RECORDS)

    def test_request_and_cli_are_s3_only(self):
        op = PHASE_OPS['S3']
        packet = request(op, 7, 5, 0)
        self.assertEqual(struct.unpack('<IHBBI', packet[2:14])[2:4], (op, 5))
        parsed = parse_args([
            '--port', 'x', '--action', 'dual', '--phase-set', 'S3',
            '--confirm', 'S3_REFERENCE_FIXED_CHASSIS_CLEAR_YAW', '--output', 'x'],
            phase_sets=('S3',), dual_confirm='S3_REFERENCE_FIXED_CHASSIS_CLEAR_YAW')
        self.assertEqual(parsed.phase_set, 'S3')
        with self.assertRaises(ValueError):
            request(op, 1, 4, 0)

    def test_previous_build_is_rejected(self):
        old_info = frame(0x3b, struct.pack('<IIHH', 0x59490702, 0x31444959, 1, 152))
        with self.assertRaises(ValueError):
            Decoder().feed(old_info)

    def test_waveform_bounds_zero_integral_and_alignment(self):
        dt = .004
        for axis, lower, limit in ((0, 29., 30.), (1, 58., 60.)):
            values = [speed_wave(ms, axis) for ms in range(0, 28001, 4)]
            self.assertLessEqual(max(map(abs, values)), limit)
            self.assertGreater(max(map(abs, values)), lower)
            self.assertGreater(sum(value > .5 for value in values), 1000)
            self.assertGreater(sum(value < -.5 for value in values), 1000)
            self.assertLess(abs(sum(values)*dt), .02)
            self.assertTrue(all(math.isfinite(value) for value in values))
        elapsed_ms = 6057
        recorded = tuple(round(speed_wave(3056, axis), 2) for axis in (0, 1))
        self.assertTrue(speed_reference_matches(elapsed_ms, recorded))
        self.assertFalse(speed_reference_matches(
            elapsed_ms, (recorded[0], recorded[1]+.10)))

    def test_time_gate_accepts_full_span_without_raw_count_threshold(self):
        profile = {'samples': 9001, 'duration_ms': 36000}
        initial = {'start_ms': 1000}
        metadata = {'count': 8994, 'phase': 5, 'reason': 0}
        rows = []
        # Seven scheduler slots are absent, but every interval remains <=8 ms
        # and the complete S3 evaluation window is bracketed.
        missing = {700, 1700, 2700, 3700, 4700, 5700, 6700}
        previous = None
        for index in range(9001):
            if index in missing:
                continue
            trace = 500+index*4000
            rows.append({'trace_us': trace,
                         'interval_us': 0 if previous is None else trace-previous,
                         'tick_ms': 1000+index*4,
                         'phase': 5 if index == 9000 else 3})
            previous = trace
        timing = s3_capture_timing_summary(rows, initial, metadata, profile)
        self.assertEqual(timing['record_shortfall'], 7)
        self.assertTrue(timing['raw_record_count_is_diagnostic_only'])
        self.assertTrue(timing['active_20ms_grid_bracketed'])
        self.assertTrue(timing['full_duration_timing_accepted'])

    def test_time_gate_rejects_unbracketed_gap(self):
        profile = {'samples': 9001, 'duration_ms': 36000}
        initial = {'start_ms': 1000}
        rows = [
            {'trace_us': 500, 'interval_us': 0, 'tick_ms': 1000, 'phase': 3},
            {'trace_us': 3000000, 'interval_us': 2999500, 'tick_ms': 4000, 'phase': 3},
            {'trace_us': 36000500, 'interval_us': 33000500, 'tick_ms': 37000, 'phase': 5},
        ]
        metadata = {'count': len(rows), 'phase': 5, 'reason': 0}
        timing = s3_capture_timing_summary(rows, initial, metadata, profile)
        self.assertFalse(timing['full_duration_timing_accepted'])

    def test_native_s3_fixture_decodes_end_to_end(self):
        path = ROOT.parent/'NoMachineTemp/yaw-slow-native-tests/can-trace-speed-s3.bin'
        if not path.exists():
            self.skipTest('run native tests first')
        decoder = Decoder(11, (5, 2, 5, 0.))
        raw = path.read_bytes()
        for index in range(0, len(raw), 137):
            decoder.feed(raw[index:index+137])
        report = decoder.report()
        self.assertTrue(decoder.complete)
        self.assertEqual(report['quality_issues'], [])
        self.assertEqual(len(decoder.rows), NOMINAL_RECORDS)
        self.assertEqual(decoder.profile['build'], BUILD)
        self.assertTrue(report['capture_timing']['full_duration_timing_accepted'])
        self.assertEqual(report['host_decoder_revision'], HOST_DECODER_REVISION)


if __name__ == '__main__':
    unittest.main()
