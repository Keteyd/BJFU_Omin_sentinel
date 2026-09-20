import hashlib
import json
import math
import struct
import unittest
from pathlib import Path

from yaw_capture_protocol import frame
from yaw_slow_cli import parse_args
from yaw_speed_trace_protocol import (BUILD, HOST_DECODER_REVISION, NOMINAL_RECORDS, PHASE_OPS,
                                      PLAN_SHA256, Decoder, request,
                                      speed_reference_matches, speed_wave)


ROOT = Path(__file__).resolve().parent
PLAN = ROOT/'mpc/speed_reference_excitation_plan.json'


class SpeedReferenceTraceTests(unittest.TestCase):
    def test_frozen_plan_hash_and_identity(self):
        self.assertEqual(hashlib.sha256(PLAN.read_bytes()).hexdigest().upper(), PLAN_SHA256)
        plan = json.loads(PLAN.read_text(encoding='utf-8'))
        self.assertEqual(plan['firmware_build'], '0x59490702')
        self.assertEqual(plan['nominal_records'], NOMINAL_RECORDS)

    def test_requests_and_cli(self):
        for label, op in PHASE_OPS.items():
            packet = request(op, 7, 5, 0)
            self.assertEqual(struct.unpack('<IHBBI', packet[2:14])[2:4], (op, 5))
            parsed = parse_args([
                '--port', 'x', '--action', 'dual', '--phase-set', label,
                '--confirm', 'SPEED_REFERENCE_FIXED_CHASSIS_CLEAR_YAW', '--output', 'x'],
                phase_sets=tuple(PHASE_OPS),
                dual_confirm='SPEED_REFERENCE_FIXED_CHASSIS_CLEAR_YAW')
            self.assertEqual(parsed.phase_set, label)
        with self.assertRaises(ValueError):
            request(PHASE_OPS['S1'], 1, 4, 0)

    def test_old_build_is_rejected(self):
        old_info = frame(0x3b, struct.pack('<IIHH', 0x59490601, 0x31444959, 1, 152))
        with self.assertRaises(ValueError):
            Decoder().feed(old_info)

    def test_waveforms_have_bounds_signs_and_near_zero_integral(self):
        dt = .004
        for phase_set in (0, 1):
            for axis, limit in ((0, 30.), (1, 60.)):
                values = [speed_wave(ms, axis, phase_set) for ms in range(0, 28001, 4)]
                self.assertLessEqual(max(map(abs, values)), limit)
                self.assertGreater(max(map(abs, values)), .97*limit)
                self.assertGreater(sum(v > .5 for v in values), 1000)
                self.assertGreater(sum(v < -.5 for v in values), 1000)
                self.assertLess(abs(sum(values)*dt), .02)
                self.assertTrue(all(math.isfinite(v) for v in values))

    def test_reference_check_accepts_one_ms_control_to_record_alignment(self):
        elapsed_ms = 6057
        previous_ms = tuple(round(speed_wave(3056, axis, 0), 2) for axis in (0, 1))
        self.assertTrue(speed_reference_matches(elapsed_ms, previous_ms, 0))
        self.assertFalse(speed_reference_matches(elapsed_ms,
                                                  (previous_ms[0], previous_ms[1]+.10), 0))

    def test_native_s1_fixture_decodes_end_to_end(self):
        path = ROOT.parent/'NoMachineTemp/yaw-slow-native-tests/can-trace-speed-s1.bin'
        if not path.exists():
            self.skipTest('run native tests first')
        firmware_header = (ROOT.parent/
                           'AGVSentinel_v10090/AGVSentinel_gimbal/Inc/Modules/'
                           'module_yaw_ident_wire.h')
        if f'#define YAW_IDENT_BUILD 0x{BUILD:08X}UL' not in firmware_header.read_text():
            self.skipTest('native fixture generator currently targets a different build')
        decoder = Decoder(9, (5, 0, 5, 0.))
        raw = path.read_bytes()
        for index in range(0, len(raw), 137):
            decoder.feed(raw[index:index+137])
        report = decoder.report()
        self.assertTrue(decoder.complete)
        self.assertEqual(report['quality_issues'], [])
        self.assertEqual(len(decoder.rows), NOMINAL_RECORDS)
        self.assertEqual(decoder.profile['build'], BUILD)
        self.assertEqual(report['speed_reference_units'], 'centidegrees_per_second')
        self.assertEqual(report['host_decoder_revision'], HOST_DECODER_REVISION)


if __name__ == '__main__':
    unittest.main()
