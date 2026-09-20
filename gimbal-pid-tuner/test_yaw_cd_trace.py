import struct
import sys
import unittest
import hashlib
from pathlib import Path

import numpy as np

from yaw_capture_protocol import frame
from yaw_slow_cli import parse_args
from yaw_slow_protocol import RECEIPT
from yaw_cd_trace_protocol import (BUILD, CD_C, CD_D, Decoder, NOMINAL_RECORDS,
                                   PLAN_SHA256, STREAM_VERSION, request)

sys.path.insert(0, str(Path(__file__).resolve().parent/'mpc'))
from preview_dual_reference_cd import DEFAULT_PLAN, reference, time_grid


class CDTraceProtocolTests(unittest.TestCase):
    def test_embedded_plan_hash_matches_frozen_json(self):
        self.assertEqual(
            hashlib.sha256(DEFAULT_PLAN.read_bytes()).hexdigest().upper(), PLAN_SHA256)

    def test_cd_requests_receipts_and_cli_phase_sets(self):
        for op, label in ((CD_C, 'C'), (CD_D, 'D')):
            packet = request(op, 7, 4, 0)
            self.assertEqual(struct.unpack('<IHBBI', packet[2:14])[2:4], (op, 4))
            parsed = parse_args([
                '--port', 'x', '--action', 'dual', '--phase-set', label,
                '--confirm', 'CD_REFERENCE_FIXED_CHASSIS_CLEAR_YAW', '--output', 'x',
            ], phase_sets=('C', 'D'), dual_confirm='CD_REFERENCE_FIXED_CHASSIS_CLEAR_YAW')
            self.assertEqual(parsed.phase_set, label)
        request(RECEIPT, 7, 0, NOMINAL_RECORDS)
        with self.assertRaises(ValueError):
            request(RECEIPT, 7, 0, NOMINAL_RECORDS + 1)
        with self.assertRaises(ValueError):
            request(CD_C, 7, 3, 0)
        with self.assertRaises(SystemExit):
            parse_args([
                '--port', 'x', '--action', 'dual', '--phase-set', 'A',
                '--confirm', 'CD_REFERENCE_FIXED_CHASSIS_CLEAR_YAW', '--output', 'x',
            ], phase_sets=('C', 'D'), dual_confirm='CD_REFERENCE_FIXED_CHASSIS_CLEAR_YAW')

    def test_cd_cli_excludes_incompatible_bench_and_single_axis_actions(self):
        actions = ('probe', 'dual', 'cancel', 'release')
        for action in ('bench', 'trial'):
            with self.assertRaises(SystemExit):
                parse_args(['--port', 'x', '--action', action, '--output', 'x'],
                           phase_sets=('C', 'D'),
                           dual_confirm='CD_REFERENCE_FIXED_CHASSIS_CLEAR_YAW',
                           actions=actions)

    def test_rejects_previous_firmware_build(self):
        old_info = frame(0x3b, struct.pack('<IIHH', 0x59490405, 0x31444959, 1, 152))
        with self.assertRaises(ValueError):
            Decoder().feed(old_info)

    def test_production_c_fixture_matches_frozen_preview_point_by_point(self):
        path = Path(__file__).resolve().parents[1]/'NoMachineTemp/yaw-slow-native-tests/can-trace-cd.bin'
        if not path.exists():
            self.skipTest('run native tests first')
        firmware_header = (Path(__file__).resolve().parents[1]/
                           'AGVSentinel_v10090/AGVSentinel_gimbal/Inc/Modules/'
                           'module_yaw_ident_wire.h')
        if f'#define YAW_IDENT_BUILD 0x{BUILD:08X}UL' not in firmware_header.read_text():
            self.skipTest('native fixture generator currently targets a different build')
        decoder = Decoder(6, (4, 0, 4, 0.))
        raw = path.read_bytes()
        for index in range(0, len(raw), 137):
            decoder.feed(raw[index:index+137])
        report = decoder.report()
        self.assertTrue(decoder.complete)
        self.assertEqual(report['quality_issues'], [])
        self.assertEqual(report['trace_version'], STREAM_VERSION)
        self.assertEqual(decoder.info, None)
        self.assertEqual(decoder.profile['build'], BUILD)
        self.assertEqual(len(decoder.rows), NOMINAL_RECORDS)
        self.assertFalse(report['reference_acceleration_runtime_abort'])
        self.assertEqual(report['offline_reference_acceleration_review_limit_dps2'], 200.)

        plan = __import__('json').loads(DEFAULT_PLAN.read_text(encoding='utf-8'))
        expected = reference(plan, 'C', time_grid(plan))*100
        recorded = np.asarray([
            [row['big_reference_offset_cdeg'], row['small_heading_reference_offset_cdeg']]
            for row in decoder.rows
        ])
        self.assertLessEqual(np.max(np.abs(recorded-expected)), 1.0)


if __name__ == '__main__':
    unittest.main()
