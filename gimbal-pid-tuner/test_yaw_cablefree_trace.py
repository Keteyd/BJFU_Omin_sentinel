import hashlib
import json
import struct
import sys
import unittest
from pathlib import Path

import numpy as np

from yaw_capture_protocol import frame
from yaw_slow_cli import parse_args
from yaw_cablefree_trace_protocol import (BUILD, CD_E, Decoder,
                                          MAXIMUM_RECORD_SHORTFALL,
                                          NOMINAL_RECORDS, PHASE_OPS,
                                          PHASE_VALUES, PLAN_SHA256, request)

sys.path.insert(0, str(Path(__file__).resolve().parent/'mpc'))
from preview_cable_free_ladder import DEFAULT_PLAN
from preview_dual_reference_cde import (DEFAULT_PLAN as SOURCE_PLAN, reference,
                                        time_grid)


class CableFreeTraceProtocolTests(unittest.TestCase):
    def test_embedded_plan_hash_matches_frozen_json(self):
        self.assertEqual(hashlib.sha256(DEFAULT_PLAN.read_bytes()).hexdigest().upper(),
                         PLAN_SHA256)

    def test_requests_and_cli_expose_only_the_three_ladder_phases(self):
        for label, op in PHASE_OPS.items():
            packet = request(op, 7, 4, 0)
            self.assertEqual(struct.unpack('<IHBBI', packet[2:14])[2:4], (op, 4))
            parsed = parse_args([
                '--port', 'x', '--action', 'dual', '--phase-set', label,
                '--confirm', 'CABLE_FREE_FIXED_CHASSIS_CLEAR_YAW', '--output', 'x',
            ], phase_sets=tuple(PHASE_OPS),
               dual_confirm='CABLE_FREE_FIXED_CHASSIS_CLEAR_YAW')
            self.assertEqual(parsed.phase_set, label)
        with self.assertRaises(SystemExit):
            parse_args([
                '--port', 'x', '--action', 'dual', '--phase-set', 'E',
                '--confirm', 'CABLE_FREE_FIXED_CHASSIS_CLEAR_YAW', '--output', 'x',
            ], phase_sets=tuple(PHASE_OPS),
               dual_confirm='CABLE_FREE_FIXED_CHASSIS_CLEAR_YAW')

    def test_rejects_previous_firmware_build(self):
        old_info = frame(0x3b, struct.pack('<IIHH', 0x59490503, 0x31444959, 1, 152))
        with self.assertRaises(ValueError):
            Decoder().feed(old_info)

    def test_native_f5_fixture_matches_scaled_e_point_by_point(self):
        path = Path(__file__).resolve().parents[1]/'NoMachineTemp/yaw-cablefree-native-tests/can-trace-f5.bin'
        if not path.exists():
            self.skipTest('run native tests first')
        decoder = Decoder(8, (4, PHASE_VALUES['F5'], 4, 0.))
        raw = path.read_bytes()
        for index in range(0, len(raw), 137):
            decoder.feed(raw[index:index+137])
        report = decoder.report()
        self.assertTrue(decoder.complete)
        self.assertEqual(report['quality_issues'], [])
        self.assertEqual(decoder.profile['build'], BUILD)
        self.assertEqual(decoder.profile['big_peak'], 15.)
        self.assertEqual(decoder.profile['small_peak'], 10.)
        self.assertEqual(len(decoder.rows), NOMINAL_RECORDS)
        self.assertEqual(report['capture_timing']['shortfall_limit'],
                         MAXIMUM_RECORD_SHORTFALL)
        self.assertEqual(report['ladder_phase_name'], 'F5')

        source = json.loads(SOURCE_PLAN.read_text(encoding='utf-8'))
        expected = reference(source, 'E', time_grid(source))*500
        recorded = np.asarray([
            [row['big_reference_offset_cdeg'], row['small_heading_reference_offset_cdeg']]
            for row in decoder.rows
        ])
        self.assertLessEqual(np.max(np.abs(recorded-expected)), 1.0)


if __name__ == '__main__':
    unittest.main()
