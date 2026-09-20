import hashlib
import json
import struct
import sys
import unittest
from pathlib import Path

import numpy as np

from yaw_capture_protocol import frame
from yaw_dual_trace_protocol import capture_timing_summary
from yaw_slow_cli import parse_args
from yaw_cde_trace_protocol import (BUILD, CD_E, Decoder, MAXIMUM_RECORD_SHORTFALL,
                                    NOMINAL_RECORDS, PLAN_SHA256, STREAM_VERSION,
                                    request)

sys.path.insert(0, str(Path(__file__).resolve().parent/'mpc'))
from preview_dual_reference_cde import DEFAULT_PLAN, reference, time_grid


class CDETraceProtocolTests(unittest.TestCase):
    def test_embedded_plan_hash_matches_frozen_json(self):
        self.assertEqual(
            hashlib.sha256(DEFAULT_PLAN.read_bytes()).hexdigest().upper(), PLAN_SHA256)

    def test_e_is_the_only_motion_action_exposed_by_this_host(self):
        packet = request(CD_E, 7, 4, 0)
        self.assertEqual(struct.unpack('<IHBBI', packet[2:14])[2:4], (CD_E, 4))
        parsed = parse_args([
            '--port', 'x', '--action', 'dual', '--phase-set', 'E',
            '--confirm', 'E_REFERENCE_FIXED_CHASSIS_CLEAR_YAW', '--output', 'x',
        ], phase_sets=('E',), dual_confirm='E_REFERENCE_FIXED_CHASSIS_CLEAR_YAW')
        self.assertEqual(parsed.phase_set, 'E')
        with self.assertRaises(ValueError):
            request(13, 7, 4, 0)
        with self.assertRaises(SystemExit):
            parse_args([
                '--port', 'x', '--action', 'dual', '--phase-set', 'D',
                '--confirm', 'E_REFERENCE_FIXED_CHASSIS_CLEAR_YAW', '--output', 'x',
            ], phase_sets=('E',), dual_confirm='E_REFERENCE_FIXED_CHASSIS_CLEAR_YAW')

    @staticmethod
    def _timing(count):
        first, last = 400, 36000000
        trace = [round(first+i*(last-first)/(count-1)) for i in range(count)]
        rows = []
        for index, value in enumerate(trace):
            rows.append({
                'trace_us': value,
                'interval_us': value if index == 0 else value-trace[index-1],
                'tick_ms': 1000+round(value/1000),
                'phase': 5 if index == count-1 else 3,
            })
        return capture_timing_summary(
            rows, {'start_ms': 1000}, {'count': count, 'phase': 5, 'reason': 0},
            {'samples': 9001, 'duration_ms': 36000},
            shortfall_limit=MAXIMUM_RECORD_SHORTFALL)

    def test_predeclared_half_percent_timing_gate(self):
        accepted = self._timing(NOMINAL_RECORDS-MAXIMUM_RECORD_SHORTFALL)
        rejected = self._timing(NOMINAL_RECORDS-MAXIMUM_RECORD_SHORTFALL-1)
        self.assertTrue(accepted['full_duration_timing_accepted'])
        self.assertEqual(accepted['shortfall_limit'], 45)
        self.assertFalse(rejected['full_duration_timing_accepted'])

    def test_zero_origin_trace_is_a_valid_full_duration_clock(self):
        count = NOMINAL_RECORDS
        rows = [{
            'trace_us': index*4000,
            'interval_us': 0 if index == 0 else 4000,
            'tick_ms': 1000+index*4,
            'phase': 5 if index == count-1 else 3,
        } for index in range(count)]
        timing = capture_timing_summary(
            rows, {'start_ms': 1000}, {'count': count, 'phase': 5, 'reason': 0},
            {'samples': count, 'duration_ms': 36000},
            shortfall_limit=MAXIMUM_RECORD_SHORTFALL)
        self.assertTrue(timing['full_duration_timing_accepted'])

    def test_rejects_previous_firmware_build(self):
        old_info = frame(0x3b, struct.pack('<IIHH', 0x59490502, 0x31444959, 1, 152))
        with self.assertRaises(ValueError):
            Decoder().feed(old_info)

    def test_production_e_fixture_matches_frozen_preview_point_by_point(self):
        path = Path(__file__).resolve().parents[1]/'NoMachineTemp/yaw-slow-native-tests/can-trace-e.bin'
        if not path.exists():
            self.skipTest('run native tests first')
        firmware_header = (Path(__file__).resolve().parents[1]/
                           'AGVSentinel_v10090/AGVSentinel_gimbal/Inc/Modules/'
                           'module_yaw_ident_wire.h')
        if f'#define YAW_IDENT_BUILD 0x{BUILD:08X}UL' not in firmware_header.read_text():
            self.skipTest('native fixture generator currently targets a different build')
        decoder = Decoder(8, (4, 2, 4, 0.))
        raw = path.read_bytes()
        for index in range(0, len(raw), 137):
            decoder.feed(raw[index:index+137])
        report = decoder.report()
        self.assertTrue(decoder.complete)
        self.assertEqual(report['quality_issues'], [])
        self.assertEqual(report['trace_version'], STREAM_VERSION)
        self.assertEqual(decoder.profile['build'], BUILD)
        self.assertEqual(decoder.profile['reverse'], 2)
        self.assertEqual(len(decoder.rows), NOMINAL_RECORDS)
        self.assertEqual(report['capture_timing']['shortfall_limit'], 45)

        plan = json.loads(DEFAULT_PLAN.read_text(encoding='utf-8'))
        expected = reference(plan, 'E', time_grid(plan))*100
        recorded = np.asarray([
            [row['big_reference_offset_cdeg'], row['small_heading_reference_offset_cdeg']]
            for row in decoder.rows
        ])
        self.assertLessEqual(np.max(np.abs(recorded-expected)), 1.0)


if __name__ == '__main__':
    unittest.main()
