import binascii
import io
import struct
import unittest
from itertools import count
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np

from yaw_capture_protocol import frame
from yaw_identification_protocol import META, RECORD, VALUES
from yaw_slow_cli import parse_args, run
from yaw_slow_protocol import PROFILE, BENCH, KEEPALIVE, PROBE, RECEIPT
from yaw_dual_trace_protocol import (BUILD, DUAL_A, DUAL_B, DUAL_KNOTS, Decoder,
                                     FIELDS, STREAM_VERSION, TRACE_FIELDS,
                                     capture_timing_summary, request)
from yaw_can_trace_protocol import (AXIS, AXIS_FIELDS, BUILD as V3_BUILD,
                                    Decoder as V3Decoder)

sys.path.insert(0, str(Path(__file__).resolve().parent/'mpc'))
from preview_dual_reference_excitation import DEFAULT_PLAN, reference, time_grid


def group(cmd, seq, raw):
    return b''.join(frame(cmd, struct.pack('<HBB', seq, i, STREAM_VERSION)+raw[i*8:i*8+8])
                    for i in range(len(raw)//8))


def status(trial_id=0, count_value=0, phase=0, reason=0, flags=0, extra=0):
    return frame(0x37, struct.pack('<IH6B', trial_id, count_value, phase, reason, flags, 3, 1, extra))


def info():
    return frame(0x3b, struct.pack('<IIHH', BUILD, 0x31444959, 1, 152))


def ack(trial_id, op, phase=2):
    return frame(0x38, struct.pack('<IBBBBI', trial_id, op, 1, phase, 0, 100))


def profile(profile_id=3, phase_set=0):
    knots = DUAL_KNOTS if profile_id == 3 else (0, 2000, 4000, 7000, 11000, 14000, 16000, 20000)
    peaks = (3., 2.) if profile_id == 3 else (15., 10.)
    return PROFILE.pack(1, BUILD, 460800, 5001, 4, 320, 20000, *knots, *peaks,
                        25., 20., 20., 10., profile_id, phase_set, STREAM_VERSION, 0)


def metadata(count_value=0, phase=2, profile_id=3, reason=0):
    c = dict.fromkeys(VALUES, 0.)
    c.update(big_effort_limit=30., small_effort_limit=6.)
    axis = 3 if profile_id == 3 else 0
    return META.pack(1, 100, BUILD, count_value, 4, phase, reason, axis, STREAM_VERSION, 1,
                     *[c[k] for k in VALUES])


def base_row(index, big_ref=0, small_ref=0, phase=None):
    if phase is None:
        phase = 2 if index < 500 else 3 if index < 4000 else 4 if index < 5000 else 5
    return RECORD.pack(100+4*index, *([0.]*6), 8170, 6816, big_ref, small_ref,
                       0, 0, big_ref+small_ref, 29 if phase != 5 else 3, 0, 0, 0, phase)


def trace(index, big_ref=0, small_ref=0, phase=None):
    now = index*4000
    r = dict.fromkeys(AXIS_FIELDS, 0)
    r.update(sequence=index+1, attempt_us=now, feedback_us=now, complete_sequence=index+1,
             complete_us=now, known_us=4000 if index else 0, current=-300,
             feedback_rpm=-2, encoder=8191, trace_flags=3, attempts=1, queued=1, completed=1)
    axis = AXIS.pack(*(r[k] for k in AXIS_FIELDS))
    payload = base_row(index, big_ref, small_ref, phase)+struct.pack('<II', now, 4000 if index else 0)+axis*2
    packet = b'\xff\x3f'+struct.pack('<IH', 1, index+1)+payload
    return packet+struct.pack('<H', binascii.crc_hqx(packet, 0xffff))+b'\r'


def fixture():
    header = group(0x3e, 0, profile())+group(0x39, 0, metadata())
    rows = [trace(i, 100 if 500 <= i < 4000 else 0,
                  -40 if 500 <= i < 4000 else 0) for i in range(5001)]
    return header+b''.join(rows[:-1])+group(0x39, 1, metadata(5001, 5))+rows[-1]


class DualProtocolTests(unittest.TestCase):
    def test_full_duration_timing_accepts_bounded_nominal_count_shortfall(self):
        profile_data = {'samples': 5001, 'duration_ms': 20000}
        initial = {'start_ms': 100}
        metadata_data = {'count': 4998, 'phase': 5, 'reason': 0}
        rows = [
            {'tick_ms': 100, 'trace_us': 409, 'interval_us': 409, 'phase': 2},
            {'tick_ms': 5100, 'trace_us': 5000409, 'interval_us': 5000000, 'phase': 3},
            {'tick_ms': 10100, 'trace_us': 10000409, 'interval_us': 5000000, 'phase': 3},
            {'tick_ms': 15100, 'trace_us': 15000409, 'interval_us': 5000000, 'phase': 3},
            {'tick_ms': 20100, 'trace_us': 20000305, 'interval_us': 4999896, 'phase': 5},
        ]
        # Replace the compressed illustrative intervals with a realistic
        # bounded-cadence sequence while retaining the measured endpoints.
        rows = [dict(tick_ms=100+round(i*20000/4997),
                     trace_us=409+round(i*(20000305-409)/4997),
                     interval_us=0, phase=5 if i == 4997 else 3)
                for i in range(4998)]
        rows[0]['phase'] = 2
        rows[0]['interval_us'] = rows[0]['trace_us']
        for index in range(1, len(rows)):
            rows[index]['interval_us'] = rows[index]['trace_us']-rows[index-1]['trace_us']
        summary = capture_timing_summary(rows, initial, metadata_data, profile_data)
        self.assertTrue(summary['full_duration_timing_accepted'])
        self.assertEqual(summary['record_shortfall'], 3)
        self.assertTrue(summary['resampling_required'])

    def test_timing_does_not_accept_short_or_overlong_capture(self):
        profile_data = {'samples': 5001, 'duration_ms': 20000}
        initial = {'start_ms': 100}
        for count_value, final_us, bad_interval in (
                (4995, 20000305, False), (4998, 19990000, False),
                (4998, 20000305, True)):
            rows = [dict(tick_ms=100+round(i*20000/(count_value-1)),
                         trace_us=409+round(i*(final_us-409)/(count_value-1)),
                         interval_us=0, phase=5 if i == count_value-1 else 3)
                    for i in range(count_value)]
            rows[0]['phase'] = 2
            rows[0]['interval_us'] = rows[0]['trace_us']
            for index in range(1, len(rows)):
                rows[index]['interval_us'] = rows[index]['trace_us']-rows[index-1]['trace_us']
            if bad_interval:
                rows[count_value//2]['interval_us'] = 10001
            metadata_data = {'count': count_value, 'phase': 5, 'reason': 0}
            summary = capture_timing_summary(rows, initial, metadata_data, profile_data)
            self.assertFalse(summary['full_duration_timing_accepted'])

    def test_full_dual_stream_has_independent_references(self):
        decoder = Decoder(1, (3, 0, 3, 0.))
        raw = fixture()
        for i in range(0, len(raw), 131):
            decoder.feed(raw[i:i+131])
        self.assertTrue(decoder.complete)
        self.assertEqual(decoder.report()['quality_issues'], [])
        self.assertEqual(decoder.report()['trace_version'], 4)
        self.assertEqual(decoder.rows[500]['big_reference_offset_cdeg'], 100)
        self.assertEqual(decoder.rows[500]['small_heading_reference_offset_cdeg'], -40)
        self.assertEqual(decoder.rows[500]['big_feedback_rpm'], -2)
        self.assertEqual(set(decoder.rows[500]), set(TRACE_FIELDS))
        self.assertEqual(len(TRACE_FIELDS), len(set(TRACE_FIELDS)))

    def test_rejects_reference_sum_and_wrong_profile(self):
        decoder = Decoder(1); decoder.feed(group(0x3e, 0, profile())+group(0x39, 0, metadata()))
        bad = bytearray(trace(0)); struct.pack_into('<h', bad, 8+40, 1)
        struct.pack_into('<H', bad, len(bad)-3, binascii.crc_hqx(bad[:-3], 0xffff))
        with self.assertRaisesRegex(ValueError, 'sum'):
            decoder.feed(bad)
        bad_profile = bytearray(profile()); struct.pack_into('<f', bad_profile, 44, 4.)
        with self.assertRaises(ValueError):
            Decoder(1).feed(group(0x3e, 0, bad_profile))

    def test_dual_request_and_cli_confirmation(self):
        self.assertEqual(struct.unpack('<IHBBI', request(DUAL_A, 7, 3, 0)[2:14])[2:4], (DUAL_A, 3))
        self.assertEqual(struct.unpack('<IHBBI', request(DUAL_B, 8, 3, 0)[2:14])[2:4], (DUAL_B, 3))
        for args in ((DUAL_A, 0, 3, 0), (DUAL_A, 1, 2, 0), (DUAL_B, 1, 3, 1)):
            with self.assertRaises(ValueError): request(*args)
        parsed = parse_args(['--port', 'x', '--action', 'dual', '--phase-set', 'A',
                             '--confirm', 'DUAL_REFERENCE_FIXED_CHASSIS_CLEAR_YAW', '--output', 'x'])
        self.assertEqual(parsed.phase_set, 'A')
        with self.assertRaises(SystemExit):
            parse_args(['--port', 'x', '--action', 'dual', '--phase-set', 'B', '--output', 'x'])

    def test_v3_and_v4_decoders_reject_each_others_firmware(self):
        v3_info = frame(0x3b, struct.pack('<IIHH', V3_BUILD, 0x31444959, 1, 152))
        with self.assertRaises(ValueError):
            V3Decoder().feed(info())
        with self.assertRaises(ValueError):
            Decoder().feed(v3_info)

    def test_real_uart_chunk_boundaries_do_not_spoil_complete_v4_trial(self):
        prefix = status()[2:]
        decoder = Decoder(1, (3, 0, 3, 0.))
        decoder.feed(prefix+fixture()+b'\xff\x37')
        report = decoder.report()
        self.assertTrue(report['download_complete'])
        self.assertEqual(report['quality_issues'], [])
        self.assertEqual(report['pre_sync_discarded_wire_bytes'], 14)
        self.assertTrue(report['buffered_wire_is_partial_status_after_complete'])

    def test_aborted_terminal_preserves_reference_but_has_zero_commands(self):
        decoder = Decoder(1, (3, 0, 3, 0.))
        raw = (group(0x3e, 0, profile())+group(0x39, 0, metadata())+
               trace(0)+group(0x39, 1, metadata(2, 6, reason=9))+
               trace(1, 37, -22, phase=6))
        decoder.feed(raw)
        report = decoder.report()
        self.assertTrue(report['download_complete'])
        self.assertEqual(len(decoder.rows), 2)
        self.assertEqual(decoder.rows[-1]['offset_cdeg'], 15)
        self.assertIn('trial_not_completed', report['quality_issues'])
        self.assertEqual(report['reason_name'], 'control_failed')

    def test_status_decodes_latched_big_controller_stop_reason(self):
        decoder = Decoder()
        decoder.feed(status(trial_id=2, count_value=2008, phase=6,
                            reason=12, flags=249, extra=(9 << 4) | 7))
        report = decoder.report()
        self.assertEqual(report['big_stop_reason_code'], 9)
        self.assertEqual(report['big_stop_reason_name'], 'angle_pid_invalid')
        self.assertTrue(report['firmware_stream_quality_latched'])

    def test_production_c_dual_fixture(self):
        path = Path(__file__).resolve().parents[1]/'NoMachineTemp/yaw-slow-native-tests/can-trace-dual-v4.bin'
        if not path.exists():
            self.skipTest('run native tests first')
        decoder = Decoder(4, (3, 0, 3, 0.))
        raw = path.read_bytes()
        for i in range(0, len(raw), 137):
            decoder.feed(raw[i:i+137])
        self.assertTrue(decoder.complete)
        self.assertEqual(decoder.report()['quality_issues'], [])
        self.assertTrue(any(r['big_reference_offset_cdeg'] > 0 for r in decoder.rows))
        self.assertTrue(any(r['big_reference_offset_cdeg'] < 0 for r in decoder.rows))
        self.assertTrue(any(r['small_heading_reference_offset_cdeg'] > 0 for r in decoder.rows))
        self.assertTrue(any(r['small_heading_reference_offset_cdeg'] < 0 for r in decoder.rows))
        plan = __import__('json').loads(DEFAULT_PLAN.read_text(encoding='utf-8'))
        expected = reference(plan, 'A', time_grid(plan))*100
        recorded = np.asarray([[r['big_reference_offset_cdeg'], r['small_heading_reference_offset_cdeg']]
                               for r in decoder.rows])
        self.assertLessEqual(np.max(np.abs(recorded-expected)), 1.0)


class FakeDualSerial:
    def __init__(self):
        self.queue = [info()+status()]
        self.sent, self.closed = [], False

    def factory(self, port, baud):
        self.assertions = (port, baud)
        return self

    def send(self, data):
        trial_id, value, op, axis, _ = struct.unpack('<IHBBI', data[2:14])
        self.sent.append((op, trial_id, value, axis))
        if op == DUAL_A:
            self.queue.append(ack(trial_id, op))
            raw = fixture()
            self.queue += [raw[i:i+4096] for i in range(0, len(raw), 4096)]
        elif op == RECEIPT and value == 5001:
            self.queue.append(status(1, 5001, 5, 0, 249, 6))

    def read(self):
        return self.queue.pop(0) if self.queue else b''

    def close(self): self.closed = True


class DualRunnerTests(unittest.TestCase):
    def test_dual_a_runs_without_release(self):
        port = FakeDualSerial(); ticks = count()
        args = SimpleNamespace(port='fake', action='dual', trial_id=None, axis=None,
                               amplitude_deg=None, reverse=False, phase_set='A')
        result = run(args, Decoder(), io.BytesIO(), io.StringIO(), port.factory,
                     lambda: next(ticks)*.001)
        self.assertEqual(result['trial_id'], 1)
        self.assertTrue(port.closed)
        self.assertIn((DUAL_A, 1, 0, 3), port.sent)
        self.assertFalse(any(op in (BENCH, 5) for op, *_ in port.sent))


if __name__ == '__main__':
    unittest.main()
