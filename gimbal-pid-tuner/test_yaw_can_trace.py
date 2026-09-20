import binascii
import csv
import io
import struct
import unittest
from itertools import count
from pathlib import Path
from types import SimpleNamespace

from yaw_capture_protocol import frame
from yaw_identification_protocol import META
from yaw_slow_protocol import Decoder as LegacyDecoder, PROFILE, BENCH, RECEIPT, PROBE, KEEPALIVE
from yaw_slow_cli import run
from yaw_can_trace_protocol import Decoder, BUILD, AXIS, AXIS_FIELDS, TRACE_FIELDS
from test_yaw_slow import profile as old_profile, metadata as old_metadata, row, status, ack, info as old_info


def group(cmd, seq, raw):
    return b''.join(frame(cmd, struct.pack('<HBB', seq, i, 3)+raw[i*8:i*8+8]) for i in range(len(raw)//8))


def metadata(count=0, phase=2):
    v = list(META.unpack(old_metadata(count, phase))); v[2], v[8] = BUILD, 3
    return group(0x39, int(phase >= 5), META.pack(*v))


def header():
    v = list(PROFILE.unpack(old_profile())); v[1], v[5], v[23] = BUILD, 320, 3
    return group(0x3e, 0, PROFILE.pack(*v))+metadata()


def info():
    return frame(0x3b, struct.pack('<IIHH', BUILD, 0x31444959, 1, 152))


def trace(index, **changes):
    now = index*4000
    r = dict.fromkeys(AXIS_FIELDS, 0)
    r.update(sequence=index+1, attempt_us=now, feedback_us=now, complete_sequence=index+1,
             complete_us=now, known_us=4000 if index else 0, current=-300, feedback_rpm=-2, encoder=8191,
             trace_flags=3, attempts=1, queued=1, completed=1)
    r.update(changes)
    axis = AXIS.pack(*(r[k] for k in AXIS_FIELDS))
    payload = row(index)+struct.pack('<II', now, 4000 if index else 0)+axis*2
    packet = b'\xff\x3f'+struct.pack('<IH', 1, index+1)+payload
    return packet+struct.pack('<H', binascii.crc_hqx(packet, 0xffff))+b'\r'


def fixture():
    return header()+b''.join(trace(i) for i in range(5000))+metadata(5001, 5)+trace(5000)


class TraceTests(unittest.TestCase):
    def test_profile_phase_uses_trace_clock_at_two_second_boundary(self):
        packets = [trace(i) for i in range(500)]
        packet = bytearray(trace(500))
        packet[55] = 2  # Firmware is still in baseline at trace_us=1,999,576.
        now, duration = 1999576, 3576
        struct.pack_into('<II', packet, 56, now, duration)
        values = dict.fromkeys(AXIS_FIELDS, 0)
        values.update(sequence=501, attempt_us=now, feedback_us=now,
                      complete_sequence=501, complete_us=now, known_us=duration,
                      current=-300, feedback_rpm=-2, encoder=8191,
                      trace_flags=3, attempts=1, queued=1, completed=1)
        axis = AXIS.pack(*(values[k] for k in AXIS_FIELDS))
        packet[64:112] = axis
        packet[112:160] = axis
        struct.pack_into('<H', packet, len(packet)-3,
                         binascii.crc_hqx(packet[:-3], 0xffff))
        d = Decoder(1)
        d.feed(header()+b''.join(packets)+packet)
        self.assertEqual(len(d.rows), 501)
        self.assertEqual(d.rows[-1]['phase'], 2)
        self.assertEqual(d.rows[-1]['trace_us'], now)

    def test_first_sample_may_follow_metadata_by_one_millisecond(self):
        packet = bytearray(trace(0))
        struct.pack_into('<I', packet, 8, 0xfffffff1)
        struct.pack_into('<II', packet, 56, 445, 445)
        struct.pack_into('<H', packet, len(packet)-3,
                         binascii.crc_hqx(packet[:-3], 0xffff))
        d = Decoder(1)
        d.feed(header()+packet)
        self.assertNotIn('irregular_sample_interval', d.report()['quality_issues'])
        self.assertNotIn('trace_clock_or_interval_invalid', d.report()['quality_issues'])

    def test_real_trace_clock_accepts_millisecond_quantization_jitter(self):
        def timed_trace(tick_ms, now_us):
            packet = bytearray(trace(1))
            struct.pack_into('<I', packet, 8, tick_ms)
            struct.pack_into('<II', packet, 56, now_us, now_us)
            values = dict.fromkeys(AXIS_FIELDS, 0)
            values.update(sequence=2, attempt_us=now_us, feedback_us=now_us,
                          complete_sequence=2, complete_us=now_us, known_us=now_us,
                          current=-300, feedback_rpm=-2, encoder=8191,
                          trace_flags=3, attempts=1, queued=1, completed=1)
            axis = AXIS.pack(*(values[k] for k in AXIS_FIELDS))
            packet[64:112] = axis
            packet[112:160] = axis
            struct.pack_into('<H', packet, len(packet)-3,
                             binascii.crc_hqx(packet[:-3], 0xffff))
            return bytes(packet)

        d = Decoder(1)
        d.feed(header()+trace(0)+timed_trace(0xfffffff5, 5000))
        self.assertNotIn('irregular_sample_interval', d.report()['quality_issues'])
        self.assertNotIn('trace_clock_or_interval_invalid', d.report()['quality_issues'])

        d = Decoder(1)
        d.feed(header()+trace(0)+timed_trace(0xfffffffa, 10001))
        self.assertIn('trace_clock_or_interval_invalid', d.report()['quality_issues'])

    def test_full_stream_and_raw_signed_current(self):
        raw = info()+fixture()
        d = Decoder(1)
        for i in range(0, len(raw), 127):
            d.feed(raw[i:i+127])
        self.assertTrue(d.complete)
        self.assertEqual(len(d.rows), 5001)
        self.assertEqual(d.report()['quality_issues'], [])
        self.assertEqual(d.rows[-1]['big_current'], -300)
        self.assertEqual(d.rows[-1]['small_feedback_rpm'], -2)
        self.assertEqual(d.rows[-1]['small_rpm'], 0)  # Original snapshot must survive.
        self.assertEqual(d.rows[-1]['small_feedback_age_us'], 0)
        self.assertEqual(set(d.rows[-1]), set(TRACE_FIELDS))
        self.assertEqual(len(TRACE_FIELDS), len(set(TRACE_FIELDS)))
        exported = io.StringIO()
        writer = csv.DictWriter(exported, fieldnames=TRACE_FIELDS)
        writer.writeheader(); writer.writerow(d.rows[-1])
        restored = next(csv.DictReader(io.StringIO(exported.getvalue())))
        self.assertEqual((restored['small_rpm'], restored['small_feedback_rpm']), ('0', '-2'))
        self.assertEqual(d.bytes_received, len(raw))
        self.assertEqual(d.bytes_consumed, len(raw))
        self.assertFalse(d.report()['hardware_takeover_allowed'])
        self.assertEqual(d.report()['can_summary']['big']['interval_counter_sums']['completed'], 5001)

    def test_production_c_fixtures(self):
        directory = Path(__file__).resolve().parents[1]/'NoMachineTemp/yaw-slow-native-tests'
        if not (directory/'can-trace-core.bin').exists():
            self.skipTest('run AGVSentinel_v10090/Tests/run_yaw_slow_tests.ps1 first')
        # The raw CAN recorder layout remains version-independent; wrap its
        # C fixture in a synthetic v3 header for the historical decoder.
        v = list(PROFILE.unpack(old_profile())); v[1], v[5], v[21], v[23] = BUILD, 320, 1, 3
        m = list(META.unpack(old_metadata())); m[1], m[2], m[7], m[8], m[10] = 100, BUILD, 1, 3, 1.
        d = Decoder(1)
        d.feed(group(0x3e, 0, PROFILE.pack(*v))+group(0x39, 0, META.pack(*m)))
        d.feed((directory/'can-trace-core.bin').read_bytes())
        r = d.rows[0]
        self.assertEqual((r['big_integral_raw_us'], r['big_known_us'], r['big_current'], r['big_pending']),
                         (300000, 900, -300, 1))
        self.assertAlmostEqual(r['big_mean_attempted_command_raw'], 1000/3)

    def test_bytewise_and_partial_tail(self):
        d = Decoder(1)
        for b in header()+trace(0):
            d.feed(bytes([b]))
        self.assertEqual(len(d.rows), 1)
        d.feed(metadata(1, 6)+trace(1)[:90])
        self.assertFalse(d.complete)
        self.assertIn('incomplete_stream', d.report()['quality_issues'])

    def test_uart_presync_and_partial_status_do_not_invalidate_complete_trial(self):
        # Opening a continuously used UART can start with the tail of an older
        # status frame; a final read can stop after the next status ff or ff/37 prefix.
        prefix = status()[2:]
        self.assertNotIn(255, prefix)
        for tail in (b'\xff', b'\xff\x37'):
            with self.subTest(tail=tail):
                d = Decoder(1)
                d.feed(prefix+fixture()+tail)
                report = d.report()
                self.assertTrue(d.complete)
                self.assertEqual(report['quality_issues'], [])
                self.assertEqual(report['pre_sync_discarded_wire_bytes'], 14)
                self.assertEqual(report['in_stream_discarded_wire_bytes'], 0)
                self.assertEqual(report['buffered_wire_bytes'], len(tail))
                self.assertTrue(report['buffered_wire_is_partial_status_after_complete'])

    def test_false_ff_candidate_before_first_valid_frame_is_resynchronized(self):
        malformed_tail = b'\xff'+b'\x00'*15
        malformed_crc = bytearray(status())
        malformed_crc[2] ^= 1
        for prefix in (malformed_tail, bytes(malformed_crc)):
            with self.subTest(prefix=prefix):
                d = Decoder()
                d.feed(prefix+info()+status())
                self.assertEqual(d.info['build'], BUILD)
                self.assertIsNotNone(d.status)
                self.assertEqual(d.pre_sync_discarded_bytes, 16)
                self.assertEqual(d.crc_errors, 0)

    def test_false_ff_candidate_after_valid_frame_remains_fatal(self):
        d = Decoder()
        d.feed(info())
        with self.assertRaisesRegex(ValueError, 'malformed legacy control frame'):
            d.feed(b'\xff'+b'\x00'*15)

    def test_discard_after_trial_stream_start_remains_a_quality_fault(self):
        d = Decoder(1)
        d.feed(header()+b'\x01'+trace(0))
        self.assertIn('discarded_wire_bytes', d.report()['quality_issues'])
        self.assertEqual(d.report()['in_stream_discarded_wire_bytes'], 1)

    def test_cross_version_rejection(self):
        for decoder, packet in ((Decoder(), old_info()), (LegacyDecoder(), info())):
            with self.assertRaises(ValueError): decoder.feed(packet)
        d = Decoder(1); d.feed(header())
        with self.assertRaises(ValueError): d.feed(frame(0x3d, b'\x00'*12))

    def test_corruption_sequence_and_trial_identity(self):
        for kind in ('crc', 'sequence', 'trial', 'tail'):
            d = Decoder(1); d.feed(header())
            packet = bytearray(trace(0))
            if kind == 'crc': packet[90] ^= 1
            elif kind == 'tail': packet[-1] = 0
            else:
                packet[2 if kind == 'trial' else 6] += 1
                packet[-3:-1] = struct.pack('<H', binascii.crc_hqx(packet[:-3], 0xffff))
            with self.subTest(kind=kind), self.assertRaises(ValueError): d.feed(packet)
            self.assertEqual(d.rows, [])

    def test_invalid_accounting_and_bench_commands(self):
        for changes in (dict(queued=0), dict(completed=0), dict(known_us=1), dict(minimum=1),
                        dict(attempt_us=1), dict(command=1, maximum=1)):
            d = Decoder(1); d.feed(header())
            with self.subTest(changes=changes), self.assertRaises(ValueError): d.feed(trace(0, **changes))
            self.assertEqual(d.rows, [])

    def test_report_status_fault_without_silent_drop(self):
        d = Decoder(1); d.feed(header())
        d.feed(trace(0, queued=0, failed=1, completed=0, complete_sequence=0, trace_flags=35))
        self.assertEqual(len(d.rows), 1)
        self.assertIn('big_CAN_status_or_counter_fault', d.report()['quality_issues'])

    def test_fake_serial_bench_and_no_automatic_release(self):
        class Port:
            def __init__(self):
                # Reproduce the observed CP2102 boundaries: open in the tail
                # of one status frame, then end the trial read at ff/37.
                self.queue = [status()[2:]+info()+status()]
                self.sent = []; self.started = self.closed = False
            def send(self, packet):
                _, value, op, _, _ = struct.unpack('<IHBBI', packet[2:14]); self.sent.append(op)
                if op == BENCH:
                    self.queue.append(ack(1, BENCH)); self.started = True
                if op == RECEIPT and value == 5001:
                    self.queue.append(status(1, 5001, 5, 253, 5)[2:])
            def read(self):
                if self.queue: return self.queue.pop(0)
                if self.started:
                    self.started = False
                    return fixture()+b'\xff\x37'
                return b''
            def close(self): self.closed = True
        port = Port(); tick = count()
        result = run(SimpleNamespace(port='fake', action='bench', trial_id=None, reverse=False),
                     Decoder(), io.BytesIO(), io.StringIO(), serial_factory=lambda *a, **k: port,
                     clock=lambda: next(tick)*.01)
        self.assertTrue(result['bench_passed'])
        self.assertTrue(port.closed)
        self.assertTrue(set(port.sent) <= {PROBE, KEEPALIVE, BENCH, RECEIPT})


if __name__ == '__main__':
    unittest.main()
