import copy
import io
import struct
import unittest
from types import SimpleNamespace

from yaw_capture_protocol import frame
from yaw_identification_protocol import META, RECORD, VALUES
from yaw_slow_protocol import (ARM_SLOW, BENCH, BUILD, CANCEL, Decoder, KEEPALIVE,
                               KNOTS, MAGIC, PROBE, PROFILE, RECEIPT, RELEASE, request)
from yaw_slow_cli import parse_args, run


def group(cmd, seq, raw):
    return b''.join(frame(cmd, struct.pack('<HBB', seq, i, 2)+raw[i*8:i*8+8]) for i in range(len(raw)//8))


def status(trial_id=0, count=0, phase=0, flags=252, extra=0):
    return frame(0x37, struct.pack('<IH6B', trial_id, count, phase, 0, flags, 0, 1, extra))


def info(build=BUILD):
    return frame(0x3b, struct.pack('<IIHH', build, MAGIC, 1, 48))


def ack(trial_id, op, phase=2):
    return frame(0x38, struct.pack('<IBBBBI', trial_id, op, 1, phase, 0, 100))


def metadata(count=0, phase=2):
    c = dict.fromkeys(VALUES, 0.)
    c.update(big_effort_limit=30., small_effort_limit=6.)
    return META.pack(1, 0xfffffff0, BUILD, count, 4, phase, 0, 0, 2, 1, *[c[k] for k in VALUES])


def row(index, **changes):
    phase = 2 if index < 500 else 3 if index < 4000 else 4 if index < 5000 else 5
    values = dict(tick=(0xfffffff0+4*index) & 0xffffffff, command=0, flags=3, offset=0)
    values.update(changes)
    return RECORD.pack(values['tick'], *([0.]*6), 8170, 6816, 0, 0,
                       values['command'], 0, values['offset'], values['flags'], 0, 0, 0, phase)


def profile():
    return PROFILE.pack(1, BUILD, 460800, 5001, 4, 1001, 20000, *KNOTS,
                        15., 10., 25., 20., 20., 10., 2, 0, 2, 0)


def header():
    return group(0x3e, 0, profile())+group(0x39, 0, metadata())


def fixture():
    records = [group(0x3d, i+1, row(i)) for i in range(5001)]
    # Firmware can send final metadata before the terminal record is drained.
    return header()+b''.join(records[:-1])+group(0x39, 1, metadata(5001, 5))+records[-1]


class ProtocolTests(unittest.TestCase):
    def test_frozen_feedback_failure_report(self):
        d = Decoder(1)
        d.metadata = dict(count=1, phase=6, reason=3)
        d.rows = [dict(phase=6, flags=(1 << 12) | (1 << 13),
                       imu_age_ms=11, big_age_ms=1, small_age_ms=0)]
        f = d.report()['feedback_failure']
        self.assertEqual(f['causes'], ['ins_not_ready', 'big_motor_offline', 'imu_age_ms_expired'])
        self.assertEqual(f['imu_age_ms'], 11)

    def test_full_stream_chunking_and_wrap(self):
        decoder = Decoder(1, (2, 0, 0, 0.))
        raw = fixture()
        for i in range(0, len(raw), 127):
            decoder.feed(raw[i:i+127])
        self.assertTrue(decoder.complete)
        self.assertEqual(decoder.report()['quality_issues'], [])
        self.assertEqual(len(decoder.rows), 5001)
        self.assertFalse(decoder.report()['hardware_takeover_allowed'])

    def test_status_extension_not_version_high_byte(self):
        decoder = Decoder()
        decoder.feed(info()+status(1, 5001, 5, 253, 5))
        self.assertEqual(decoder.status['version'], 1)
        self.assertEqual(decoder.status['extra'], 5)

    def test_reject_wrong_firmware_profile_and_config(self):
        with self.assertRaises(ValueError):
            Decoder().feed(info(BUILD-1))
        p = bytearray(profile()); p[16] = 0
        with self.assertRaises(ValueError):
            Decoder(1).feed(group(0x3e, 0, p))
        d = Decoder(1); d.feed(header())
        m = bytearray(metadata(5001, 5)); m[-1] = 1
        with self.assertRaises(ValueError):
            d.feed(group(0x39, 1, m))

    def test_no_receipt_for_corrupt_missing_duplicate_or_motion_in_bench(self):
        for bad in ('crc', 'part', 'sequence', 'command', 'flags'):
            d = Decoder(1); d.feed(header())
            raw = bytearray(group(0x3d, 1, row(0)))
            if bad == 'crc': raw[14] ^= 1
            elif bad == 'part': del raw[16:32]
            elif bad == 'sequence': raw = group(0x3d, 2, row(0))
            elif bad == 'command': raw = group(0x3d, 1, row(0, command=1))
            else: raw = group(0x3d, 1, row(0, flags=29))
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                d.feed(raw)
            self.assertEqual(len(d.rows), 0)
        d = Decoder(1); d.feed(header()+group(0x3d, 1, row(0)))
        with self.assertRaises(ValueError):
            d.feed(group(0x3d, 1, row(0)))

    def test_early_abort_and_incomplete_are_not_success(self):
        d = Decoder(1)
        d.feed(group(0x3e, 0, profile()))
        d.feed(group(0x39, 0, metadata())[:32])
        d.feed(group(0x39, 1, metadata(0, 6)))
        self.assertTrue(d.complete)
        self.assertIn('trial_not_completed', d.report()['quality_issues'])
        d = Decoder(1); d.feed(header())
        self.assertFalse(d.complete)

    def test_irregular_sample_is_not_qualified(self):
        d = Decoder(1); d.feed(header())
        d.feed(group(0x3d, 1, row(0)))
        d.feed(group(0x3d, 2, row(1, tick=0xfffffff5)))
        self.assertIn('irregular_sample_interval', d.report()['quality_issues'])

    def test_no_accidental_motion_request(self):
        for invalid in ((BENCH, 1, 1, 0), (PROBE, 0, 0, 15), (RECEIPT, 1, 0, 5002), (ARM_SLOW, 1, 2, 1500)):
            with self.assertRaises(ValueError): request(*invalid)
        self.assertEqual(struct.unpack('<IHBBI', request(BENCH, 1)[2:14]), (1, 0, BENCH, 0, MAGIC))
        with self.assertRaises(SystemExit):
            parse_args(['--port', '/dev/null', '--action', 'bench', '--output', 'unused'])


class FakeSerial:
    def __init__(self, *, fail_after_arm=False, owned=False):
        self.queue = [info()+status(1 if owned else 0, 5001 if owned else 0, 5 if owned else 0,
                                    253 if owned else 252)]
        self.sent, self.closed, self.started = [], False, False
        self.fail_after_arm = fail_after_arm
        self.raw = fixture()

    def factory(self, port, baud):
        assert baud == 460800
        return self

    def send(self, data):
        trial_id, value, op, axis, magic = struct.unpack('<IHBBI', data[2:14])
        self.sent.append((op, trial_id, value, axis))
        if op == BENCH:
            self.started = True
            self.queue.append(ack(trial_id, op))
            if not self.fail_after_arm:
                self.queue += [self.raw[i:i+4096] for i in range(0, len(self.raw), 4096)]
        elif op == RECEIPT and value == 5001:
            self.queue.append(status(1, 5001, 5, 253, 5))
        elif op == RELEASE:
            self.queue.append(ack(trial_id, op, 0))

    def read(self):
        if self.fail_after_arm and self.started:
            raise OSError('test unplug')
        return self.queue.pop(0) if self.queue else b''

    def close(self): self.closed = True


class RunnerTests(unittest.TestCase):
    def args(self, action='bench'):
        return SimpleNamespace(port='fake', action=action, trial_id=None, axis=None,
                               amplitude_deg=None, reverse=False)

    def execute(self, port, args=None):
        now = [0.]
        def clock():
            now[0] += .0005
            return now[0]
        return run(args or self.args(), Decoder(), io.BytesIO(), io.StringIO(), port.factory, clock)

    def test_bench_only_and_no_auto_release(self):
        port = FakeSerial()
        result = self.execute(port)
        self.assertTrue(result['bench_passed'])
        self.assertTrue(port.closed)
        self.assertNotIn(ARM_SLOW, [x[0] for x in port.sent])
        self.assertNotIn(RELEASE, [x[0] for x in port.sent])
        self.assertNotIn(CANCEL, [x[0] for x in port.sent])
        receipts = [x[2] for x in port.sent if x[0] == RECEIPT]
        self.assertEqual(receipts[-1], 5001)
        self.assertEqual(receipts, sorted(set(receipts)))

    def test_disconnect_cancels_and_closes(self):
        port = FakeSerial(fail_after_arm=True)
        with self.assertRaises(OSError): self.execute(port)
        self.assertTrue(port.closed)
        self.assertIn((CANCEL, 1, 0, 0), port.sent)

    def test_probe_has_no_receipt_arm_cancel_release(self):
        port = FakeSerial(owned=True)
        self.execute(port, self.args('probe'))
        self.assertTrue(all(x[0] == PROBE for x in port.sent))

    def test_owned_bench_refused_and_release_idempotent(self):
        port = FakeSerial(owned=True)
        with self.assertRaises(RuntimeError): self.execute(port)
        self.assertTrue(all(x[0] == PROBE for x in port.sent))
        port = FakeSerial()
        result = self.execute(port, self.args('release'))
        self.assertTrue(result['already_unowned'])
        self.assertTrue(all(x[0] == PROBE for x in port.sent))


if __name__ == '__main__':
    unittest.main()
