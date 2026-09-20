import struct
import unittest
import fcntl
import os
import termios
from unittest.mock import patch

from big_yaw_tune_cli import TuneTransaction, decode_config, encode_values, release_serial, full_values, FullConfigReader, NAMES, write_transaction
from pitch_tune_cli import extract_frames, make_frame


def reply(values=(2600, 600, 4000, 300), request=0, status=0, flags=0x93):
    return make_frame(0x2E, struct.pack("<5H2B", *values, request, status, flags))


class BigYawTests(unittest.TestCase):
    def test_transaction_frames_are_paced_and_never_retried(self):
        frames = b"".join(make_frame(0x30, struct.pack("<HBBff", 1, part, 0xC3, 0, 0))
                          for part in range(4))
        events = []
        with patch("big_yaw_tune_cli.time.sleep", side_effect=lambda t: events.append(t)), \
             patch("big_yaw_tune_cli.write_frame", side_effect=lambda fd, f: events.append(f)):
            write_transaction(99, frames)
        self.assertEqual(events, [value for i in range(4) for value in (.020, frames[i*16:i*16+16])])
        with patch("big_yaw_tune_cli.time.sleep"), \
             patch("big_yaw_tune_cli.write_frame", side_effect=OSError("lost")) as writer:
            with self.assertRaises(OSError): write_transaction(99, frames)
            self.assertEqual(writer.call_count, 1)

    def full_replies(self, values, request=0, flags=0x58):
        return [make_frame(0x31, struct.pack("<HBBff", request, part, flags,
                                           *values[part*2:part*2+2])) for part in range(4)]

    def test_full_float_pid_transaction_and_readback(self):
        initial=(4,.7,8,.03,0,0,0,0)
        requested=dict(zip(NAMES,(10000,20000,30,.5,.01,.1,.02,.2)))
        tx=TuneTransaction(requested)
        outgoing=None
        for frame in self.full_replies(initial): outgoing=tx.receive(frame)
        frames=extract_frames(bytearray(outgoing))
        self.assertEqual(len(frames),4)
        for i, frame in enumerate(frames):
            self.assertEqual(frame[1],0x30)
            self.assertEqual(struct.unpack_from("<HBB",frame,2),(tx.request_id,i,0xC3))
            self.assertEqual(struct.unpack_from("<ff",frame,6),full_values(requested)[i*2:i*2+2])
        self.assertFalse(tx.done)
        for frame in self.full_replies(full_values(requested),tx.request_id,0x59): tx.receive(frame)
        self.assertTrue(tx.done)
        self.assertTrue(tx.result['full_pid'])

    def test_full_reader_rejects_mixed_missing_unsafe_and_invalid(self):
        values=(4,.7,8,.03,0,0,0,0)
        frames=self.full_replies(values,123)
        reader=FullConfigReader()
        self.assertIsNone(reader.receive(frames[0]))
        self.assertIsNone(reader.receive(frames[2]))
        self.assertIsNone(reader.receive(frames[3]))
        self.assertIsNone(decode_config(reply(flags=0xF3)))
        for flags in (0x50,0x48,0x78):
            tx=TuneTransaction({'angle_ki':.01})
            with self.assertRaises(ValueError):
                for frame in self.full_replies(values,flags=flags): tx.receive(frame)
        for bad in (float('nan'),float('inf'),-1,1e40,1e-50):
            v=dict(zip(NAMES,values));v['angle_ki']=bad
            with self.assertRaises(ValueError): full_values(v)
        tx=TuneTransaction({'angle_ki':.01})
        with self.assertRaises(ValueError): tx.receive(reply())

    def test_full_effort_range_requires_firmware_capability(self):
        self.assertEqual(decode_config(reply())["effort_limit_max"], 8)
        with self.assertRaises(ValueError):
            TuneTransaction({"effort_limit": 12}).receive(reply())
        for effort in (12, 30):
            tx = TuneTransaction({"effort_limit": effort})
            packet = tx.receive(reply(flags=0xB3))
            values = struct.unpack_from("<4H", packet, 2)
            self.assertEqual(values[2], effort * 1000)
            tx.receive(reply(values, tx.request_id, 1, flags=0xB3))
            self.assertTrue(tx.done)
            self.assertEqual(tx.result["effort_limit"], effort)
        for value in (-1, 30.001, float("inf"), float("nan")):
            with self.assertRaises(ValueError):
                encode_values((4, 0.7, value, .03))
        with self.assertRaises(ValueError):
            decode_config(reply((4000, 700, 12000, 300), flags=0x93))

    def test_exclusive_released_before_close(self):
        calls = []
        with patch("big_yaw_tune_cli.fcntl.ioctl", side_effect=lambda *args: calls.append(("ioctl", *args))), \
             patch("big_yaw_tune_cli.os.close", side_effect=lambda fd: calls.append(("close", fd))):
            release_serial(42, True)
        self.assertEqual(calls, [("ioctl", 42, termios.TIOCNXCL), ("close", 42)])

    def test_release_failure_still_closes(self):
        with patch("big_yaw_tune_cli.fcntl.ioctl", side_effect=OSError("disconnected")), \
             patch("big_yaw_tune_cli.os.close") as close:
            with self.assertRaises(OSError):
                release_serial(42, True)
            close.assert_called_once_with(42)

    def test_no_unlock_without_ownership(self):
        with patch("big_yaw_tune_cli.fcntl.ioctl") as ioctl, \
             patch("big_yaw_tune_cli.os.close") as close:
            release_serial(42, False)
            ioctl.assert_not_called()
            close.assert_called_once_with(42)

    def test_pty_reopens_after_release(self):
        master, slave = os.openpty()
        reopened = None
        try:
            name = os.ttyname(slave)
            fcntl.ioctl(slave, termios.TIOCEXCL)
            closing, slave = slave, None
            release_serial(closing, True)
            reopened = os.open(name, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        finally:
            if slave is not None:
                os.close(slave)
            if reopened is not None:
                os.close(reopened)
            os.close(master)

    def test_read_only_never_writes(self):
        tx = TuneTransaction()
        self.assertIsNone(tx.receive(reply()))
        self.assertTrue(tx.done)
        self.assertEqual(tx.result["speed_kp"], 0.6)

    def test_partial_update_preserves_other_values_and_requires_ack(self):
        tx = TuneTransaction({"speed_kp": 0.7})
        packet = tx.receive(reply())
        self.assertEqual(packet[1], 0x2D)
        fields = struct.unpack_from("<5H2B", packet, 2)
        self.assertEqual(fields[:4], (2600, 700, 4000, 300))
        self.assertEqual(fields[-2:], (0xB7, 1))
        self.assertFalse(tx.done)
        self.assertIsNone(tx.receive(reply()))
        tx.receive(reply(fields[:4], tx.request_id, 1))
        self.assertTrue(tx.done)

    def test_active_offline_and_old_protocol_refuse_write(self):
        for flags in (0x90, 0x91, 0x92, 0x97, 0):
            with self.assertRaises(ValueError):
                TuneTransaction({"speed_kp": 0.7}).receive(reply(flags=flags))

    def test_reject_and_mismatch_never_report_success(self):
        for status in (0, 2, 3, 4):
            tx = TuneTransaction({"speed_kp": 0.7})
            tx.receive(reply())
            with self.assertRaises(ValueError):
                tx.receive(reply(request=tx.request_id, status=status))
            self.assertFalse(tx.done)
        tx = TuneTransaction({"speed_kp": 0.7})
        tx.receive(reply())
        with self.assertRaises(ValueError):
            tx.receive(reply(request=tx.request_id, status=1))

    def test_invalid_parameter(self):
        for value in (-1, 5.1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                encode_values((2.6, value, 4, 0.03))

    def test_crc_and_fragmentation(self):
        packet = reply()
        buffer = bytearray(packet[:7])
        self.assertEqual(extract_frames(buffer), [])
        buffer.extend(packet[7:])
        self.assertEqual(extract_frames(buffer), [packet])
        corrupt = bytearray(packet)
        corrupt[5] ^= 1
        self.assertEqual(extract_frames(corrupt), [])
        self.assertEqual(decode_config(packet)["filter_tau_s"], 0.03)


if __name__ == "__main__":
    unittest.main()
