import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import time
import unittest

from yaw_capture_protocol import Decoder, frame, heartbeat
from yaw_release_capture_cli import verify_burst
from yaw_release_trigger import ReleaseTrigger


class TriggerTests(unittest.TestCase):
    def setUp(self):
        self.trigger = ReleaseTrigger()
        self.index = 0

    def step(self, **changes):
        self.index += 1
        row = dict(valid=1, flags=207, big_command=0, small_command=0,
                   small_encoder_deg=299.53125, tick_ms=self.index * 25,
                   host_s=self.index * .025, segment=0, sequence=self.index, rc_yaw=0)
        row.update(changes)
        return self.trigger.feed(row)

    def ready(self):
        for _ in range(21):
            self.assertFalse(self.step())
        self.assertEqual(self.trigger.state, "ready")

    def active(self):
        self.ready()
        for _ in range(5):
            self.step(flags=222)
        self.assertEqual(self.trigger.state, "wait_excursion")

    def excursion(self):
        for _ in range(5):
            self.step(flags=222, rc_yaw=80)

    def test_zero_does_not_trigger(self):
        self.active()
        for _ in range(50):
            self.assertFalse(self.step(flags=222))

    def test_release_once(self):
        self.active()
        self.excursion()
        self.assertTrue(self.step(flags=222))
        self.assertFalse(self.step(flags=222))
        self.assertEqual(self.trigger.event["rc_yaw"], 0)

    def test_up_stick_is_not_excursion(self):
        self.ready()
        for _ in range(6):
            self.assertFalse(self.step(rc_yaw=100))
        self.assertFalse(self.step())

    def test_safe_is_not_release(self):
        self.active()
        self.excursion()
        with self.assertRaises(RuntimeError):
            self.step()

    def test_short_pulse_does_not_trigger(self):
        self.active()
        self.step(flags=222, rc_yaw=80)
        self.assertFalse(self.step(flags=222))

    def test_must_enable_centered(self):
        self.ready()
        with self.assertRaises(RuntimeError):
            self.step(flags=222, rc_yaw=80)

    def test_reject_unsafe_or_invalid(self):
        for change in (dict(valid=0), dict(flags=79), dict(flags=239),
                       dict(flags=463), dict(big_command=.1), dict(small_command=.1),
                       dict(small_encoder_deg=330), dict(small_encoder_deg=float("nan")),
                       dict(flags=222)):
            with self.subTest(change=change):
                self.setUp()
                with self.assertRaises(RuntimeError):
                    self.step(**change)

    def test_reject_discontinuity(self):
        for change in (dict(tick_ms=100), dict(sequence=8), dict(segment=1), dict(host_s=2)):
            with self.subTest(change=change):
                self.setUp()
                self.step()
                with self.assertRaises(RuntimeError):
                    self.step(**change)

    def test_negative_excursion(self):
        self.active()
        for _ in range(5):
            self.step(flags=222, rc_yaw=-80)
        self.assertTrue(self.step(flags=222))


def wire(version, sequence, tick, flags=207, stick=0):
    floats = [.002, 0, 0, 0, 0, 0, 354, 299.53125, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    if version == 2:
        floats += [.002, .001]
    data = struct.pack("<IIHHHhHH" + str(len(floats)) + "f", tick, sequence, 1, 1, 1,
                       stick, flags, 0, *floats)
    return b"".join(frame(0x32, struct.pack("<HBB", sequence, part, version)
                          + data[part * 8:part * 8 + 8]) for part in range(len(data) // 8))


class BurstValidationTests(unittest.TestCase):
    def test_alignment_and_quality(self):
        decoder = Decoder(2)
        rows = decoder.feed(b"".join(wire(2, i, 1000 + 2 * i) for i in range(1, 513)), 0)
        self.assertEqual(verify_burst(decoder, rows, {"tick_ms": 1000}), 2)
        with self.assertRaises(RuntimeError):
            verify_burst(decoder, rows, {"tick_ms": 500})
        with self.assertRaises(RuntimeError):
            verify_burst(decoder, rows[:-1], {"tick_ms": 1000})
        rows[4]["big_command"] = 1
        with self.assertRaises(RuntimeError):
            verify_burst(decoder, rows, {"tick_ms": 1000})


@unittest.skipUnless(sys.platform.startswith("linux"), "Linux pseudo-terminal integration")
class SerialIntegrationTests(unittest.TestCase):
    def test_monitor_only_handoff(self):
        import pty
        import select
        master, slave = pty.openpty()
        port = os.ttyname(slave)
        os.close(slave)
        os.set_blocking(master, False)
        process = None
        try:
            with tempfile.TemporaryDirectory() as tmp:
                output = Path(tmp) / "capture"
                process = subprocess.Popen([sys.executable,
                    str(Path(__file__).with_name("yaw_release_capture_cli.py")),
                    "--port", port, "--output", str(output), "--wait", "10"],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                incoming = bytearray()
                outgoing = bytearray()
                commands = []
                mode = 0
                seq = 0
                next_row = time.monotonic()
                deadline = next_row + 15
                while process.poll() is None and time.monotonic() < deadline:
                    readable, writable, _ = select.select([master], [master] if outgoing else [], [], .005)
                    if readable:
                        try:
                            incoming.extend(os.read(master, 4096))
                        except OSError:
                            time.sleep(.01)
                        while len(incoming) >= 16:
                            packet = bytes(incoming[:16])
                            del incoming[:16]
                            self.assertIn(packet, (heartbeat(1), heartbeat(2)))
                            commands.append(packet)
                            if packet == heartbeat(1) and mode == 0:
                                mode = 1
                            if packet == heartbeat(2) and mode == 1:
                                mode = 2
                                outgoing.extend(b"".join(wire(2, i, seq * 25 + 2 * i,
                                    flags=222) for i in range(1, 513)))
                    if mode == 1 and time.monotonic() >= next_row:
                        seq += 1
                        flags = 207 if seq <= 24 else 222
                        stick = 80 if 31 <= seq <= 36 else 0
                        outgoing.extend(wire(1, seq, seq * 25, flags, stick))
                        next_row = time.monotonic() + .025
                    if writable:
                        try:
                            count = os.write(master, outgoing[:4096])
                            del outgoing[:count]
                        except BlockingIOError:
                            pass
                stdout, stderr = process.communicate(timeout=2)
                self.assertEqual(process.returncode, 0, stdout + stderr)
                report = json.loads((output / "report.json").read_text())
                self.assertIsNone(report["error"])
                self.assertEqual(report["v2"]["statistics"]["records"], 512)
                self.assertEqual(report["release_to_first_burst_ms"], 2)
                self.assertIn(heartbeat(2), commands)
        finally:
            if process is not None and process.poll() is None:
                process.kill()
                process.communicate()
            os.close(master)


if __name__ == "__main__":
    unittest.main()
