import math
import struct
import unittest
import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile

from yaw_capture_protocol import Decoder, RECORD, PARTS, frame, heartbeat


def group(seq=1, tick=100, **changes):
    values = [tick, 50, 1, 1, 1, -3, 15, 0, .001] + [0.] * 16
    if changes.get("nan"):
        values[9] = math.nan
    if changes.get("stale"):
        values[2] = 500
    data = RECORD.pack(*values)
    return [frame(0x32, struct.pack("<HBB", seq, i, 1) + data[i*8:i*8+8])
            for i in range(PARTS)]


class CaptureTests(unittest.TestCase):
    def test_fragmentation(self):
        data = b"junk" + b"".join(group())
        for stride in range(1, len(data) + 1):
            decoder = Decoder()
            rows = []
            for offset in range(0, len(data), stride):
                rows += decoder.feed(data[offset:offset+stride], 0)
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["rc_yaw"], -3)
            self.assertEqual(rows[0]["valid"], 1)

    def test_corruption_and_recovery(self):
        packets = group()
        damaged = bytearray(packets[3]); damaged[7] ^= 1
        decoder = Decoder()
        rows = decoder.feed(b"".join(packets[:3] + [bytes(damaged)] + packets[4:] + group(2, 125)), 0)
        self.assertEqual([r["sequence"] for r in rows], [2])
        self.assertEqual(decoder.stats["crc_errors"], 1)

    def test_missing_reordered_duplicate_parts(self):
        packets = group()
        for broken in (packets[:4]+packets[5:], packets[:3]+[packets[4],packets[3]]+packets[5:],
                       packets[:3]+[packets[2]]+packets[3:]):
            decoder = Decoder()
            rows = decoder.feed(b"".join(broken + group(2, 125)), 0)
            self.assertEqual([r["sequence"] for r in rows], [2])

    def test_wrap_loss_and_reset(self):
        decoder = Decoder()
        rows = decoder.feed(b"".join(group(65535, 0xFFFFFFF0)+group(0,9)+group(2,59)), 0)
        self.assertEqual(rows[1]["delta_ms"], 25)
        self.assertEqual(decoder.stats["missing_groups"], 1)
        row = decoder.feed(b"".join(group(1, 1)), 0)[0]
        self.assertEqual(row["segment"], 1)

    def test_timeout_version_and_invalid(self):
        decoder = Decoder()
        decoder.feed(b"".join(group()[:5]), 0)
        self.assertEqual(decoder.feed(b"".join(group()[5:]), 1), [])
        self.assertEqual(decoder.feed(frame(0x32, struct.pack("<HBB", 1, 0, 2)+bytes(8)), 1), [])
        rows = decoder.feed(b"".join(group(2,125,nan=True)+group(3,150,stale=True)), 1)
        self.assertEqual([r["valid"] for r in rows], [0, 0])
        self.assertEqual(decoder.stats["unsupported_versions"], 1)

    def test_monitor_only(self):
        self.assertEqual(heartbeat()[1:6], bytes((0x24, 0xA5, 1, 0xD1, 1)))

    def test_replay_cli(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            raw = path / "test.bin"
            raw.write_bytes(b"".join(group()+group(2,125)))
            command = [sys.executable, str(Path(__file__).with_name("yaw_capture_cli.py")),
                       "--replay", str(raw), "--output", str(path / "out")]
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            with (path / "out" / "samples.csv").open(newline="") as stream:
                self.assertEqual(len(list(csv.DictReader(stream))), 2)
            report = json.loads((path / "out" / "report.json").read_text())
            self.assertEqual(report["commands"], "none")
            # Existing evidence must not be overwritten on repeat execution.
            self.assertNotEqual(subprocess.run(command, capture_output=True).returncode, 0)

    def test_c_wire_fixture(self):
        fixture = Path(__file__).resolve().parents[1] / "NoMachineTemp" / "yaw-capture-tests.exe"
        if not fixture.exists():
            self.skipTest("compile Tests/test_yaw_capture.c for cross-language wire check")
        data = bytes.fromhex(subprocess.check_output([str(fixture)], text=True).strip())
        row = Decoder().feed(data, 0)[0]
        self.assertEqual(row["sequence"], 65535)
        self.assertEqual(row["rc_yaw"], -321)
        self.assertAlmostEqual(row["yaw_deg"], -87.15, places=4)
        self.assertEqual(row["gyro_z_rad_s"], -1.25)
        self.assertEqual(row["small_command"], -2.5)
        self.assertEqual(row["small_current_raw"], -1234)
        self.assertEqual(row["valid"], 1)


if __name__ == "__main__":
    unittest.main()
