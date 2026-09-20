import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

from yaw_capture_protocol import Decoder, frame, heartbeat


def sample(sequence):
    data = struct.pack("<IIHHHhHH19f", sequence * 2, sequence,
                       0, 0, 0, 0, 207, 0, .001, *([0.] * 16), .002, .001)
    return b"".join(frame(0x32, struct.pack("<HBB", sequence, part, 2)
                          + data[part * 8:part * 8 + 8]) for part in range(12))


class BurstTests(unittest.TestCase):
    def test_imu_diagnostics_interleaved(self):
        data = sample(1)
        diag = frame(0x34, struct.pack("<IIBBBB", 12, 90, 3, 3, 30, 15))
        timing = frame(0x35, struct.pack("<IIf", 12, 2, .012))
        decoder = Decoder(2)
        rows = decoder.feed(data[:32] + diag + timing + data[32:], 1.0)
        self.assertEqual(len(rows), 1)
        self.assertEqual(decoder.stats["incomplete_groups"], 0)
        self.assertEqual(decoder.diagnostics[0]["failed_stage"], 3)
        self.assertEqual(decoder.diagnostics[1]["dt_rejected"], 2)
        self.assertAlmostEqual(decoder.diagnostics[1]["last_dt_s"], .012)

    def test_decode_and_compatibility(self):
        data = sample(1) + sample(2)
        for chunk_size in (1, 7, 16, 101, len(data)):
            decoder = Decoder(2)
            rows = []
            for i in range(0, len(data), chunk_size):
                rows += decoder.feed(data[i:i + chunk_size], 0)
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[1]["delta_ms"], 2)
            self.assertAlmostEqual(rows[0]["control_dt_s"], .002)
            self.assertEqual(decoder.stats["invalid_records"], 0)
        legacy = Decoder()
        self.assertEqual(legacy.feed(data, 0), [])
        self.assertGreater(legacy.stats["unsupported_versions"], 0)
        self.assertEqual(heartbeat(2), frame(0x24, bytes((0xA5, 1, 0xD2, 2))))

    def test_missing_and_corruption(self):
        decoder = Decoder(2)
        rows = decoder.feed(sample(1) + sample(3), 0)
        self.assertEqual(len(rows), 2)
        self.assertEqual(decoder.stats["missing_groups"], 1)
        broken = bytearray(sample(4))
        broken[50] ^= 1
        rows = decoder.feed(broken + sample(5), 0)
        self.assertEqual([r["sequence"] for r in rows], [5])
        self.assertGreater(decoder.stats["crc_errors"], 0)

    def test_full_replay_and_incomplete_rejection(self):
        cli = Path(__file__).with_name("yaw_capture_cli.py")
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            raw = root / "raw.bin"
            raw.write_bytes(b"".join(sample(i) for i in range(1, 513)))
            result = subprocess.run([sys.executable, str(cli), "--buffered",
                                     "--replay", str(raw), "--output", str(root / "ok")],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
            report = json.loads((root / "ok/report.json").read_text())
            self.assertEqual(report["statistics"]["records"], 512)
            self.assertEqual(report["commands"], "none")
            raw.write_bytes(sample(1) + sample(512))
            result = subprocess.run([sys.executable, str(cli), "--buffered",
                                     "--replay", str(raw), "--output", str(root / "partial")],
                                    capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)

    def test_actual_c_layout_and_buffer(self):
        exe = Path(__file__).resolve().parent.parent / "NoMachineTemp/yaw-burst-tests.exe"
        if sys.platform != "win32" or not exe.exists():
            self.skipTest("build the host C fixture on Windows first")
        result = subprocess.run([str(exe)], capture_output=True, text=True, check=True)
        decoder = Decoder(2)
        rows = []
        for line in result.stdout.splitlines():
            rows += decoder.feed(frame(0x32, bytes.fromhex(line)), 0)
        self.assertEqual(len(rows), 512)
        self.assertEqual(rows[-1]["tick_ms"], 1022)
        self.assertEqual(rows[-1]["sequence"], 512)
        self.assertEqual(decoder.stats["invalid_records"], 0)
        self.assertAlmostEqual(rows[-1]["gimbal_dt_s"], .001)


if __name__ == "__main__":
    unittest.main()
