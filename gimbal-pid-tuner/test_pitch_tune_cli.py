import contextlib
import io
import json
import unittest
from unittest.mock import patch

import pitch_tune_cli as cli


class FakeSerial:
    def __init__(self, batches=(), fail_after_start=False):
        self.now = 0.0
        self.batches = list(batches)
        self.writes = []
        self.fail_after_start = fail_after_start

    def clock(self):
        return self.now

    def select(self, *args):
        self.now += 0.05
        if self.fail_after_start:
            raise OSError("simulated serial failure")
        return ([7] if self.batches else [], [], [])

    def read(self, *args):
        return self.batches.pop(0)

    def write(self, fd, frame):
        self.writes.append(frame)
        return len(frame)


class PitchCliTests(unittest.TestCase):
    def run_fake(self, options, fake):
        with contextlib.ExitStack() as stack:
            for target, value in [
                ("os.open", lambda *a: 7), ("os.close", lambda *a: None),
                ("configure_serial", lambda *a: None),
                ("os.write", fake.write), ("os.read", fake.read),
                ("select.select", fake.select), ("time.monotonic", fake.clock),
                ("termios.tcdrain", lambda *a: None),
            ]:
                stack.enter_context(patch("pitch_tune_cli." + target, value))
            output = stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            result = cli.run(cli.parser().parse_args(options))
            return result, json.loads(output.getvalue())

    def test_monitor_sends_only_nonlocking_session(self):
        fake = FakeSerial()
        result, report = self.run_fake(["--monitor-only", "--duration", "0.7"], fake)
        self.assertEqual(result, 1)  # Missing capability/telemetry is not success.
        self.assertFalse(report["monitor_verified"])
        self.assertTrue(fake.writes)
        for frame in fake.writes:
            self.assertEqual(frame[1], cli.CMD_SESSION)
            self.assertEqual(frame[2:4], cli.MONITOR_SESSION)
            self.assertEqual(cli.crc8(frame[:14]), frame[14])

    def test_active_telemetry_loss_disables_even_leave_enabled(self):
        fake = FakeSerial()
        result, report = self.run_fake(["--enable", "--armed", "I_CONFIRM",
                                        "--leave-enabled", "--duration", "2"], fake)
        self.assertEqual(result, 1)
        self.assertEqual(report["abort_reason"], "telemetry_timeout")
        self.assertEqual(fake.writes[-1][1], cli.CMD_TUNE)
        self.assertEqual(fake.writes[-1][13] & cli.TUNE_ENABLE, 0)

    def test_serial_exception_attempts_disable(self):
        fake = FakeSerial(fail_after_start=True)
        with self.assertRaises(OSError):
            self.run_fake(["--enable", "--armed", "I_CONFIRM", "--leave-enabled"], fake)
        self.assertEqual(fake.writes[-1][1], cli.CMD_TUNE)
        self.assertEqual(fake.writes[-1][13] & cli.TUNE_ENABLE, 0)

    def test_invalid_parameters_never_open_port(self):
        for options in [["--duration", "nan"], ["--duration", "-1"],
                        ["--motor-kd", "nan"], ["--abort-speed", "0"]]:
            with patch("pitch_tune_cli.os.open") as opened:
                with self.assertRaises(ValueError):
                    cli.run(cli.parser().parse_args(options))
                opened.assert_not_called()

    def test_raw_error_not_hidden_by_control_deadband(self):
        s = cli.Sample(0, 0.002, 0, 0, 0, 0, 0)
        report = cli.summarize([s])
        self.assertEqual(report["peak_abs_error_deg"], 0)
        self.assertGreater(report["peak_abs_raw_error_deg"], 0.1)


if __name__ == "__main__":
    unittest.main()
