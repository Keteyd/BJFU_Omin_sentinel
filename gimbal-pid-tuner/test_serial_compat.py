"""Real Linux PTY tests for CLI-to-Web-Serial terminal settings; no hardware."""
import os
import select
import termios
import tty
import unittest
from unittest.mock import patch

import pid_tune_cli
import pitch_tune_cli
import prepare_browser_serial


class SerialCompatibilityTests(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = os.openpty()
        os.set_blocking(self.slave, False)

    def tearDown(self):
        if self.slave is not None:
            os.close(self.slave)
        os.close(self.master)

    def test_old_configuration_returns_false_eof(self):
        tty.setraw(self.slave)
        attrs = termios.tcgetattr(self.slave)
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.slave, termios.TCSANOW, attrs)
        self.assertEqual(os.read(self.slave, 16), b'')

    def test_both_helpers_keep_idle_reads_nonblocking_without_false_eof(self):
        for module in (pid_tune_cli, pitch_tune_cli):
            with self.subTest(module=module.__name__):
                module.configure_serial(self.slave)
                attrs = termios.tcgetattr(self.slave)
                self.assertEqual(attrs[6][termios.VMIN], 1)
                self.assertEqual(attrs[6][termios.VTIME], 0)
                with self.assertRaises(BlockingIOError):
                    os.read(self.slave, 16)
                self.assertEqual(select.select([self.slave], [], [], 0)[0], [])
                os.write(self.master, b'\xff\x2e\x00\x0d')
                self.assertTrue(select.select([self.slave], [], [], .5)[0])
                self.assertEqual(os.read(self.slave, 16), b'\xff\x2e\x00\x0d')

    def test_browser_style_reopen_preserves_no_false_eof(self):
        path = os.ttyname(self.slave)
        pitch_tune_cli.configure_serial(self.slave)
        os.close(self.slave)
        self.slave = None
        self.slave = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.assertEqual(termios.tcgetattr(self.slave)[6][termios.VMIN], 1)
        with self.assertRaises(BlockingIOError):
            os.read(self.slave, 16)

    def test_browser_preflight_preserves_settings_and_pending_bytes(self):
        tty.setraw(self.slave)
        attrs = termios.tcgetattr(self.slave)
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.slave, termios.TCSANOW, attrs)
        os.write(self.master, b'pending')
        self.assertTrue(select.select([self.slave], [], [], .5)[0])
        info = prepare_browser_serial.prepare_fd(self.slave)
        self.assertEqual(info['previous_vmin'], 0)
        expected = attrs[:6] + [attrs[6][:]]
        expected[6][termios.VMIN] = 1
        self.assertEqual(termios.tcgetattr(self.slave), expected)
        self.assertEqual(os.read(self.slave, 16), b'pending')
        with self.assertRaises(BlockingIOError): os.read(self.slave, 16)

    def test_browser_preflight_refuses_busy_port_without_open(self):
        with patch('prepare_browser_serial.subprocess.run') as run, \
             patch('prepare_browser_serial.os.open') as opened:
            for rc in (0, 2):
                run.return_value.returncode = rc
                with self.assertRaises(RuntimeError): prepare_browser_serial.prepare_serial('/fake')
            opened.assert_not_called()

    def test_browser_preflight_releases_lock_and_fd_on_failure(self):
        with patch('prepare_browser_serial.subprocess.run') as run, \
             patch('prepare_browser_serial.os.open', return_value=123), \
             patch('prepare_browser_serial.os.close') as close, \
             patch('prepare_browser_serial.fcntl.flock'), \
             patch('prepare_browser_serial.fcntl.ioctl') as ioctl, \
             patch('prepare_browser_serial.prepare_fd', side_effect=OSError('injected')):
            run.return_value.returncode = 1
            with self.assertRaises(OSError): prepare_browser_serial.prepare_serial('/fake')
            self.assertEqual(ioctl.call_args_list[-1].args, (123, termios.TIOCNXCL))
            close.assert_called_once_with(123)


if __name__ == '__main__':
    unittest.main()
