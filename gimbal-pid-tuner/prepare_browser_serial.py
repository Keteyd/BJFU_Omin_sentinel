#!/usr/bin/env python3
"""Prepare an unowned Linux serial port for Web Serial, without data IO."""

import argparse
import fcntl
import json
import os
import subprocess
import termios

PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"


def prepare_fd(fd):
    before = termios.tcgetattr(fd)
    after = before[:6] + [before[6][:]]
    after[6][termios.VMIN] = 1
    after[6][termios.VTIME] = 0
    # Do not flush input or change baud, modem signals, framing or other flags.
    termios.tcsetattr(fd, termios.TCSANOW, after)
    actual = termios.tcgetattr(fd)
    def number(value):
        return value[0] if isinstance(value, bytes) else value
    if number(actual[6][termios.VMIN]) != 1 or number(actual[6][termios.VTIME]) != 0:
        raise RuntimeError("Failed to verify VMIN=1 / VTIME=0")
    return {"previous_vmin": number(before[6][termios.VMIN]),
            "previous_vtime": number(before[6][termios.VTIME]), "vmin": 1, "vtime": 0}


def prepare_serial(port=PORT):
    owner = subprocess.run(["fuser", port], capture_output=True)
    if owner.returncode != 1:
        raise RuntimeError("Serial port is busy or ownership check failed; close serial clients first")
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    exclusive = False
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        exclusive = True
        return {"port": os.path.realpath(port), **prepare_fd(fd)}
    finally:
        try:
            if exclusive:
                fcntl.ioctl(fd, termios.TIOCNXCL)
        finally:
            os.close(fd)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=PORT)
    print(json.dumps(prepare_serial(parser.parse_args().port)))
