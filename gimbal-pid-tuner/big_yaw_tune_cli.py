#!/usr/bin/env python3
"""Read back big-yaw parameters; optional SAFE-only writes. Never command motion."""

import argparse
import fcntl
import json
import math
import os
import secrets
import select
import struct
import subprocess
import termios
import time

from pitch_tune_cli import configure_serial, extract_frames, make_frame

CMD_TUNE = 0x2D
CMD_CONFIG = 0x2E
SCALES = (1000, 1000, 1000, 10000)
NAMES = ("angle_kp", "speed_kp", "effort_limit", "filter_tau_s",
         "angle_ki", "angle_kd", "speed_ki", "speed_kd")
MAXIMUM = (8, 5, 30, 0.1)


def decode_config(frame):
    if frame[1] != CMD_CONFIG:
        return None
    *values, request_id, status, flags = struct.unpack_from("<5H2B", frame, 2)
    if not flags & 0x80 or status > 4:
        raise ValueError("unsupported big-yaw configuration reply")
    if flags & 0x40:
        return None  # Wait for a complete, coherent 0x31 float snapshot.
    result = dict(zip(NAMES, (v / s for v, s in zip(values, SCALES))))
    result.update(request_id=request_id, status=status, remote_safe=bool(flags & 1),
                  outputs_off=bool(flags & 2), active=bool(flags & 4),
                  saturated=bool(flags & 8), motor_online=bool(flags & 16),
                  effort_limit_max=30.0 if flags & 32 else 8.0)
    result.update(full_pid=False, angle_ki=0.0, angle_kd=0.0, speed_ki=0.0, speed_kd=0.0)
    encode_values([result[name] for name in NAMES[:4]])
    if result["effort_limit"] > result["effort_limit_max"]:
        raise ValueError("effort readback exceeds advertised firmware capability")
    return result


def full_values(values):
    result = []
    for name in NAMES:
        value = values[name]
        if not math.isfinite(value) or value < 0:
            raise ValueError(f"{name} must be finite and nonnegative")
        if name == "effort_limit" and value > 30:
            raise ValueError("effort exceeds GM6020 command full scale (30)")
        try:
            encoded = struct.unpack("<f", struct.pack("<f", value))[0]
        except (OverflowError, struct.error) as error:
            raise ValueError(f"{name} is not representable as float32") from error
        if not math.isfinite(encoded) or (value != 0 and encoded == 0):
            raise ValueError(f"{name} is not representable as a nonzero finite float32")
        result.append(encoded)
    return tuple(result)


class FullConfigReader:
    def __init__(self):
        self.parts = []
        self.key = None
        self.started = 0

    def receive(self, frame):
        if frame[1] != 0x31:
            return None
        request, part, flags, a, b = struct.unpack_from("<HBBff", frame, 2)
        if part == 0:
            self.parts, self.key, self.started = [], (request, flags), time.monotonic()
        if (part != len(self.parts) or part > 3 or (request, flags) != self.key or
                time.monotonic() - self.started > .35 or flags & 7 > 4):
            self.parts, self.key = [], None
            return None
        self.parts.append((a, b))
        if len(self.parts) != 4:
            return None
        values = dict(zip(NAMES, (v for pair in self.parts for v in pair)))
        self.parts, self.key = [], None
        full_values(values)
        values.update(request_id=request, status=flags & 7, remote_safe=bool(flags & 8),
                      outputs_off=bool(flags & 16), active=bool(flags & 32),
                      motor_online=bool(flags & 64), saturated=bool(flags & 128),
                      effort_limit_max=30.0, full_pid=True)
        return values


def encode_values(values):
    if len(values) != 4:
        raise ValueError("exactly four parameters required")
    for name, value, maximum in zip(NAMES, values, MAXIMUM):
        if not math.isfinite(value) or not 0 <= value <= maximum:
            raise ValueError(f"{name} must be finite and within 0..{maximum}")
    return tuple(round(v * s) for v, s in zip(values, SCALES))


class TuneTransaction:
    def __init__(self, changes=None):
        self.changes = changes
        self.request_id = None
        self.expected = None
        self.done = False
        self.result = None
        self.full_reader = FullConfigReader()
        self.full = False

    def receive(self, frame):
        config = self.full_reader.receive(frame) if frame[1] == 0x31 else decode_config(frame)
        if config is None:
            return None
        if self.changes is None:
            self.result, self.done = config, True
            return None
        if self.request_id is None:
            if not config["remote_safe"] or not config["outputs_off"] or config["active"]:
                raise ValueError("write refused: receiver must be SAFE and yaw outputs off")
            values = {name: self.changes.get(name, config[name]) for name in NAMES}
            self.full = config["full_pid"]
            if values["effort_limit"] > config["effort_limit_max"]:
                raise ValueError("write refused: effort above 8 requires full-range gimbal firmware")
            if not self.full and any(values[name] != 0 for name in NAMES[4:]):
                raise ValueError("I/D tuning requires complete-PID firmware")
            self.expected = full_values(values) if self.full else encode_values([values[name] for name in NAMES[:4]])
            self.request_id = secrets.randbelow(65535) + 1
            if self.request_id == config["request_id"]:
                self.request_id = self.request_id % 65535 + 1
            if self.full:
                return b"".join(make_frame(0x30, struct.pack("<HBBff", self.request_id, part, 0xC3,
                    *self.expected[part * 2:part * 2 + 2])) for part in range(4))
            return make_frame(CMD_TUNE, struct.pack("<5H2B", *self.expected,
                                                   self.request_id, 0xB7, 1))
        if config["request_id"] != self.request_id:
            return None
        if config["status"] != 1:
            raise ValueError(f"firmware rejected write, status={config['status']}")
        if config["full_pid"] != self.full:
            raise ValueError("firmware interface changed during write; query again")
        actual = full_values(config) if self.full else encode_values([config[name] for name in NAMES[:4]])
        if actual != self.expected:
            raise ValueError("readback mismatch; do not assume requested values were applied")
        self.result, self.done = config, True
        return None


def write_frame(fd, frame):
    remaining = memoryview(frame)
    deadline = time.monotonic() + 0.5
    while remaining:
        if time.monotonic() >= deadline:
            raise TimeoutError("serial write incomplete; parameter state may be unknown")
        if select.select([], [fd], [], 0.05)[1]:
            try:
                count = os.write(fd, remaining)
                if count <= 0:
                    raise OSError("serial disconnected")
                remaining = remaining[count:]
            except BlockingIOError:
                pass


def release_serial(fd, exclusive):
    try:
        if exclusive:
            fcntl.ioctl(fd, termios.TIOCNXCL)
    finally:
        os.close(fd)


def write_transaction(fd, frames):
    if not frames or len(frames) % 16:
        raise ValueError("transaction must contain whole 16-byte frames")
    for offset in range(0, len(frames), 16):
        # Also supports older IDLE-only RX firmware; never retry a transaction.
        time.sleep(0.020)
        write_frame(fd, frames[offset:offset + 16])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0")
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--arm", default="")
    for name in NAMES:
        parser.add_argument("--" + name.replace("_", "-"), type=float)
    args = parser.parse_args()
    changes = {name: getattr(args, name) for name in NAMES if getattr(args, name) is not None}
    if args.apply:
        if args.arm != "I_CONFIRM" or not changes:
            parser.error("writes require --apply --arm I_CONFIRM and at least one parameter")
    elif changes:
        parser.error("parameter values require --apply; omitted values are read from firmware")
    try:
        full_values({name: changes.get(name, 0.0) for name in NAMES})
    except ValueError as error:
        parser.error(str(error))
    owner = subprocess.run(["fuser", args.port], capture_output=True)
    if owner.returncode != 1:
        raise RuntimeError("serial port is busy or its ownership could not be checked")
    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    exclusive = False
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        exclusive = True
        configure_serial(fd)
        transaction = TuneTransaction(changes if args.apply else None)
        buffer = bytearray()
        deadline, heartbeat = time.monotonic() + 4, 0
        while not transaction.done and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= heartbeat:
                write_frame(fd, make_frame(0x24, b"\xa5\x01"))
                heartbeat = now + 0.2
            if select.select([fd], [], [], 0.05)[0]:
                try:
                    data = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                if not data:
                    raise OSError("serial disconnected")
                buffer.extend(data)
                for frame in extract_frames(buffer):
                    output = transaction.receive(frame)
                    if output is not None:
                        write_transaction(fd, output)
                    if transaction.done:
                        break
        if not transaction.done:
            raise TimeoutError("no verified readback; old firmware or lost ACK; do not retry blindly")
        print(json.dumps(transaction.result, indent=2))
    finally:
        release_serial(fd, exclusive)


if __name__ == "__main__":
    main()
