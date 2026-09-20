#!/usr/bin/env python3
"""Small-yaw PID telemetry and guarded step-test utility.

The default mode only starts a tuning session and observes telemetry. A step
test requires an explicit arming phrase and always applies bounded parameters
before moving the reference.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import select
import struct
import sys
import termios
import time
import tty
from dataclasses import asdict, dataclass
from pathlib import Path


FRAME_LEN = 16
CMD_TUNE = 0x20
CMD_DATA = 0x21
CMD_AUX = 0x22
CMD_ACTION = 0x23
CMD_SESSION = 0x24
ACTION_RESET = 0x01
ACTION_STEP = 0x02
ARMING_PHRASE = "I_CONFIRM"


@dataclass
class Sample:
    time_s: float
    ref_deg: float
    fdb_deg: float
    error_deg: float
    speed_ref_rpm: float
    speed_rpm: float
    effort: float
    raw_speed_rpm: float | None = None
    current: int | None = None
    motor_online: bool | None = None
    axis_enabled: bool | None = None


def crc8(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def make_frame(command: int, payload: bytes = b"") -> bytes:
    frame = bytearray(FRAME_LEN)
    frame[0] = 0xFF
    frame[1] = command
    frame[2:2 + min(12, len(payload))] = payload[:12]
    frame[14] = crc8(frame[:14])
    frame[15] = 0x0D
    return bytes(frame)


def configure_serial(fd: int) -> None:
    tty.setraw(fd, termios.TCSANOW)
    attrs = termios.tcgetattr(fd)
    attrs[0] &= ~(
        termios.IGNBRK
        | termios.BRKINT
        | termios.PARMRK
        | termios.ISTRIP
        | termios.INLCR
        | termios.IGNCR
        | termios.ICRNL
        | termios.IXON
        | termios.IXOFF
    )
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    # O_NONBLOCK handles idle reads; VMIN=0 can look like device loss to Chrome.
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)


def extract_frames(buffer: bytearray) -> list[bytes]:
    frames: list[bytes] = []
    while len(buffer) >= FRAME_LEN:
        try:
            start = buffer.index(0xFF)
        except ValueError:
            buffer.clear()
            break
        if start:
            del buffer[:start]
        if len(buffer) < FRAME_LEN:
            break
        candidate = bytes(buffer[:FRAME_LEN])
        if candidate[15] == 0x0D and crc8(candidate[:14]) == candidate[14]:
            frames.append(candidate)
            del buffer[:FRAME_LEN]
        else:
            del buffer[0]
    return frames


def encode_optional_cap(value: float, scale: int) -> int:
    if value == -1:
        return 0xFFFF
    if not math.isfinite(value) or value < 0 or round(value * scale) >= 0xFFFF:
        raise ValueError("rate cap must be -1 (unlimited) or a nonnegative encodable value")
    return round(value * scale)


def tune_payload(args: argparse.Namespace, effort_limit: float | None = None) -> bytes:
    effort = args.effort_limit if effort_limit is None else effort_limit
    values = (
        round(args.angle_kp * 1000),
        round(args.speed_kp * 1000),
        encode_optional_cap(args.speed_limit, 100),
        round(args.filter_alpha * 10000),
        encode_optional_cap(args.manual_step, 10000),
        round(effort * 1000),
    )
    if any(value < 0 or value > 0xFFFF for value in values):
        raise ValueError("tuning parameter is outside the protocol's uint16 range")
    return struct.pack("<6H", *values)


def summarize(samples: list[Sample]) -> dict[str, object]:
    if not samples:
        return {"samples": 0}
    errors = [sample.error_deg for sample in samples]
    speeds = [sample.speed_rpm for sample in samples]
    efforts = [sample.effort for sample in samples]
    currents = [abs(sample.current) for sample in samples if sample.current is not None]
    signs = [math.copysign(1, value) for value in errors if abs(value) >= 0.03]
    return {
        "samples": len(samples),
        "duration_s": round(samples[-1].time_s - samples[0].time_s, 3),
        "final_ref_deg": samples[-1].ref_deg,
        "final_fdb_deg": samples[-1].fdb_deg,
        "final_error_deg": samples[-1].error_deg,
        "peak_abs_error_deg": round(max(map(abs, errors)), 3),
        "peak_abs_speed_rpm": round(max(map(abs, speeds)), 3),
        "peak_abs_effort": round(max(map(abs, efforts)), 3),
        "peak_abs_current": max(currents) if currents else None,
        "error_sign_changes": sum(a != b for a, b in zip(signs, signs[1:])),
        "motor_online": samples[-1].motor_online,
        "axis_enabled": samples[-1].axis_enabled,
    }


def write_csv(path: Path, samples: list[Sample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=Sample.__dataclass_fields__.keys())
        writer.writeheader()
        writer.writerows(asdict(sample) for sample in samples)


def run(args: argparse.Namespace) -> int:
    active = args.step is not None
    if active and args.armed != ARMING_PHRASE:
        print(f"Step test refused. Pass --armed {ARMING_PHRASE} after physical safety checks.", file=sys.stderr)
        return 2

    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    configure_serial(fd)
    receive_buffer = bytearray()
    samples: list[Sample] = []
    latest_aux: dict[str, object] = {}
    started = time.monotonic()
    deadline = started + args.duration
    next_heartbeat = started
    step_at = started + args.lead_in
    step_sent = False
    stopped_for_limit = False
    raw_bytes = 0
    vision_frames = 0
    command_counts: dict[str, int] = {}
    raw_capture = bytearray()

    try:
        if active:
            os.write(fd, make_frame(CMD_TUNE, tune_payload(args)))
            os.write(fd, make_frame(CMD_ACTION, bytes([0, 0, ACTION_RESET])))

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_heartbeat:
                os.write(fd, make_frame(CMD_SESSION))
                next_heartbeat = now + 0.2
            if active and not step_sent and now >= step_at:
                payload = struct.pack("<hB9x", round(args.step * 100), ACTION_STEP)
                os.write(fd, make_frame(CMD_ACTION, payload))
                step_sent = True

            readable, _, _ = select.select([fd], [], [], min(0.04, max(0.0, deadline - now)))
            if readable:
                try:
                    chunk = os.read(fd, 4096)
                    raw_bytes += len(chunk)
                    if args.raw:
                        raw_capture.extend(chunk)
                    vision_frames += sum(
                        chunk[index] == 0x53 and chunk[index + 14] == 0x45
                        for index in range(max(0, len(chunk) - 14))
                    )
                    receive_buffer.extend(chunk)
                except BlockingIOError:
                    pass

            for frame in extract_frames(receive_buffer):
                command = f"0x{frame[1]:02x}"
                command_counts[command] = command_counts.get(command, 0) + 1
                if frame[1] == CMD_AUX:
                    raw_speed, current, motor_online, axis_enabled = struct.unpack_from("<hhBB", frame, 2)
                    latest_aux = {
                        "raw_speed_rpm": raw_speed / 10.0,
                        "current": current,
                        "motor_online": bool(motor_online),
                        "axis_enabled": bool(axis_enabled),
                    }
                elif frame[1] == CMD_DATA:
                    ref, fdb, error, speed_ref, speed, effort = struct.unpack_from("<HHhhhh", frame, 2)
                    sample = Sample(
                        time_s=round(time.monotonic() - started, 4),
                        ref_deg=ref / 100.0,
                        fdb_deg=fdb / 100.0,
                        error_deg=error / 100.0,
                        speed_ref_rpm=speed_ref / 10.0,
                        speed_rpm=speed / 10.0,
                        effort=effort / 1000.0,
                        **latest_aux,
                    )
                    samples.append(sample)
                    unsafe = (
                        abs(sample.speed_rpm) > args.abort_speed
                        or (sample.current is not None and abs(sample.current) > args.abort_current)
                    )
                    if active and unsafe and not stopped_for_limit:
                        os.write(fd, make_frame(CMD_TUNE, tune_payload(args, effort_limit=0.0)))
                        os.write(fd, make_frame(CMD_ACTION, bytes([0, 0, ACTION_RESET])))
                        stopped_for_limit = True
                        print("ABORT: speed/current limit exceeded; effort limit set to zero.", file=sys.stderr)
                        deadline = min(deadline, time.monotonic() + 0.5)
    except KeyboardInterrupt:
        if active:
            os.write(fd, make_frame(CMD_TUNE, tune_payload(args, effort_limit=0.0)))
            os.write(fd, make_frame(CMD_ACTION, bytes([0, 0, ACTION_RESET])))
        print("Interrupted; active test output was limited to zero.", file=sys.stderr)
    finally:
        os.close(fd)

    if args.csv:
        write_csv(Path(args.csv), samples)
    if args.raw:
        raw_path = Path(args.raw)
        raw_path.parent.mkdir(parents=True, exist_ok=True)
        raw_path.write_bytes(raw_capture)
    report = summarize(samples)
    report["aborted"] = stopped_for_limit
    report["raw_bytes"] = raw_bytes
    report["vision_frames_seen"] = vision_frames
    report["valid_command_frames"] = command_counts
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 1 if not samples or stopped_for_limit else 0


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--port", default="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0")
    result.add_argument("--duration", type=float, default=4.0)
    result.add_argument("--csv")
    result.add_argument("--raw", help="write the received byte stream for protocol diagnosis")
    result.add_argument("--step", type=float, help="guarded small-yaw reference step in degrees")
    result.add_argument("--armed", default="")
    result.add_argument("--lead-in", type=float, default=1.0)
    result.add_argument("--angle-kp", type=float, default=2.60)
    result.add_argument("--speed-kp", type=float, default=0.60)
    result.add_argument("--speed-limit", type=float, default=-1.0,
                        help="rpm cap; -1 disables the fixed cap (new firmware required)")
    result.add_argument("--filter-alpha", type=float, default=0.15)
    result.add_argument("--manual-step", type=float, default=-1.0,
                        help="deg/tick cap; -1 disables the fixed cap (new firmware required)")
    result.add_argument("--effort-limit", type=float, default=6.0)
    result.add_argument("--abort-speed", type=float, default=12.0)
    result.add_argument("--abort-current", type=int, default=6000)
    return result


if __name__ == "__main__":
    try:
        raise SystemExit(run(parser().parse_args()))
    except (OSError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2)
