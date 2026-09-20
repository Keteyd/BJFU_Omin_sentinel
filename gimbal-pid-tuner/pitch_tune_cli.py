#!/usr/bin/env python3
"""Pitch IMU telemetry and guarded gravity-feedforward step tester."""

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
CMD_SESSION = 0x24
CMD_TUNE = 0x25
CMD_DATA = 0x26
CMD_AUX = 0x27
CMD_ACTION = 0x28
CMD_LIMITS = 0x29
MONITOR_SESSION = bytes((0xA5, 2))
ACTION_RESET = 0x01
ACTION_STEP = 0x02
ACTION_MIT_GAINS = 0x04
TUNE_ENABLE = 0x01
ARMING_PHRASE = "I_CONFIRM"


@dataclass
class Sample:
    time_s: float
    ref_rad: float
    fdb_rad: float
    error_rad: float
    motor_ref_rad: float
    motor_fdb_rad: float
    motor_rate_ref_rad_s: float
    imu_rate_rad_s: float | None = None
    gravity_effort: float | None = None
    motor_speed_rad_s: float | None = None
    motor_effort: float | None = None
    motor_online: bool | None = None
    axis_enabled: bool | None = None
    applied_step_rad: float | None = None


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
        termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP
        | termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON
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
        frame = bytes(buffer[:FRAME_LEN])
        if frame[15] == 0x0D and crc8(frame[:14]) == frame[14]:
            frames.append(frame)
            del buffer[:FRAME_LEN]
        else:
            del buffer[0]
    return frames


def tune_payload(args: argparse.Namespace, enabled: bool) -> bytes:
    values = (
        round(args.imu_kp * 1000),
        round(args.imu_kd * 1000),
        round(args.rate_limit * 1000),
        round(args.gravity * 1000),
        round(args.angle_deadband * 100000),
        round(args.rate_deadband * 1000),
    )
    if any(value < 0 or value > 0xFFFF for value in (values[0], values[1], values[2], values[4])):
        raise ValueError("unsigned tuning parameter is outside protocol range")
    if values[3] < -32768 or values[3] > 32767:
        raise ValueError("gravity effort is outside protocol range")
    if values[5] < 0 or values[5] > 0xFF:
        raise ValueError("rate deadband is outside protocol range")
    return struct.pack("<HHHhHBB", *values, TUNE_ENABLE if enabled else 0)


def action_payload(step_mrad: int, flags: int, args: argparse.Namespace) -> bytes:
    motor_kp = round(args.motor_kp * 100)
    motor_kd = round(args.motor_kd * 1000)
    if motor_kp < 0 or motor_kp > 0xFFFF:
        raise ValueError("MIT motor Kp is outside protocol range")
    if motor_kd < 0 or motor_kd > 0xFFFF:
        raise ValueError("MIT motor Kd is outside protocol range")
    return struct.pack("<hBHH5x", step_mrad, flags, motor_kp, motor_kd)


def summarize(samples: list[Sample]) -> dict[str, object]:
    if not samples:
        return {"samples": 0}
    errors = [sample.error_rad for sample in samples]
    raw_errors = [sample.ref_rad - sample.fdb_rad for sample in samples]
    speeds = [abs(sample.motor_speed_rad_s) for sample in samples if sample.motor_speed_rad_s is not None]
    efforts = [abs(sample.motor_effort) for sample in samples if sample.motor_effort is not None]
    applied_steps = [sample.applied_step_rad for sample in samples if sample.applied_step_rad is not None]
    return {
        "samples": len(samples),
        "duration_s": round(samples[-1].time_s - samples[0].time_s, 3),
        "final_error_deg": round(math.degrees(samples[-1].error_rad), 3),
        "peak_abs_error_deg": round(math.degrees(max(map(abs, errors))), 3),
        "peak_abs_raw_error_deg": round(math.degrees(max(map(abs, raw_errors))), 3),
        "imu_span_deg": round(math.degrees(max(s.fdb_rad for s in samples) - min(s.fdb_rad for s in samples)), 3),
        "peak_abs_motor_speed_rad_s": round(max(speeds), 3) if speeds else None,
        "peak_abs_motor_effort": round(max(efforts), 3) if efforts else None,
        "final_motor_ref_rad": samples[-1].motor_ref_rad,
        "final_motor_fdb_rad": samples[-1].motor_fdb_rad,
        "motor_online": samples[-1].motor_online,
        "axis_enabled": samples[-1].axis_enabled,
        "primary_imu_delta_deg": round(
            math.degrees(samples[-1].fdb_rad - samples[0].fdb_rad), 3),
        "applied_step_deg": round(
            math.degrees(applied_steps[-1]), 3) if applied_steps else None,
    }


def run(args: argparse.Namespace) -> int:
    active = args.enable or args.step_deg is not None
    if args.monitor_only and active:
        print("--monitor-only cannot be combined with --enable or --step-deg.", file=sys.stderr)
        return 2
    if active and args.armed != ARMING_PHRASE:
        print(f"Active test refused. Pass --armed {ARMING_PHRASE} after physical checks.", file=sys.stderr)
        return 2
    if args.step_deg is not None and not args.enable:
        print("A step test also requires --enable.", file=sys.stderr)
        return 2

    if not math.isfinite(args.duration) or args.duration <= 0:
        raise ValueError("duration must be finite and positive")
    for name in ("abort_error", "abort_speed", "abort_effort", "abort_motor_travel"):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if not math.isfinite(args.lead_in) or args.lead_in < 0:
        raise ValueError("lead-in must be finite and nonnegative")
    if args.step_deg is not None and (not math.isfinite(args.step_deg) or
                                     abs(math.radians(args.step_deg)) > 0.01):
        raise ValueError("firmware step range is +/-0.01 rad (+/-0.573 degrees)")
    # Validate every command before opening the live serial port.
    tune_on = tune_payload(args, args.enable) if not args.monitor_only else b""
    tune_off = tune_payload(args, False) if not args.monitor_only else b""
    initial_action = action_payload(0, ACTION_RESET | ACTION_MIT_GAINS, args) if not args.monitor_only else b""

    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    receive_buffer = bytearray()
    samples: list[Sample] = []
    latest_aux: dict[str, object] = {}
    started = time.monotonic()
    deadline = started + args.duration
    next_heartbeat = started
    step_at = started + args.lead_in
    step_sent = False
    aborted = False
    command_counts: dict[str, int] = {}
    initial_motor_ref: float | None = None
    last_data = started
    last_aux = started
    abort_reason = None
    limits = None

    try:
        configure_serial(fd)
        if args.monitor_only:
            os.write(fd, make_frame(CMD_SESSION, MONITOR_SESSION))
        else:
            os.write(fd, make_frame(CMD_TUNE, tune_on))
            os.write(fd, make_frame(CMD_ACTION, initial_action))
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_heartbeat:
                os.write(fd, make_frame(CMD_SESSION, MONITOR_SESSION if args.monitor_only else b""))
                next_heartbeat = now + 0.2
            if active and (now - last_data > 0.5 or now - last_aux > 0.5):
                aborted = True
                abort_reason = "telemetry_timeout"
                break
            if (args.step_deg is not None and not step_sent and now >= step_at and
                    samples and latest_aux.get("motor_online") and
                    latest_aux.get("axis_enabled")):
                step_mrad = round(math.radians(args.step_deg) * 1000)
                os.write(fd, make_frame(
                    CMD_ACTION, action_payload(step_mrad, ACTION_STEP, args)))
                step_sent = True

            readable, _, _ = select.select([fd], [], [], min(0.04, max(0.0, deadline - now)))
            if readable:
                try:
                    receive_buffer.extend(os.read(fd, 4096))
                except BlockingIOError:
                    pass

            for frame in extract_frames(receive_buffer):
                command_counts[f"0x{frame[1]:02x}"] = command_counts.get(f"0x{frame[1]:02x}", 0) + 1
                if frame[1] == CMD_AUX:
                    last_aux = time.monotonic()
                    imu_rate, gravity, motor_speed, motor_effort, online, enabled, applied_step = struct.unpack_from("<4hBBh", frame, 2)
                    latest_aux = {
                        "imu_rate_rad_s": imu_rate / 1000.0,
                        "gravity_effort": gravity / 1000.0,
                        "motor_speed_rad_s": motor_speed / 1000.0,
                        "motor_effort": motor_effort / 1000.0,
                        "motor_online": bool(online),
                        "axis_enabled": bool(enabled),
                        "applied_step_rad": applied_step / 1000.0,
                    }
                elif frame[1] == CMD_LIMITS:
                    lo, hi, ilo, ihi, valid, state, monitor, version = struct.unpack_from("<4h4B", frame, 2)
                    limits = dict(motor_min_rad=lo / 1000, motor_max_rad=hi / 1000,
                                  imu_min_rad=ilo / 1000, imu_max_rad=ihi / 1000,
                                  initialized=bool(valid), dm_state=state,
                                  monitor_only=bool(monitor), protocol_version=version)
                elif frame[1] == CMD_DATA:
                    last_data = time.monotonic()
                    values = struct.unpack_from("<6h", frame, 2)
                    sample = Sample(
                        time_s=round(time.monotonic() - started, 4),
                        ref_rad=values[0] / 1000.0,
                        fdb_rad=values[1] / 1000.0,
                        error_rad=values[2] / 1000.0,
                        motor_ref_rad=values[3] / 1000.0,
                        motor_fdb_rad=values[4] / 1000.0,
                        motor_rate_ref_rad_s=values[5] / 1000.0,
                        **latest_aux,
                    )
                    samples.append(sample)
                    if initial_motor_ref is None:
                        initial_motor_ref = sample.motor_ref_rad
                    unsafe = (
                        abs(sample.ref_rad - sample.fdb_rad) > args.abort_error
                        or (sample.motor_speed_rad_s is not None and abs(sample.motor_speed_rad_s) > args.abort_speed)
                        or (sample.motor_effort is not None and abs(sample.motor_effort) > args.abort_effort)
                        or (initial_motor_ref is not None and
                            abs(sample.motor_ref_rad - initial_motor_ref) > args.abort_motor_travel)
                    )
                    if active and unsafe:
                        aborted = True
                        abort_reason = "motion_threshold"
                        deadline = min(deadline, time.monotonic() + 0.3)
                        os.write(fd, make_frame(CMD_TUNE, tune_payload(args, False)))
                        break
            if active and latest_aux and (not latest_aux["motor_online"] or
                                          not latest_aux["axis_enabled"]):
                aborted = True
                abort_reason = "axis_not_ready"
                break
    except KeyboardInterrupt:
        aborted = active
        abort_reason = "interrupted"
    except (OSError, ValueError):
        aborted = active
        raise
    finally:
        if active and (not samples or (args.step_deg is not None and not step_sent)):
            aborted = True
        if active and (aborted or not args.leave_enabled):
            try:
                os.write(fd, make_frame(CMD_TUNE, tune_off))
                termios.tcdrain(fd)
            except OSError:
                pass
        os.close(fd)

    if args.csv:
        path = Path(args.csv)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=Sample.__dataclass_fields__.keys())
            writer.writeheader()
            writer.writerows(asdict(sample) for sample in samples)

    report = summarize(samples)
    report["aborted"] = aborted
    report["abort_reason"] = abort_reason
    report["limits"] = limits
    report["monitor_verified"] = bool(limits and limits["monitor_only"] and limits["protocol_version"] == 1)
    report["valid_command_frames"] = command_counts
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 1 if not samples or aborted or (args.monitor_only and not report["monitor_verified"]) else 0


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--port", default="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0")
    result.add_argument("--duration", type=float, default=5.0)
    result.add_argument("--csv")
    result.add_argument("--monitor-only", action="store_true")
    result.add_argument("--enable", action="store_true")
    result.add_argument("--step-deg", type=float)
    result.add_argument("--armed", default="")
    result.add_argument("--lead-in", type=float, default=1.0)
    result.add_argument("--leave-enabled", action="store_true")
    result.add_argument("--imu-kp", type=float, default=14.0)
    result.add_argument("--imu-kd", type=float, default=0.30)
    result.add_argument("--rate-limit", type=float, default=2.0)
    result.add_argument("--gravity", type=float, default=0.30)
    result.add_argument("--motor-kp", type=float, default=50.0)
    result.add_argument("--motor-kd", type=float, default=3.2)
    result.add_argument("--angle-deadband", type=float, default=0.0026)
    result.add_argument("--rate-deadband", type=float, default=0.020)
    result.add_argument("--abort-error", type=float, default=0.035)
    result.add_argument("--abort-speed", type=float, default=2.0)
    result.add_argument("--abort-effort", type=float, default=5.0)
    result.add_argument("--abort-motor-travel", type=float, default=0.06)
    return result


if __name__ == "__main__":
    try:
        raise SystemExit(run(parser().parse_args()))
    except (OSError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2)
