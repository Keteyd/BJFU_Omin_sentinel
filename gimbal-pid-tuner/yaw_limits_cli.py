#!/usr/bin/env python3
"""Read SAFE yaw landmarks or observe coordination; never send motor commands."""

from __future__ import annotations

import argparse
import json
import math
import os
import select
import statistics
import struct
import time
from datetime import datetime, timezone
from pathlib import Path

from pitch_tune_cli import configure_serial, extract_frames, make_frame


CMD_ENCODERS = 0x2A
CMD_LIMITS = 0x2B
CMD_COORDINATOR = 0x2C
CMD_PEAKS = 0x2F
MONITOR_FRAME = make_frame(0x24, bytes((0xA5, 1)))
COUNTS_PER_TURN = 8192


def count_delta(value: float, reference: float) -> float:
    return (value - reference + 4096) % COUNTS_PER_TURN - 4096


def decode(frame: bytes, timestamp: float) -> dict:
    small, big, small_age, big_age, valid, active, monitor, version = (
        struct.unpack_from("<4H4B", frame, 2)
    )
    return dict(time_s=round(timestamp, 4), small_raw_count=small,
                big_raw_count=big, small_age_ms=small_age, big_age_ms=big_age,
                feedback_valid_mask=valid, yaw_active=bool(active),
                monitor_only=bool(monitor), protocol_version=version)


def collect(fd: int, duration: float) -> list[dict]:
    samples = []
    buffer = bytearray()
    started = time.monotonic()
    deadline = started + duration
    heartbeat = started
    latest_limits = None
    latest_coordinator = None
    latest_peaks = None
    pending_peaks = []
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= heartbeat:
            if os.write(fd, MONITOR_FRAME) != len(MONITOR_FRAME):
                raise OSError("incomplete monitor heartbeat")
            heartbeat = now + 0.2
        readable, _, _ = select.select([fd], [], [], min(0.04, max(0, deadline - now)))
        if readable:
            try:
                buffer.extend(os.read(fd, 4096))
            except BlockingIOError:
                pass
        for frame in extract_frames(buffer):
            timestamp = time.monotonic() - started
            if frame[1] == CMD_LIMITS:
                latest_limits = decode_limits(frame, timestamp)
            elif frame[1] == CMD_COORDINATOR:
                latest_coordinator = decode_coordinator(frame, timestamp)
            elif frame[1] == CMD_PEAKS:
                latest_peaks = decode_peaks(frame, timestamp)
                pending_peaks.append(latest_peaks)
            elif frame[1] == CMD_ENCODERS:
                sample = decode(frame, timestamp)
                if latest_limits is not None:
                    sample["soft_limits"] = latest_limits
                if latest_coordinator is not None:
                    sample["coordinator"] = latest_coordinator
                if latest_peaks is not None:
                    sample["big_yaw_peaks"] = latest_peaks
                sample["big_yaw_peak_windows"] = pending_peaks
                pending_peaks = []
                samples.append(sample)
    return samples


def decode_limits(frame: bytes, timestamp: float) -> dict:
    joint, lower, upper, speed, valid, status, big, version = struct.unpack_from(
        "<4h4B", frame, 2)
    return dict(time_s=round(timestamp, 4), joint_deg=joint / 100,
                minimum_deg=lower / 100, maximum_deg=upper / 100,
                speed_ref_rpm=speed / 10, valid=bool(valid), status=status,
                big_yaw_enabled=bool(big), protocol_version=version)


def decode_coordinator(frame: bytes, timestamp: float) -> dict:
    error, ref, speed, effort, mode, active, negative, version = struct.unpack_from(
        "<4h4B", frame, 2)
    return dict(time_s=round(timestamp, 4), big_error_deg=error / 100,
                big_speed_ref_rpm=ref / 100, big_speed_rpm=speed / 100,
                big_effort=effort / 1000, mode=mode, active=bool(active),
                relief_direction=-1 if negative else 1, protocol_version=version)


def decode_peaks(frame: bytes, timestamp: float) -> dict:
    count_flags, span, error, speed, effort, duration = struct.unpack_from("<6H", frame, 2)
    return dict(time_s=round(timestamp, 4), task_samples=count_flags & 0x3FFF,
                invalid=bool(count_flags & 0x8000), saturated=bool(count_flags & 0x4000),
                span_deg=span * 360 / COUNTS_PER_TURN, peak_error_deg=error / 100,
                peak_raw_speed_rpm=speed / 100, peak_effort=effort / 1000,
                window_ms=duration)


def summarize(samples: list[dict], duration: float, observe: bool = False,
              require_peaks: bool = False) -> dict:
    if require_peaks and not observe:
        raise ValueError("--require-peaks requires --observe")
    reasons = []
    if not samples:
        return dict(accepted=False, reasons=["no_encoder_telemetry; firmware 0x2A required"], samples=0)
    if any(s["protocol_version"] != 1 or not s["monitor_only"] for s in samples):
        reasons.append("monitor_protocol_not_verified")
    if not observe and any(s["yaw_active"] for s in samples):
        reasons.append("yaw_not_in_SAFE")
    if any(s["feedback_valid_mask"] & 3 != 3 or s["small_age_ms"] > 100 or
           s["big_age_ms"] > 100 or s["small_raw_count"] >= COUNTS_PER_TURN or
           s["big_raw_count"] >= COUNTS_PER_TURN for s in samples):
        reasons.append("invalid_or_stale_encoder_feedback")
    gaps = [b["time_s"] - a["time_s"] for a, b in zip(samples, samples[1:])]
    if (len(samples) < duration * 10 or samples[0]["time_s"] > 0.3 or
            duration - samples[-1]["time_s"] > 0.3 or any(g > 0.25 for g in gaps)):
        reasons.append("incomplete_capture")
    result = dict(samples=len(samples), capture_span_s=round(
        samples[-1]["time_s"] - samples[0]["time_s"], 4))
    for axis in ("small", "big"):
        key = f"{axis}_raw_count"
        anchor = samples[0][key]
        if observe:
            offsets = [0.0]
            for a, b in zip(samples, samples[1:]):
                offsets.append(offsets[-1] + count_delta(b[key], a[key]))
            raw = samples[-1][key]
            result[f"{axis}_net_delta_deg"] = round(offsets[-1] * 360 / COUNTS_PER_TURN, 5)
        else:
            offsets = [count_delta(s[key], anchor) for s in samples]
            raw = (anchor + statistics.median(offsets)) % COUNTS_PER_TURN
        span = (max(offsets) - min(offsets)) * 360 / COUNTS_PER_TURN
        result[f"{axis}_raw_count"] = raw
        result[f"{axis}_encoder_deg"] = round(raw * 360 / COUNTS_PER_TURN, 5)
        result[f"{axis}_span_deg"] = round(span, 5)
        if not observe and span > 0.5:
            reasons.append(f"{axis}_moved_during_capture")
    if observe:
        for key in ("soft_limits", "coordinator"):
            if any(key not in s or s[key]["protocol_version"] != 1 or
                   not 0 <= s["time_s"] - s[key]["time_s"] <= 0.25
                   for s in samples if s["time_s"] >= 0.3):
                reasons.append(f"missing_or_stale_{key}")
        coordinators = [s["coordinator"] for s in samples if "coordinator" in s]
        result["coordinator_modes_seen"] = sorted({s["mode"] for s in coordinators})
        result["peak_abs_big_effort"] = max((abs(s["big_effort"]) for s in coordinators), default=None)
        windows = [w for s in samples for w in s.get("big_yaw_peak_windows", [])]
        if windows:
            result["mcu_peaks"] = dict(
                windows=len(windows), invalid_windows=sum(w["invalid"] for w in windows),
                saturated_windows=sum(w["saturated"] for w in windows),
                active_task_samples=sum(w["task_samples"] for w in windows),
                window_time_s=sum(w["window_ms"] for w in windows) / 1000,
                max_window_span_deg=max(w["span_deg"] for w in windows),
                peak_error_deg=max(w["peak_error_deg"] for w in windows),
                peak_raw_speed_rpm=max(w["peak_raw_speed_rpm"] for w in windows),
                peak_effort=max(w["peak_effort"] for w in windows))
        if require_peaks:
            if not windows or any("big_yaw_peaks" not in s or not
                    0 <= s["time_s"] - s["big_yaw_peaks"]["time_s"] <= 0.25
                    for s in samples if s["time_s"] >= 0.3):
                reasons.append("missing_or_stale_mcu_peaks; firmware 0x2F required")
            if any(w["invalid"] for w in windows):
                reasons.append("invalid_mcu_peak_window")
            if windows and (windows[0]["time_s"] > 0.3 or
                    duration - windows[-1]["time_s"] > 0.3 or
                    any(b["time_s"] - a["time_s"] > 0.25 for a, b in zip(windows, windows[1:])) or
                    abs(sum(w["window_ms"] for w in windows) / 1000 -
                        (windows[-1]["time_s"] - windows[0]["time_s"] +
                         windows[0]["window_ms"] / 1000)) > 0.15):
                reasons.append("incomplete_mcu_peak_coverage")
    result.update(accepted=not reasons, reasons=reasons,
                  capture_mode="observation" if observe else "landmark")
    if "soft_limits" in samples[-1]:
        result["soft_limits"] = samples[-1]["soft_limits"]
        result["soft_limits_age_s"] = round(
            samples[-1]["time_s"] - result["soft_limits"]["time_s"], 4)
    if "coordinator" in samples[-1]:
        result["coordinator"] = samples[-1]["coordinator"]
        result["coordinator_age_s"] = round(
            samples[-1]["time_s"] - result["coordinator"]["time_s"], 4)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0")
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--label", required=True, choices=("middle", "left", "right", "check"))
    parser.add_argument("--json", required=True, type=Path)
    parser.add_argument("--observe", action="store_true",
                        help="record moving/enabled axes; not a landmark calibration")
    parser.add_argument("--require-peaks", action="store_true",
                        help="require fresh, valid MCU control-task peak windows (0x2F)")
    args = parser.parse_args()
    maximum_duration = 120 if args.observe else 30
    if not math.isfinite(args.duration) or not 2 <= args.duration <= maximum_duration:
        raise ValueError(f"duration must be 2..{maximum_duration} seconds")
    if args.observe and args.label != "check":
        raise ValueError("--observe requires --label check; it cannot record a landmark")
    if args.require_peaks and not args.observe:
        raise ValueError("--require-peaks requires --observe")
    if args.json.exists():
        raise ValueError("capture file already exists; choose a new name")
    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        configure_serial(fd)
        samples = collect(fd, args.duration)
    finally:
        os.close(fd)
    report = summarize(samples, args.duration, args.observe, args.require_peaks)
    report.update(label=args.label, captured_utc=datetime.now(timezone.utc).isoformat(),
                  counts_per_encoder_turn=COUNTS_PER_TURN,
                  note=("Read-only observation; acceptance means recording quality, not validated control."
                        if args.observe else "Encoder landmarks only; this tool does not apply limits."))
    args.json.parent.mkdir(parents=True, exist_ok=True)
    with args.json.open("x", encoding="utf-8") as handle:
        json.dump(dict(report=report, samples=samples), handle, indent=2)
    print(json.dumps(report, indent=2))
    return 0 if report["accepted"] else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyboardInterrupt) as error:
        print(f"Capture failed: {error}")
        raise SystemExit(2)
