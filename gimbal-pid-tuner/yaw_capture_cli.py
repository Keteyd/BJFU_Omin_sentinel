#!/usr/bin/env python3
"""Collect dual-yaw observations or replay a raw capture. Never command motion."""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import time

from yaw_capture_protocol import CSV_FIELDS, Decoder, heartbeat


def collect(args, decoder, writer, raw):
    # Linux serial dependencies are deferred so offline replay works on Windows.
    import fcntl
    import select
    import subprocess
    import termios
    from pitch_tune_cli import configure_serial

    owner = subprocess.run(["fuser", "-v", args.port], capture_output=True, text=True)
    if owner.returncode != 1:
        raise RuntimeError("serial port occupied or ownership check failed: "
                           + owner.stdout + owner.stderr)
    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    exclusive = False
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        exclusive = True
        configure_serial(fd)
        if args.buffered:
            # Let any previous monitor session expire before starting a new burst.
            time.sleep(1.0)
            termios.tcflush(fd, termios.TCIFLUSH)
        started = next_heartbeat = time.monotonic()
        pending = b""
        print("Capture started. Monitor packets only; no motion or tuning commands.", flush=True)
        while time.monotonic() - started < args.duration:
            now = time.monotonic()
            if not pending and now >= next_heartbeat:
                pending = heartbeat(decoder.version)
                next_heartbeat = now + .2
            readable, writable, _ = select.select([fd], [fd] if pending else [], [], .02)
            if writable:
                try:
                    count = os.write(fd, pending)
                    pending = pending[count:]
                except BlockingIOError:
                    pass
            if readable:
                try:
                    chunk = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                if not chunk:
                    raise RuntimeError("serial device disconnected")
                raw.write(chunk)
                for row in decoder.feed(chunk, time.monotonic() - started):
                    writer.writerow(row)
                if args.buffered and decoder.stats["records"] >= 512:
                    break
            if now - started > (5 if args.buffered else 3) and decoder.stats["records"] == 0:
                raise RuntimeError("no capture records: verify capture firmware, port and baud")
    finally:
        try:
            if exclusive:
                fcntl.ioctl(fd, termios.TIOCNXCL)
        finally:
            os.close(fd)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="Linux serial device, preferably /dev/serial/by-id/...")
    source.add_argument("--replay", type=Path, help="previous raw .bin capture; no serial access")
    parser.add_argument("--duration", type=float, default=20)
    parser.add_argument("--buffered", action="store_true",
                        help="v2: acquire 512 samples in MCU RAM, then download once")
    parser.add_argument("--output", type=Path, required=True, help="new output directory")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 600:
        parser.error("duration must be 1..600 seconds")
    args.output.mkdir(parents=True, exist_ok=False)
    decoder = Decoder(2 if args.buffered else 1)
    error = None
    try:
        with (args.output / "samples.csv").open("x", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=decoder.csv_fields)
            writer.writeheader()
            if args.replay:
                with args.replay.open("rb") as raw:
                    while True:
                        chunk = raw.read(4096)
                        if not chunk:
                            break
                        # Offline input has no host timing; MCU timestamp remains authoritative.
                        for row in decoder.feed(chunk, 0):
                            writer.writerow(row)
            else:
                with (args.output / "raw.bin").open("xb") as raw:
                    collect(args, decoder, writer, raw)
            if args.buffered and (decoder.stats["records"] != 512 or
                                  decoder.stats["missing_groups"] or
                                  decoder.previous_sequence != 512):
                raise RuntimeError("incomplete burst: need samples 1..512; retain partial data")
    except (OSError, RuntimeError, KeyboardInterrupt) as exc:
        error = type(exc).__name__ + ": " + str(exc)
    finally:
        decoder.finish()
        report = dict(protocol_version=decoder.version,
                      nominal_hz=500 if args.buffered else 40, source=args.port or str(args.replay),
                      error=error, statistics=decoder.stats, ranges=decoder.ranges,
                      imu_diagnostics=decoder.diagnostics,
                      commands=("0x24 A5 01 D2 02 buffered monitor heartbeat only" if args.buffered
                                else "0x24 A5 01 D1 01 monitor heartbeat only") if args.port else "none",
                      output_units="software motor command, not measured torque or CAN acknowledgment")
        (args.output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report, indent=2))
    return 1 if error or decoder.stats["records"] == 0 else 0


if __name__ == "__main__":
    raise SystemExit(main())
