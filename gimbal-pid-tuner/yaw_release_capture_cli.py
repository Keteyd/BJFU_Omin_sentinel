#!/usr/bin/env python3
"""Record v1 stick release, then request one v2 burst. No actuator commands."""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import time

from yaw_capture_protocol import Decoder, heartbeat
from yaw_release_trigger import ReleaseTrigger


def verify_burst(decoder, rows, event):
    stats = decoder.stats
    if (len(rows) != 512 or stats["invalid_records"] or stats["missing_groups"] or
            stats["crc_errors"] or stats["incomplete_groups"] or
            rows[0]["sequence"] != 1 or rows[-1]["sequence"] != 512 or
            any(row["delta_ms"] != 2 for row in rows[1:])):
        raise RuntimeError("burst quality check failed; retain partial evidence")
    delay = (rows[0]["tick_ms"] - event["tick_ms"]) & 0xffffffff
    if delay > 150:
        raise RuntimeError("burst starts too late or MCU reset; cannot align release")
    if any(row["big_command"] != 0 or not row["flags"] & 128 or row["flags"] & 32
           for row in rows):
        raise RuntimeError("big-yaw isolation missing in buffered evidence")
    return delay


def collect(args, live, burst, trigger, writers, raw, report):
    import fcntl
    import select
    import subprocess
    import termios
    from pitch_tune_cli import configure_serial

    owner = subprocess.run(["fuser", "-v", args.port], capture_output=True, text=True)
    if owner.returncode != 1:
        raise RuntimeError("serial occupied or ownership check failed: " + owner.stdout + owner.stderr)
    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    exclusive = False
    rows = []
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        exclusive = True
        configure_serial(fd)
        time.sleep(1)
        termios.tcflush(fd, termios.TCIFLUSH)
        started = last_row = next_heartbeat = time.monotonic()
        pending = b""
        pending_version = 1
        request_sent = None
        stop_prompted = False
        version = 1
        print("MONITOR ONLY. Keep right switch UP and yaw stick centered. No motor stop capability.", flush=True)
        while True:
            now = time.monotonic()
            if request_sent is not None:
                if not stop_prompted and now - request_sent >= 1.3:
                    print("RETURN RIGHT SWITCH UP NOW. Downloaded samples are historical, NOT live status.", flush=True)
                    stop_prompted = True
                if now - request_sent > 20:
                    raise RuntimeError("burst download timed out")
            elif now - started > args.wait:
                raise RuntimeError("release not captured before deadline")
            if version == 1 and now - last_row > (3 if not live.stats["records"] else .5):
                raise RuntimeError("live telemetry timeout")
            if not pending and now >= next_heartbeat:
                pending_version = version
                pending = heartbeat(version)
                next_heartbeat = now + .2
            readable, writable, _ = select.select([fd], [fd] if pending else [], [], .01)
            if writable:
                try:
                    pending = pending[os.write(fd, pending):]
                    if not pending and pending_version == 2 and request_sent is None:
                        request_sent = time.monotonic()
                        report["request_sent_host_s"] = request_sent - started
                except BlockingIOError:
                    pass
            if not readable:
                continue
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                continue
            if not chunk:
                raise RuntimeError("serial device disconnected")
            raw.write(chunk)
            decoder = live if version == 1 else burst
            for row in decoder.feed(chunk, time.monotonic() - started):
                writers[decoder.version].writerow(row)
                last_row = time.monotonic()
                if decoder.version == 2:
                    rows.append(row)
                    continue
                previous_state = trigger.state
                if trigger.feed(row):
                    report["release"] = trigger.event
                    version = 2
                    next_heartbeat = 0
                    # Finish any partially sent v1 frame before sending v2.
                    print("RELEASE DETECTED. Burst requested; switch UP within about 1 second, immediately if abnormal.", flush=True)
                if previous_state != trigger.state and trigger.state == "ready":
                    print("SAFE baseline valid. Await operator's confirmed DOWN/manual test, stick centered.", flush=True)
                if previous_state != trigger.state and trigger.state == "wait_excursion":
                    print("Enabled center seen. One brief yaw stick excursion, then release. Switch UP immediately if abnormal.", flush=True)
            if len(rows) >= 512:
                report["release_to_first_burst_ms"] = verify_burst(burst, rows, trigger.event)
                report["buffered_output_enabled_records"] = sum(bool(r["flags"] & 16) for r in rows)
                return
    finally:
        print("End/abort: keep right switch UP. Closing serial does NOT stop motors.", flush=True)
        try:
            if exclusive:
                fcntl.ioctl(fd, termios.TIOCNXCL)
        finally:
            os.close(fd)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--wait", type=float, default=60, help="maximum seconds waiting for release")
    args = parser.parse_args()
    if not math.isfinite(args.wait) or not 5 <= args.wait <= 120:
        parser.error("wait must be 5..120 seconds")
    args.output.mkdir(parents=True, exist_ok=False)
    live, burst = Decoder(1), Decoder(2)
    trigger = ReleaseTrigger()
    report = {"error": None, "port": args.port,
              "commands": "only 0x24 monitor heartbeats A5 01 D1 01 / A5 01 D2 02",
              "limitations": "Host trigger, no high-rate pretrigger. Buffered flags are historical. No motor stop."}
    try:
        with (args.output / "raw.bin").open("xb") as raw, \
                (args.output / "live.csv").open("x", newline="", encoding="utf-8") as lf, \
                (args.output / "burst.csv").open("x", newline="", encoding="utf-8") as bf:
            writers = {1: csv.DictWriter(lf, fieldnames=live.csv_fields),
                       2: csv.DictWriter(bf, fieldnames=burst.csv_fields)}
            for writer in writers.values():
                writer.writeheader()
            collect(args, live, burst, trigger, writers, raw, report)
    except (OSError, RuntimeError, KeyboardInterrupt) as exc:
        report["error"] = type(exc).__name__ + ": " + str(exc)
        print("Capture failed. SWITCH UP; this script cannot stop motors.", flush=True)
    finally:
        for decoder in (live, burst):
            decoder.finish()
            report["v" + str(decoder.version)] = dict(statistics=decoder.stats,
                ranges=decoder.ranges, diagnostics=decoder.diagnostics)
        report["trigger_state"] = trigger.state
        (args.output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report, indent=2))
    return 1 if report["error"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
