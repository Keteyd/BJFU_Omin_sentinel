#!/usr/bin/env python3
"""Dual-yaw identification. Default: query only. Trials require explicit consent."""

import argparse
import csv
import json
import os
from pathlib import Path
import time

from yaw_identification_protocol import (
    ARM, BUILD, CANCEL, DOWNLOAD, KEEPALIVE, PROBE, RELEASE, Decoder, FIELDS, PHASES, request,
)


class Serial:
    """Linux exclusive ownership, using the existing 115200-baud configuration."""
    def __init__(self, port, baud=115200):
        if baud not in (115200, 460800):
            raise ValueError('unsupported identification baud rate')
        if os.name != 'posix':
            raise RuntimeError('live serial collection requires Linux on the NUC; use --replay on Windows')
        import fcntl
        import subprocess
        import termios
        from pitch_tune_cli import configure_serial
        self.fd = None
        self.exclusive = False
        result = subprocess.run(['fuser', '-v', port], capture_output=True, text=True)
        if result.returncode != 1:
            raise RuntimeError('port occupied or ownership check failed: ' + result.stdout + result.stderr)
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            self.exclusive = True
            configure_serial(self.fd)
            if baud == 460800:
                attrs = termios.tcgetattr(self.fd)
                attrs[4] = attrs[5] = termios.B460800
                termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            termios.tcflush(self.fd, termios.TCIFLUSH)
        except BaseException:
            self.close()
            raise

    def close(self):
        if self.fd is not None:
            import fcntl
            import termios
            try:
                if self.exclusive:
                    fcntl.ioctl(self.fd, termios.TIOCNXCL)
            finally:
                os.close(self.fd)
                self.fd = None

    def send(self, data):
        import select
        deadline = time.monotonic() + .1
        while data:
            if time.monotonic() > deadline:
                raise RuntimeError('serial write timeout')
            if select.select([], [self.fd], [], .01)[1]:
                try:
                    data = data[os.write(self.fd, data):]
                except BlockingIOError:
                    pass

    def read(self):
        import select
        if not select.select([self.fd], [], [], .01)[0]:
            return b''
        try:
            data = os.read(self.fd, 4096)
        except BlockingIOError:
            return b''
        if not data:
            raise RuntimeError('serial disconnected')
        return data


def safe(status):
    return status is not None and status['flags'] & 12 == 12


def run(args, decoder, raw, events, serial_factory=Serial):
    port = serial_factory(args.port)
    trial_id = args.trial_id
    armed = False
    started = time.monotonic()
    next_beat = started

    def poll(predicate, seconds, beat=PROBE):
        nonlocal next_beat
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_beat:
                port.send(request(beat, trial_id if beat == KEEPALIVE else 0))
                next_beat = now + .1
            chunk = port.read()
            raw.write(chunk)
            for kind, value in decoder.feed(chunk):
                events.write(json.dumps(dict(host_s=now-started, kind=kind, value=value)) + '\n')
            if predicate():
                return
        raise RuntimeError('timed out waiting for firmware; return right switch UP')

    def command(op, axis=0, amplitude=0):
        index = len(decoder.acks)
        port.send(request(op, trial_id, axis, amplitude))
        poll(lambda: any(a['id'] == trial_id and a['op'] == op for a in decoder.acks[index:]),
             2, KEEPALIVE if armed else PROBE)
        ack = next(a for a in decoder.acks[index:] if a['id'] == trial_id and a['op'] == op)
        if ack['status'] != 1:
            raise RuntimeError('command rejected: ' + json.dumps(ack))

    try:
        poll(lambda: decoder.status is not None and decoder.info is not None, 3)
        print('Firmware status:', decoder.status, flush=True)
        if args.action == 'probe':
            if getattr(args, 'remote_detail', False):
                if decoder.status['flags'] & 1:
                    raise RuntimeError('remote diagnostics require unowned firmware; no release is sent')
                if decoder.info['build'] != BUILD:
                    raise RuntimeError('remote diagnostics need the new firmware; no ARM was sent')
                poll(lambda: decoder.remote is not None, 3)
                print('Remote snapshot:', decoder.remote, flush=True)
            return
        if args.action == 'trial':
            if decoder.info['build'] != BUILD:
                raise RuntimeError('trial requires matching current firmware; no ARM was sent')
            if decoder.status['flags'] & 1 or decoder.status['flags'] & 0xfc != 0xfc:
                raise RuntimeError('need unowned firmware, fresh UP/feedback, neutral controls, '
                                   'centered stationary yaw, valid configuration and zero yaw commands')
            trial_id = decoder.status['id'] + 1
            decoder.trial_id = trial_id
            # Mark responsibility before sending ARM: an ACK can be lost after acceptance.
            armed = True
            command(ARM, 1 if args.axis == 'big' else 2, args.amplitude_deg)
            print('ARM accepted. Keep sticks centered. Switch DOWN within 15 seconds to start. '
                  'UP aborts. Pitch holding torque is removed at termination.', flush=True)
            poll(lambda: decoder.status['id'] == trial_id and decoder.status['phase'] in (5, 6),
                 22, KEEPALIVE)
            print('Trial:', PHASES[decoder.status['phase']], 'reason:', decoder.status['reason'],
                  'Return right switch UP now; leave it UP.', flush=True)
            poll(lambda: decoder.status['id'] == trial_id and safe(decoder.status), 45)
        else:
            if decoder.status['id'] != trial_id or not decoder.status['flags'] & 1:
                raise RuntimeError('trial ID does not match the owned firmware')
            decoder.trial_id = trial_id
            if args.action == 'cancel':
                command(CANCEL)
                return
            if not safe(decoder.status) or decoder.status['phase'] not in (5, 6):
                raise RuntimeError('download/release requires terminal state and fresh remote UP')
            if args.action == 'release':
                command(RELEASE)
                return
        command(DOWNLOAD)
        poll(lambda: decoder.complete, 45)
        print('Download complete. Ownership remains locked; no automatic return to normal control.', flush=True)
    finally:
        if armed and (decoder.status is None or decoder.status['id'] != trial_id or
                      decoder.status['phase'] not in (5, 6)):
            try:
                port.send(request(CANCEL, trial_id))
            except (OSError, RuntimeError):
                pass  # Firmware's independent lease expiry remains the fallback.
        port.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    source = p.add_mutually_exclusive_group(required=True)
    source.add_argument('--port')
    source.add_argument('--replay', type=Path)
    p.add_argument('--action', choices=('probe', 'trial', 'download', 'cancel', 'release'), default='probe')
    p.add_argument('--axis', choices=('big', 'small'))
    p.add_argument('--amplitude-deg', type=float, default=1)
    p.add_argument('--trial-id', type=int)
    p.add_argument('--confirm', help='trial: FIXED_CHASSIS_CLEAR_YAW; release: REMOTE_UP')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--remote-detail', action='store_true', help='probe only: wait for per-input neutral diagnostics')
    args = p.parse_args()
    if args.remote_detail and (args.action != 'probe' or args.replay):
        p.error('--remote-detail is only for live --action probe')
    if args.replay:
        if not args.trial_id:
            p.error('replay needs the expected --trial-id')
    elif args.action == 'trial':
        if not args.axis or args.confirm != 'FIXED_CHASSIS_CLEAR_YAW':
            p.error('trial needs --axis and --confirm FIXED_CHASSIS_CLEAR_YAW')
        try:
            request(ARM, 1, 1, args.amplitude_deg)
        except ValueError as exc:
            p.error(str(exc))
    elif args.action != 'probe':
        if not args.trial_id or not 0 < args.trial_id <= 0xffffffff:
            p.error('recovery commands need --trial-id')
        if args.action == 'release' and args.confirm != 'REMOTE_UP':
            p.error('release needs --confirm REMOTE_UP')
    args.output.mkdir(parents=True, exist_ok=False)
    setup = json.loads((Path(__file__).parent / 'mpc/identification_setup.json').read_text(encoding='utf-8'))
    (args.output / 'setup.json').write_text(json.dumps(setup, indent=2), encoding='utf-8')
    decoder = Decoder(args.trial_id)
    error = None
    try:
        if args.replay:
            with args.replay.open('rb') as raw:
                while chunk := raw.read(4096):
                    decoder.feed(chunk)
            if not decoder.complete:
                raise RuntimeError('incomplete replay')
        else:
            with (args.output / 'raw.bin').open('xb') as raw, (args.output / 'events.jsonl').open('x', encoding='utf-8') as events:
                run(args, decoder, raw, events)
    except (OSError, ValueError, RuntimeError, KeyboardInterrupt) as exc:
        error = type(exc).__name__ + ': ' + str(exc)
    finally:
        with (args.output / 'samples.csv').open('x', newline='', encoding='utf-8') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerows(decoder.rows)
        data_action = bool(args.replay or args.action in ('trial', 'download'))
        report = decoder.report() if data_action else dict(download_requested=False, model_status='not_identified')
        report.update(error=error, action=args.action, last_status=decoder.status, firmware_info=decoder.info,
                      trial_id=decoder.trial_id, metadata=decoder.metadata, remote_diagnostics=decoder.remote)
        if decoder.status is not None:
            flags = decoder.status['flags']
            report['checks'] = {name: bool(flags & (1 << bit)) for bit, name in enumerate((
                'owned', 'yaw_permitted', 'remote_up_fresh', 'yaw_commands_zero',
                'yaw_imu_feedback_fresh', 'operator_neutral', 'yaw_centered_stationary', 'configuration_valid'))}
        (args.output / 'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(report, indent=2))
    return 1 if error else (2 if data_action and report['quality_issues'] else 0)


if __name__ == '__main__':
    raise SystemExit(main())
