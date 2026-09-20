#!/usr/bin/env python3
"""460800-baud slow identification. Default is query only; no automatic release."""

import argparse
import csv
import json
import math
from pathlib import Path
import time

from yaw_identification_cli import Serial
from yaw_slow_protocol import (ARM_REVERSE, ARM_SLOW, BENCH, BUILD, CANCEL, Decoder,
                               FIELDS, KEEPALIVE, PROBE, RECEIPT, RELEASE, request)


def remote_safe(status):
    return status is not None and status['flags'] & 12 == 12


def run(args, decoder, raw, events, serial_factory=Serial, clock=time.monotonic):
    request_fn = getattr(decoder, 'REQUEST_FN', request)
    phase_ops = getattr(decoder, 'PHASE_OPS', {'A': 10, 'B': 11})
    if getattr(decoder, 'REQUEST_FN', None) is not None:
        pass
    elif getattr(decoder, 'STREAM_VERSION', 0) >= 5:
        from yaw_cd_trace_protocol import request as request_fn
    elif getattr(decoder, 'STREAM_VERSION', 0) >= 4:
        from yaw_dual_trace_protocol import request as request_fn
    port = serial_factory(args.port, baud=460800)
    trial_id = None
    responsible = False
    next_beat = next_receipt = 0.
    last_receipt = 0
    result = {'action': args.action, 'bench_passed': False}

    def poll(predicate, timeout):
        nonlocal next_beat, next_receipt, last_receipt
        deadline = clock()+timeout
        while clock() < deadline:
            now = clock()
            if now >= next_beat:
                port.send(request_fn(KEEPALIVE, trial_id) if responsible else request_fn(PROBE))
                next_beat = now+.1
            chunk = port.read()
            raw.write(chunk)
            raw.flush()
            before = decoder.status
            decoder.feed(chunk)
            if decoder.status is not before:
                events.write(json.dumps(dict(host_s=now, status=decoder.status))+'\n')
                events.flush()
                if responsible and (decoder.status['id'] != trial_id or not decoder.status['flags'] & 1):
                    raise RuntimeError('ownership changed or MCU restarted during capture')
            # Parsing must succeed for the whole chunk before advancing cumulative receipt.
            if responsible and now >= next_receipt and len(decoder.rows) > last_receipt:
                port.send(request_fn(RECEIPT, trial_id, value=len(decoder.rows)))
                last_receipt = len(decoder.rows)
                next_receipt = now+.02
            if predicate():
                return
        raise RuntimeError('timeout: keep/return right switch UP; no automatic retry')

    def command(op, axis=0, value=0):
        start = len(decoder.acks)
        packet = request_fn(op, trial_id, axis, value)
        port.send(packet)
        poll(lambda: any(a['id'] == trial_id and a['op'] == op for a in decoder.acks[start:]), 2)
        ack = next(a for a in decoder.acks[start:] if a['id'] == trial_id and a['op'] == op)
        if ack['status'] != 1:
            raise RuntimeError('firmware rejected command: '+json.dumps(ack))

    try:
        poll(lambda: decoder.status is not None and decoder.info is not None, 4)
        print('Firmware 0x%08X, 460800. Status:' % decoder.BUILD, decoder.status, flush=True)
        if args.action == 'probe':
            return result
        status = decoder.status
        if args.action in ('release', 'cancel'):
            trial_id = args.trial_id if args.trial_id is not None else status['id']
            if not status['flags'] & 1 and args.action == 'release':
                print('Already unowned; no RELEASE sent.', flush=True)
                result['already_unowned'] = True
                return result
            if trial_id != status['id'] or not status['flags'] & 1:
                raise RuntimeError('trial ID does not match current ownership')
            if args.action == 'release' and (not remote_safe(status) or status['phase'] not in (5, 6)):
                raise RuntimeError('release needs terminal phase, fresh UP and zero yaw outputs')
            command(RELEASE if args.action == 'release' else CANCEL)
            result['trial_id'] = trial_id
            return result
        if status['flags'] & 1:
            raise RuntimeError('need unowned firmware before starting a new capture')
        trial_id = status['id']+1
        bench = args.action == 'bench'
        dual = args.action == 'dual'
        axis = 0 if bench else getattr(decoder, 'PROFILE_AXIS', 3) if dual else (1 if args.axis == 'big' else 2)
        value = 0 if bench or dual else round(args.amplitude_deg*100)
        if bench:
            op = BENCH
        elif dual:
            op = phase_ops[args.phase_set]
        else:
            op = ARM_REVERSE if args.reverse else ARM_SLOW
        decoder.trial_id = trial_id
        phase_values = getattr(decoder, 'PHASE_VALUES', {})
        decoder.expected = (2 if bench else getattr(decoder, 'PROFILE_ID', 3) if dual else 1,
                            (phase_values.get(args.phase_set, list(phase_ops).index(args.phase_set))
                             if dual else int(args.reverse if not bench else False)),
                            axis, value/100)
        # Responsibility starts before the ARM packet: acceptance ACK may be lost.
        responsible = True
        next_beat = 0.
        command(op, axis, value)
        print('Optional BENCH accepted: stay UP for 20s. Both yaw and Pitch outputs remain OFF.' if bench else
              ('DUAL %s accepted: operator owns setup checks; switch DOWN within 15s. UP aborts.' % args.phase_set
               if dual else 'ARM accepted: operator owns setup checks; switch DOWN within 15s. UP aborts.'), flush=True)
        print('Pitch has no holding torque at termination. Do not release support unexpectedly.', flush=True)
        poll(lambda: decoder.complete, 25 if bench else getattr(decoder, 'LIVE_TIMEOUT_S', 41))
        if len(decoder.rows) > last_receipt:
            raw.flush()
            port.send(request_fn(RECEIPT, trial_id, value=len(decoder.rows)))
            last_receipt = len(decoder.rows)
        quality_ok = not decoder.report()['quality_issues']
        poll(lambda: decoder.status['id'] == trial_id and decoder.status['phase'] in (5, 6), 3)
        result.update(trial_id=trial_id,
                      bench_passed=bool(bench and quality_ok and decoder.metadata and
                                        decoder.metadata['phase'] == 5 and not decoder.metadata['reason']))
        print('Optional BENCH capture passed data checks.' if result['bench_passed'] else
              'Capture finished: phase=%s reason=%s.' % (decoder.metadata['phase'], decoder.metadata['reason']), flush=True)
        print('Keep/return right switch UP. Ownership remains locked. No RELEASE or next trial was sent.', flush=True)
        return result
    finally:
        if responsible:
            # A terminal decoder report is enough to avoid an unnecessary CANCEL
            # after a complete capture.
            terminal = decoder.metadata is not None and decoder.metadata['phase'] in (5, 6)
            if not terminal:
                try:
                    port.send(request_fn(CANCEL, trial_id))
                except (OSError, RuntimeError, ValueError):
                    pass
        port.close()


def parse_args(argv=None, phase_sets=('A', 'B'),
               dual_confirm='DUAL_REFERENCE_FIXED_CHASSIS_CLEAR_YAW',
               actions=('probe', 'bench', 'trial', 'dual', 'cancel', 'release')):
    p = argparse.ArgumentParser(description=__doc__)
    source = p.add_mutually_exclusive_group(required=True)
    source.add_argument('--port')
    source.add_argument('--replay', type=Path)
    p.add_argument('--action', choices=actions, default='probe')
    p.add_argument('--trial-id', type=int)
    p.add_argument('--axis', choices=('big', 'small'))
    p.add_argument('--amplitude-deg', type=float)
    p.add_argument('--reverse', action='store_true')
    p.add_argument('--phase-set', choices=phase_sets)
    p.add_argument('--confirm')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args(argv)
    if args.trial_id is not None and not 1 <= args.trial_id <= 0xffffffff:
        p.error('trial ID outside uint32 range')
    if args.replay:
        if args.trial_id is None:
            p.error('replay requires --trial-id')
    elif args.action == 'bench':
        if args.confirm != 'BENCH_REMOTE_UP':
            p.error('bench requires --confirm BENCH_REMOTE_UP; stay UP throughout')
    elif args.action == 'trial':
        if args.confirm != 'FIXED_CHASSIS_CLEAR_YAW' or args.axis is None or args.amplitude_deg is None:
            p.error('trial requires axis, amplitude and --confirm FIXED_CHASSIS_CLEAR_YAW')
        if not math.isfinite(args.amplitude_deg) or not .01 <= args.amplitude_deg <= (15 if args.axis == 'big' else 10):
            p.error('amplitude outside approved profile')
    elif args.action == 'dual':
        if args.confirm != dual_confirm or args.phase_set is None:
            p.error('dual requires --phase-set %s and --confirm %s' %
                    ('/'.join(phase_sets), dual_confirm))
    elif args.action == 'release' and args.confirm != 'REMOTE_UP':
        p.error('release requires --confirm REMOTE_UP')
    if args.action != 'trial' and (args.axis or args.amplitude_deg is not None or args.reverse):
        p.error('axis/amplitude/reverse options are only for trial')
    if args.action != 'dual' and args.phase_set is not None:
        p.error('phase-set is only for dual')
    return args


def main(argv=None, decoder_class=Decoder, fields=FIELDS, phase_sets=('A', 'B'),
         dual_confirm='DUAL_REFERENCE_FIXED_CHASSIS_CLEAR_YAW',
         actions=('probe', 'bench', 'trial', 'dual', 'cancel', 'release')):
    args = parse_args(argv, phase_sets=phase_sets, dual_confirm=dual_confirm,
                      actions=actions)
    args.output.mkdir(parents=True, exist_ok=False)
    setup_path = Path(__file__).parent/'mpc/identification_setup.json'
    setup = json.loads(setup_path.read_text(encoding='utf-8'))
    (args.output/'setup.json').write_text(json.dumps(setup, indent=2), encoding='utf-8')
    decoder = decoder_class(args.trial_id if args.replay else None)
    result, error = {}, None
    try:
        if args.replay:
            with args.replay.open('rb') as f:
                while chunk := f.read(4096):
                    decoder.feed(chunk)
        else:
            with (args.output/'raw.bin').open('xb') as raw, (args.output/'events.jsonl').open('x', encoding='utf-8') as events:
                result = run(args, decoder, raw, events)
    except (OSError, ValueError, RuntimeError, KeyboardInterrupt) as exc:
        error = type(exc).__name__+': '+str(exc)
    finally:
        with (args.output/'samples.csv').open('x', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader(); writer.writerows(decoder.rows)
        # A rejected live preflight has no trial and therefore no data stream to
        # grade.  Reporting it as an incomplete capture obscures the actual
        # precondition error printed below.
        data_action = bool(args.replay or
                           (args.action in ('bench', 'trial', 'dual') and
                            decoder.trial_id is not None))
        report = decoder.report() if data_action else {'model_status': 'not_identified'}
        report.update(result, action=args.action, replay=bool(args.replay), error=error,
                      trial_id=decoder.trial_id if data_action else result.get('trial_id'),
                      last_status=decoder.status, firmware_info=decoder.info)
        (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
        print(json.dumps({k: v for k, v in report.items() if k not in ('metadata', 'initial_metadata')}, indent=2), flush=True)
    return 1 if error else 2 if data_action and report['quality_issues'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
