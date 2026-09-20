"""Read-only audit of saved serial captures. No serial access or control commands."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

from yaw_slow_protocol import Decoder


def timing_summary(ticks, period_ms, start_ms=None):
    if type(period_ms) is not int or period_ms <= 0:
        raise ValueError('period must be a positive integer')
    if any(type(t) is not int or not 0 <= t <= 0xffffffff for t in ticks):
        raise ValueError('timestamps must be uint32 integers')
    intervals, anomalies = Counter(), []
    duplicates = backwards = wraps = elapsed = 0
    monotonic = True
    max_grid_error = 0
    for i in range(1, len(ticks)):
        delta = (ticks[i]-ticks[i-1]) & 0xffffffff
        intervals[delta] += 1
        duplicates += delta == 0
        backwards += delta >= 0x80000000
        wraps += ticks[i] < ticks[i-1] and 0 < delta < 0x80000000
        if delta != period_ms and len(anomalies) < 20:
            anomalies.append(dict(sample_index=i, previous_tick=ticks[i-1], tick=ticks[i], delta_ms=delta))
        monotonic &= 0 < delta < 0x80000000
        if monotonic:
            elapsed += delta
            max_grid_error = max(max_grid_error, abs(elapsed-i*period_ms))
    return dict(samples=len(ticks), expected_period_ms=period_ms,
                interval_histogram_ms={str(k): v for k, v in sorted(intervals.items())},
                irregular_intervals=sum(n for dt, n in intervals.items() if dt != period_ms),
                duplicate_timestamps=duplicates, backwards_or_ambiguous_intervals=backwards,
                uint32_wraps=wraps, strictly_forward=monotonic if len(ticks) > 1 else None,
                elapsed_ms=elapsed if ticks and monotonic else None,
                max_grid_error_ms=max_grid_error if ticks and monotonic else None,
                first_tick_matches_metadata=None if not ticks or start_ms is None else ticks[0] == start_ms,
                anomaly_examples=anomalies)


def audit(capture, trial_id=None):
    capture = Path(capture)
    stored = json.loads((capture/'report.json').read_text(encoding='utf-8'))
    if not isinstance(stored, dict):
        raise ValueError('report.json must contain an object')
    if trial_id is None:
        trial_id = stored.get('trial_id')
    if type(trial_id) is not int or not 1 <= trial_id <= 0xffffffff:
        raise ValueError('need a valid trial_id in report.json or --trial-id')
    d = Decoder(trial_id)
    digest, parse_error, raw_bytes = hashlib.sha256(), None, 0
    with (capture/'raw.bin').open('rb') as f:
        while chunk := f.read(4096):
            digest.update(chunk)
            raw_bytes += len(chunk)
            if parse_error is None:
                try:
                    d.feed(chunk)
                except ValueError as exc:
                    parse_error = str(exc)
    decoded = d.report()
    initial = d.initial or {}
    timing = timing_summary([r['tick_ms'] for r in d.rows], initial.get('period_ms', 4), initial.get('start_ms'))
    findings = []
    if parse_error:
        findings.append('decoder_rejected_stream')
    if not d.complete:
        findings.append('measurement_download_incomplete')
    if d.discarded_bytes:
        findings.append('bytes_discarded_during_frame_resynchronization')
    if d.buffer:
        findings.append('unparsed_bytes_remaining')
    if d.groups:
        findings.append('incomplete_fragment_groups')
    if timing['irregular_intervals']:
        findings.append('irregular_sample_intervals')
    if timing['first_tick_matches_metadata'] is False:
        findings.append('first_tick_differs_from_metadata')
    mismatches = []
    comparisons = dict(trial_id=trial_id, records=len(d.rows),
                       download_complete=d.complete and parse_error is None,
                       crc_errors=d.crc_errors, firmware_info=d.info)
    for key, value in comparisons.items():
        if key in stored and stored[key] != value:
            mismatches.append(dict(field=key, stored=stored[key], decoded=value))
    if mismatches:
        findings.append('stored_summary_differs_from_raw')
    return dict(diagnostic_only=True, hardware_access=False, trial_id=trial_id,
                raw_sha256=digest.hexdigest(), raw_bytes=raw_bytes,
                bytes_submitted_to_decoder=d.bytes_received, bytes_consumed=d.bytes_consumed,
                discarded_bytes=d.discarded_bytes, pending_bytes=len(d.buffer),
                pending_fragment_groups=len(d.groups), crc_errors=d.crc_errors,
                first_crc_error_offset=d.first_crc_error_offset,
                frame_counts={f'0x{k:02x}': v for k, v in sorted(d.frame_counts.items())},
                parse_error=parse_error, download_complete=d.complete and parse_error is None,
                decoded_quality_issues=decoded['quality_issues'], timing=timing,
                findings=findings, summary_mismatches=mismatches,
                limitations=['MCU sample ticks are not host reception times or CAN completion times.',
                             'Trailing bytes may be a partial status frame; they do not alone prove lost measurements.',
                             'Decoder errors stop parsing; later bytes are hashed but not interpreted.',
                             'This report neither repairs data nor approves a model or hardware operation.'])


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture', type=Path)
    p.add_argument('--trial-id', type=int)
    p.add_argument('--output', type=Path)
    args = p.parse_args(argv)
    try:
        result = audit(args.capture, args.trial_id)
        encoded = json.dumps(result, indent=2, allow_nan=False)
        if args.output is not None:
            with args.output.open('x', encoding='utf-8') as f:
                f.write(encoded+'\n')
    except (OSError, ValueError) as exc:
        p.exit(1, f'Audit failed: {exc}\n')
    print(encoded)
    return 2 if result['findings'] or result['decoded_quality_issues'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
