"""Strict 0x59490203 streaming decoder. No hardware access or automatic commands."""

import math
import struct

from yaw_capture_protocol import crc8, frame
from yaw_identification_protocol import FIELDS, MAGIC, META, RECORD, VALUES

BUILD = 0x59490203
PROBE, KEEPALIVE, CANCEL, RELEASE = 0, 2, 3, 5
ARM_SLOW, ARM_REVERSE, BENCH, RECEIPT = 6, 7, 8, 9
PROFILE = struct.Struct('<III4H8H6f4B')
KNOTS = (0, 2000, 4000, 7000, 11000, 14000, 16000, 20000)


def request(op=PROBE, trial_id=0, axis=0, value=0):
    if (op not in (PROBE, KEEPALIVE, CANCEL, RELEASE, ARM_SLOW, ARM_REVERSE, BENCH, RECEIPT)
            or type(trial_id) is not int or not 0 <= trial_id <= 0xffffffff
            or type(value) is not int or type(axis) is not int):
        raise ValueError('invalid operation, ID or integer payload')
    if op != PROBE and not trial_id:
        raise ValueError('non-query command needs a nonzero trial ID')
    if op in (ARM_SLOW, ARM_REVERSE):
        if axis not in (1, 2) or not 1 <= value <= (1500 if axis == 1 else 1000):
            raise ValueError('slow target exceeds big15/small10-degree profile')
    elif op == RECEIPT:
        if axis or not 1 <= value <= 5001:
            raise ValueError('invalid cumulative record receipt')
    elif axis or value:
        raise ValueError('command requires zero axis/value')
    return frame(0x36, struct.pack('<IHBBI', trial_id, value, op, axis, MAGIC))


class Decoder:
    BUILD, STREAM_VERSION, CAPACITY, RECORD_BYTES = BUILD, 2, 1001, 48
    MAX_RECORDS = EXPECTED_RECORDS = 5001
    FIRST_TICK_RANGE_MS = (0, 0)
    TICK_INTERVAL_RANGE_MS = (4, 4)
    FIELDS = FIELDS
    def __init__(self, trial_id=None, expected=None):
        self.trial_id, self.expected = trial_id, expected
        self.buffer = bytearray()
        self.groups = {}
        self.status = self.info = self.profile = self.initial = self.metadata = None
        self.acks, self.rows = [], []
        self.crc_errors = 0
        self.issues = set()
        self.bytes_received = self.bytes_consumed = self.discarded_bytes = 0
        self.frame_counts = {}
        self.first_crc_error_offset = None

    def feed(self, chunk):
        self.bytes_received += len(chunk)
        self.buffer.extend(chunk)
        while len(self.buffer) >= 16:
            if self.buffer[0] != 255 or self.buffer[15] != 13:
                del self.buffer[0]
                self.discarded_bytes += 1
                self.bytes_consumed += 1
                continue
            packet = bytes(self.buffer[:16])
            if crc8(packet[:14]) != packet[14]:
                self.crc_errors += 1
                if self.first_crc_error_offset is None:
                    self.first_crc_error_offset = self.bytes_consumed
                raise ValueError('serial CRC error; receipt stopped')
            del self.buffer[:16]
            self.bytes_consumed += 16
            cmd, data = packet[1], packet[2:14]
            self.frame_counts[cmd] = self.frame_counts.get(cmd, 0)+1
            if cmd == 0x37:
                keys = ('id', 'count', 'phase', 'reason', 'flags', 'axis', 'version', 'extra')
                status = dict(zip(keys, struct.unpack('<IH6B', data)))
                if status['version'] != 1 or status['phase'] > 6 or status['count'] > self.MAX_RECORDS:
                    raise ValueError('unsupported status format')
                self.status = status
            elif cmd == 0x3b:
                self.info = dict(zip(('build', 'magic', 'version', 'record_bytes'), struct.unpack('<IIHH', data)))
                if tuple(self.info.values()) != (self.BUILD, MAGIC, 1, self.RECORD_BYTES):
                    raise ValueError(f'need firmware 0x{self.BUILD:08X} at 460800; no automatic fallback')
            elif cmd == 0x38:
                self.acks.append(dict(zip(('id', 'op', 'status', 'phase', 'reason', 'tick_ms'),
                                         struct.unpack('<IBBBBI', data))))
            elif cmd in (0x39, 0x3d, 0x3e) and self.trial_id is not None:
                self._fragment(cmd, data)

    def _fragment(self, cmd, data):
        seq, part, version = struct.unpack_from('<HBB', data)
        count = {0x39: 24, 0x3d: 6, 0x3e: 8}[cmd]
        if (version != self.STREAM_VERSION or part >= count or (cmd == 0x39 and seq not in (0, 1))
                or (cmd == 0x3e and seq != 0)
                or (cmd == 0x3d and seq != len(self.rows)+1)):
            raise ValueError('wrong fragment version/sequence; receipt stopped')
        key = (cmd, seq)
        pending = self.groups.setdefault(key, bytearray())
        if part != len(pending)//8:
            raise ValueError('missing or duplicate fragment; receipt stopped')
        pending.extend(data[4:])
        if len(pending) != count*8:
            return
        raw = bytes(self.groups.pop(key))
        if cmd == 0x3e:
            self._profile(raw)
        elif cmd == 0x39:
            self._metadata(seq, raw)
        else:
            self._row(raw)

    def _profile(self, raw):
        v = PROFILE.unpack(raw)
        p = dict(zip(('id', 'build', 'baud', 'samples', 'period_ms', 'capacity', 'duration_ms'), v[:7]))
        p.update(knots_ms=list(v[7:15]), big_peak=v[15], small_peak=v[16], big_travel=v[17],
                 small_travel=v[18], heading_travel=v[19], center_deg=v[20],
                 profile=v[21], reverse=v[22], version=v[23], reserved=v[24])
        if (tuple(v[:7]) != (self.trial_id, self.BUILD, 460800, 5001, 4, self.CAPACITY, 20000)
                or tuple(v[7:15]) != KNOTS or tuple(v[15:21]) != (15., 10., 25., 20., 20., 10.)
                or p['profile'] not in (1, 2) or p['reverse'] not in (0, 1)
                or (p['profile'] == 2 and p['reverse']) or v[23:] != (self.STREAM_VERSION, 0)):
            raise ValueError('unexpected profile/limits/build')
        if self.expected and (p['profile'], p['reverse']) != self.expected[:2]:
            raise ValueError('firmware started a different profile')
        if self.profile is not None and self.profile != p:
            raise ValueError('profile changed midstream')
        self.profile = p

    def _metadata(self, seq, raw):
        if self.profile is None:
            raise ValueError('metadata arrived before profile')
        v = META.unpack(raw)
        m = dict(zip(('id', 'start_ms', 'build', 'count', 'period_ms', 'phase',
                      'reason', 'axis', 'version', 'setup'), v[:10]))
        m['configuration'] = dict(zip(VALUES, v[10:]))
        c = m['configuration']
        if (m['id'] != self.trial_id or m['build'] != self.BUILD or m['period_ms'] != 4
                or m['version'] != self.STREAM_VERSION or m['setup'] != 1 or m['count'] > 5001
                or not all(math.isfinite(x) for x in v[10:])):
            raise ValueError('invalid metadata')
        bench = self.profile['profile'] == 2
        if (bench and (m['axis'] != 0 or c['amplitude_deg'] != 0)) or (
                not bench and (m['axis'] not in (1, 2) or not 0 < c['amplitude_deg'] <= (15 if m['axis'] == 1 else 10))):
            raise ValueError('metadata axis/amplitude mismatch')
        if self.expected and (m['axis'] != self.expected[2] or abs(c['amplitude_deg']-self.expected[3]) > 1e-4):
            raise ValueError('firmware amplitude differs from requested target')
        if not 0 < c['big_effort_limit'] <= 30 or not 0 < c['small_effort_limit'] <= 6:
            raise ValueError('unexpected output limits')
        if seq == 0 and (m['count'] or m['phase'] != 2 or m['reason']):
            raise ValueError('invalid initial metadata')
        if seq == 1 and (m['phase'] not in (5, 6) or m['count'] < len(self.rows)):
            raise ValueError('invalid terminal metadata')
        frozen = lambda item: {k: x for k, x in item.items() if k not in ('count', 'phase', 'reason')}
        if self.initial and frozen(self.initial) != frozen(m):
            raise ValueError('configuration or anchors changed')
        if self.initial is None:
            self.initial = m
        if seq == 1:
            if self.metadata is not None and self.metadata != m:
                raise ValueError('terminal metadata changed')
            self.metadata = m
            # An early abort may interrupt the initial metadata transmission.
            self.groups.pop((0x39, 0), None)

    def _row(self, raw, profile_elapsed_ms=None):
        if self.initial is None or self.profile is None:
            raise ValueError('record arrived before configuration')
        r = dict(zip(self.FIELDS, RECORD.unpack(raw)))
        if (len(self.rows) >= self.MAX_RECORDS or
                (self.metadata and len(self.rows) >= self.metadata['count'])
                or not all(math.isfinite(r[k]) for k in FIELDS[1:7])
                or r['big_raw'] > 8191 or r['small_raw'] > 8191 or r['phase'] not in (2, 3, 4, 5, 6)):
            raise ValueError('invalid streamed measurement')
        if self.rows and self.rows[-1]['phase'] in (5, 6):
            raise ValueError('records follow terminal sample')
        elapsed = (r['tick_ms']-self.initial['start_ms']) & 0xffffffff
        if profile_elapsed_ms is None:
            profile_elapsed_ms = elapsed
        tick_interval = ((r['tick_ms']-self.rows[-1]['tick_ms']) & 0xffffffff) if self.rows else None
        if (not self.rows and not self.FIRST_TICK_RANGE_MS[0] <= elapsed <= self.FIRST_TICK_RANGE_MS[1]) or (self.rows and not
                self.TICK_INTERVAL_RANGE_MS[0] <= tick_interval <= self.TICK_INTERVAL_RANGE_MS[1]):
            self.issues.add('irregular_sample_interval')
        if r['flags'] & (1 << 11):
            self.issues.add('irregular_sample_interval')
        terminal = r['phase'] in (5, 6)
        bench = self.profile['profile'] == 2
        if not terminal:
            phase = self._expected_phase(profile_elapsed_ms)
            if r['phase'] != phase:
                raise ValueError('waveform phase differs from profile')
            required, forbidden = (3, 20) if bench else (29, 2)
            if r['flags'] & required != required or r['flags'] & forbidden:
                raise ValueError('remote/output state inconsistent with profile')
        if (bench and (r['big_command_raw'] or r['small_command_raw'] or r['offset_cdeg'])):
            raise ValueError('nonzero command/reference in bench sample')
        if terminal and (r['big_command_raw'] or r['small_command_raw']):
            raise ValueError('nonzero command in terminal sample')
        # A normal completion occurs after the reference has returned to zero.
        # An abort preserves the last scheduled reference for diagnosis while
        # the actual motor commands must already be zero.
        if r['phase'] == 5 and r['offset_cdeg']:
            raise ValueError('nonzero reference in completed terminal sample')
        if not terminal and (not r['flags'] & 1 or r['imu_age_ms'] > 10
                             or max(r['big_age_ms'], r['small_age_ms']) > 20):
            raise ValueError('stale feedback while enabled')
        c = self.initial['configuration']
        if abs(r['big_command_raw']) > c['big_effort_limit']*1000+.01 or abs(r['small_command_raw']) > c['small_effort_limit']*1000+.01:
            raise ValueError('recorded command exceeds frozen output limit')
        if r['flags'] & ((31 << 5) | (1 << 10)):
            self.issues.add('limit_or_saturation')
        self.rows.append(r)

    def _expected_phase(self, profile_elapsed_ms):
        return (2 if profile_elapsed_ms < 2000 else 3 if profile_elapsed_ms < 16000
                else 4 if profile_elapsed_ms < 20000 else -1)

    @property
    def complete(self):
        return (self.metadata is not None and len(self.rows) == self.metadata['count']
                and not self.groups)

    def report(self):
        issues = set(self.issues)
        failure = None
        if self.metadata and self.metadata['reason'] == 3 and self.rows:
            last = self.rows[-1]
            causes = [name for bit, name in enumerate(
                ('ins_not_ready', 'big_motor_offline', 'small_motor_offline', 'invalid_feedback_or_mode'), 12)
                if last['flags'] & (1 << bit)]
            for key, limit in (('imu_age_ms', 10), ('big_age_ms', 20), ('small_age_ms', 20)):
                if last[key] > limit:
                    causes.append(key + '_expired')
            failure = dict(causes=causes, imu_age_ms=last['imu_age_ms'],
                           big_age_ms=last['big_age_ms'], small_age_ms=last['small_age_ms'],
                           snapshot='abort observation; terminal angles remain post-stop samples')
        if not self.complete:
            issues.add('incomplete_stream')
        if len(self.rows) != self.EXPECTED_RECORDS:
            issues.add('sample_count_not_%d' % self.EXPECTED_RECORDS)
        if self.metadata is None or self.metadata['phase'] != 5 or self.metadata['reason']:
            issues.add('trial_not_completed')
        if self.rows and self.metadata and self.rows[-1]['phase'] != self.metadata['phase']:
            issues.add('terminal_sample_mismatch')
        if self.crc_errors:
            issues.add('serial_crc_errors')
        return dict(download_complete=self.complete, records=len(self.rows), quality_issues=sorted(issues),
                    crc_errors=self.crc_errors, model_status='not_identified',
                    hardware_takeover_allowed=False, profile=self.profile, feedback_failure=failure,
                    metadata=self.metadata, initial_metadata=self.initial,
                    output_units='software voltage command integer, not measured torque or CAN acknowledgment')
