"""Identification v1: strict offline decoder, no serial or actuator access."""

import math
import struct

from yaw_capture_protocol import crc8, frame

MAGIC = 0x31444959
BUILD = 0x59490106
SUPPORTED_BUILDS = (0x59490101, 0x59490102, 0x59490103, 0x59490104, 0x59490105, BUILD)
PROBE, ARM, KEEPALIVE, CANCEL, DOWNLOAD, RELEASE = range(6)
PHASES = ('idle', 'armed', 'baseline', 'excite', 'settle', 'done', 'aborted')
RECORD = struct.Struct('<I6f2H5hH4B')
META = struct.Struct('<IIIHHBBBBI42f')
REMOTE = struct.Struct('<I7hBBHH')
REMOTE_FIELDS = ('tick_ms ch0 ch1 ch2 ch3 ch4 mouse_x mouse_y mouse_left mouse_right age_ms failed_mask').split()
NEUTRAL_INPUTS = ('ch0 ch1 ch2 ch3 ch4 mouse_x mouse_y mouse_left mouse_right').split()
FIELDS = ('tick_ms yaw_deg gyro_x_rad_s gyro_y_rad_s gyro_z_rad_s roll_deg '
          'pitch_deg big_raw small_raw big_rpm small_rpm big_command_raw '
          'small_command_raw offset_cdeg flags imu_age_ms big_age_ms small_age_ms phase').split()
CONFIG = ('big_angle_kp big_speed_kp big_effort_limit big_speed_filter_tau_s '
          'big_angle_ki big_angle_kd big_speed_ki big_speed_kd '
          'small_angle_kp small_speed_kp small_effort_limit small_speed_filter_alpha '
          'small_angle_ki small_angle_kd small_speed_ki small_speed_kd '
          'small_speed_limit_rpm small_manual_step_deg small_direction big_direction '
          'pitch_imu_kp pitch_imu_kd pitch_gravity_effort pitch_rate_limit_rad_s '
          'pitch_angle_deadband_rad pitch_rate_deadband_rad_s pitch_imu_enable '
          'pitch_mit_kp pitch_mit_kd yaw_control_state pitch_control_state small_enable').split()
VALUES = ('amplitude_deg big_anchor_deg small_heading_anchor_deg small_joint_start_deg '
          'pitch_target_rad imu_roll_start_deg imu_pitch_start_deg small_zero_deg '
          'small_min_deg small_max_deg').split() + CONFIG


def request(op=PROBE, trial_id=0, axis=0, amplitude_deg=0):
    if op not in range(6) or not isinstance(trial_id, int) or not 0 <= trial_id <= 0xffffffff:
        raise ValueError('invalid operation or trial ID')
    if not math.isfinite(amplitude_deg):
        raise ValueError('nonfinite amplitude')
    cdeg = round(amplitude_deg * 100)
    if op == ARM:
        if not trial_id or axis not in (1, 2) or not .01 <= amplitude_deg <= 5 or not 1 <= cdeg <= 500:
            raise ValueError('ARM needs ID, axis 1/2 and amplitude .01..5 degrees')
    elif axis or amplitude_deg or (op != PROBE and not trial_id):
        raise ValueError('non-ARM command must have zero axis/amplitude and a matching ID')
    return frame(0x36, struct.pack('<IHBBI', trial_id, cdeg, op, axis, MAGIC))


class Decoder:
    def __init__(self, trial_id=None):
        self.trial_id = trial_id
        self.buffer = bytearray()
        self.partial = bytearray()
        self.part = 0
        self.sequence = 0
        self.metadata = None
        self.rows = []
        self.status = None
        self.info = None
        self.remote = None
        self.remote_partial = bytearray()
        self.remote_sequence = None
        self.remote_part = 0
        self.remote_version = None
        self.acks = []
        self.crc_errors = 0
        self.quality_issues = set()

    def feed(self, chunk):
        self.buffer.extend(chunk)
        events = []
        while len(self.buffer) >= 16:
            if self.buffer[0] != 255 or self.buffer[15] != 13:
                del self.buffer[0]
                continue
            packet = bytes(self.buffer[:16])
            if crc8(packet[:14]) != packet[14]:
                self.crc_errors += 1
                del self.buffer[0]
                continue
            del self.buffer[:16]
            cmd, data = packet[1], packet[2:14]
            if cmd == 0x37:
                values = struct.unpack('<IHBBBBH', data)
                self.status = dict(zip(('id', 'count', 'phase', 'reason', 'flags', 'axis', 'version'), values))
                if self.status['version'] != 1 or self.status['phase'] not in range(7):
                    raise ValueError('unsupported identification status')
                events.append(('status', self.status))
            elif cmd == 0x38:
                ack = dict(zip(('id', 'op', 'status', 'phase', 'reason', 'tick_ms'),
                               struct.unpack('<IBBBBI', data)))
                self.acks.append(ack)
                events.append(('ack', ack))
            elif cmd == 0x3b:
                self.info = dict(zip(('build', 'magic', 'version', 'record_bytes'),
                                     struct.unpack('<IIHH', data)))
                if (self.info['build'] not in SUPPORTED_BUILDS or
                        tuple(self.info.values())[1:] != (MAGIC, 1, 48)):
                    raise ValueError('firmware build/protocol mismatch; do not ARM')
                events.append(('info', self.info))
            elif cmd == 0x3c:
                result = self._remote_part(data)
                if result is not None:
                    events.append(('remote', result))
            elif cmd in (0x39, 0x3a):
                self._part(cmd, data)
        return events

    def _remote_part(self, data):
        seq, part, version = struct.unpack_from('<HBB', data)
        if version not in (1, 2) or part >= 3:
            raise ValueError('unsupported remote diagnostic fragment')
        if part == 0:
            self.remote_partial.clear()
            self.remote_sequence = seq
            self.remote_part = 0
            self.remote_version = version
        if (seq != self.remote_sequence or part != self.remote_part or
                version != self.remote_version):
            raise ValueError('missing or mixed remote diagnostic fragment')
        self.remote_partial.extend(data[4:])
        self.remote_part += 1
        if self.remote_part == 3:
            result = dict(zip(REMOTE_FIELDS, REMOTE.unpack(self.remote_partial)))
            values = [result[k] for k in NEUTRAL_INPUTS]
            failures = [abs(value) > 10 if i < 5 else value != 0
                        for i, value in enumerate(values)]
            if version == 2:
                failures[4] = result['ch4'] > 100
            expected = sum(1 << i for i, failed in enumerate(failures) if failed)
            if result['failed_mask'] != expected:
                raise ValueError('remote diagnostic mask disagrees with snapshot')
            result['sequence'] = seq
            result['version'] = version
            result['failed_inputs'] = [k for i, k in enumerate(NEUTRAL_INPUTS) if expected & (1 << i)]
            self.remote = result
            self.remote_partial.clear()
            self.remote_sequence = None
            self.remote_part = 0
            return result
        return None

    def _part(self, cmd, data):
        seq, part, version = struct.unpack_from('<HBB', data)
        if version != 1 or seq != self.sequence or part != self.part:
            raise ValueError('missing, duplicate, reordered or mixed capture fragment')
        expected_cmd = 0x39 if self.sequence == 0 else 0x3a
        if cmd != expected_cmd:
            raise ValueError('unexpected capture group')
        self.partial.extend(data[4:])
        self.part += 1
        if self.part != (24 if seq == 0 else 6):
            return
        if seq == 0:
            values = META.unpack(self.partial)
            self.metadata = dict(zip(('id', 'start_ms', 'build', 'count', 'period_ms',
                                      'phase', 'reason', 'axis', 'version', 'setup'), values[:10]))
            m = self.metadata
            if (self.trial_id is None or m['id'] != self.trial_id or m['build'] not in SUPPORTED_BUILDS or
                    m['version'] != 1 or m['setup'] != 1 or m['period_ms'] != 4 or
                    m['phase'] not in (5, 6) or m['count'] > 1001 or m['axis'] not in (1, 2)):
                raise ValueError('metadata does not match expected trial/build/setup')
            if not all(math.isfinite(v) for v in values[10:]):
                raise ValueError('nonfinite trial configuration')
            m['configuration'] = dict(zip(VALUES, values[10:]))
        else:
            row = dict(zip(FIELDS, RECORD.unpack(self.partial)))
            if len(self.rows) >= self.metadata['count']:
                raise ValueError('capture longer than frozen metadata')
            if not all(math.isfinite(row[k]) for k in FIELDS[1:7]):
                self.quality_issues.add('nonfinite_imu')
            if row['big_raw'] > 8191 or row['small_raw'] > 8191:
                self.quality_issues.add('invalid_encoder')
            if self.rows:
                delta = (row['tick_ms'] - self.rows[-1]['tick_ms']) & 0xffffffff
                if delta != 4:
                    self.quality_issues.add('irregular_sample_interval')
            if not row['flags'] & 1 or row['imu_age_ms'] > 10 or max(row['big_age_ms'], row['small_age_ms']) > 20:
                self.quality_issues.add('stale_or_invalid_feedback')
            if row['flags'] & ((31 << 5) | (1 << 10)):
                self.quality_issues.add('limit_or_saturation')
            if row['flags'] & (1 << 11):
                self.quality_issues.add('irregular_sample_interval')
            self.rows.append(row)
        self.sequence += 1
        self.part = 0
        self.partial.clear()

    @property
    def complete(self):
        return self.metadata is not None and len(self.rows) == self.metadata['count'] and self.part == 0

    def report(self):
        issues = set(self.quality_issues)
        if not self.complete:
            issues.add('incomplete_download')
        if self.metadata is None or self.metadata['phase'] != 5:
            issues.add('trial_not_completed')
        if len(self.rows) != 1001:
            issues.add('sample_count_not_1001')
        if self.crc_errors:
            issues.add('serial_crc_errors')
        return dict(download_complete=self.complete, records=len(self.rows),
                    quality_issues=sorted(issues), crc_errors=self.crc_errors,
                    model_status='not_identified',
                    output_units='software voltage command integer; not torque/current or CAN acknowledgment')
