"""Strict trace-v5 decoder for the prospectively frozen S3 speed reference."""

import bisect
import math
import struct

from yaw_capture_protocol import frame
from yaw_identification_protocol import MAGIC, META, VALUES
from yaw_slow_protocol import (BENCH, CANCEL, KEEPALIVE, PROBE, PROFILE, RECEIPT,
                               RELEASE, request as legacy_request)
from yaw_cd_trace_protocol import Decoder as CDDecoder


BUILD = 0x59490801
STREAM_VERSION = 5
SPEED_S3 = 17
SPEED_KNOTS = (0, 3000, 31000, 36000, 0, 0, 0, 0)
NOMINAL_RECORDS = 9001
DURATION_MS = 36000
PLAN_SHA256 = 'BF41B63E5FA37AE20F065BB69603BF496E3E75CC5C4828D5A616F90267828B19'
HOST_DECODER_REVISION = '59490801'
PHASE_OPS = {'S3': SPEED_S3}
PHASE_VALUES = {'S3': 2}

_HARMONICS = ((9, 21, 43, 73), (10, 27, 53, 91))
_AMPLITUDES = (
    (2.9025746, 2.2617464, 1.2150312, .3903836),
    (5.1704974, 3.4818164, 1.9511311, .6714932),
)
_PHASES = (
    (.9650392, 4.5401046, 2.7112651, 1.3533682),
    (.4001477, 4.6934328, 6.2240131, .1600084),
)


def speed_wave(drive_ms, axis, phase_set=2):
    """Mirror the frozen S3 firmware reference in degrees/second."""
    if (drive_ms <= 0 or drive_ms >= 28000 or axis not in (0, 1)
            or phase_set != 2):
        return 0.0
    duration = 28.0
    t = drive_ms*.001
    x = t/duration
    window = math.sin(math.pi*x)**2
    window_rate = math.pi/duration*math.sin(2*math.pi*x)
    position = position_rate = 0.0
    for harmonic, amplitude, phase in zip(
            _HARMONICS[axis], _AMPLITUDES[axis], _PHASES[axis]):
        omega = 2*math.pi*harmonic/duration
        angle = omega*t+phase
        position += amplitude*math.sin(angle)
        position_rate += amplitude*omega*math.cos(angle)
    limit = (30.0, 60.0)[axis]
    return max(-limit, min(limit, window_rate*position+window*position_rate))


def speed_reference_matches(elapsed_ms, recorded, phase_set=2):
    """Allow the record timestamp to follow the applied control reference by 1 ms."""
    if phase_set != 2:
        return False
    if not 3000 <= elapsed_ms < 31000:
        return all(abs(value) <= .04 for value in recorded)
    drive_ms = elapsed_ms-3000
    return any(all(abs(value-speed_wave(drive_ms-lag_ms, axis, phase_set)) <= .04
                       for axis, value in enumerate(recorded))
               for lag_ms in (0, 1))


def s3_capture_timing_summary(rows, initial, metadata, profile):
    """Grade actual time coverage for the frozen 20 ms validation grid."""
    count = len(rows)
    result = {
        'nominal_records': NOMINAL_RECORDS,
        'actual_records': count,
        'record_shortfall': NOMINAL_RECORDS-count,
        'raw_record_count_is_diagnostic_only': True,
        'first_trace_us': None,
        'last_trace_us': None,
        'trace_span_us': None,
        'terminal_elapsed_ms': None,
        'minimum_interval_us': None,
        'maximum_interval_us': None,
        'active_grid_points': 1400,
        'maximum_left_sample_distance_us': None,
        'maximum_right_sample_distance_us': None,
        'active_20ms_grid_bracketed': False,
        'full_duration_timing_accepted': False,
        'resampling_required': count != NOMINAL_RECORDS,
    }
    if not rows or not initial or not metadata or not profile:
        return result
    traces = [row['trace_us'] for row in rows]
    intervals = [row['interval_us'] for row in rows[1:]]
    if any(right <= left for left, right in zip(traces, traces[1:])):
        return result
    worst_left = worst_right = 0
    bracketed = True
    for target in range(3000000, 31000000, 20000):
        index = bisect.bisect_left(traces, target)
        if index == 0 or index == len(traces):
            bracketed = False
            break
        worst_left = max(worst_left, target-traces[index-1])
        worst_right = max(worst_right, traces[index]-target)
    first, last = rows[0], rows[-1]
    elapsed_ms = (last['tick_ms']-initial['start_ms']) & 0xffffffff
    span = (traces[-1]-traces[0]) & 0xffffffff
    result.update(
        first_trace_us=traces[0], last_trace_us=traces[-1], trace_span_us=span,
        terminal_elapsed_ms=elapsed_ms,
        minimum_interval_us=min(intervals, default=None),
        maximum_interval_us=max(intervals, default=None),
        maximum_left_sample_distance_us=worst_left if bracketed else None,
        maximum_right_sample_distance_us=worst_right if bracketed else None,
        active_20ms_grid_bracketed=bool(
            bracketed and worst_left <= 10000 and worst_right <= 10000),
    )
    result['resampling_required'] = bool(
        count != NOMINAL_RECORDS or any(interval != 4000 for interval in intervals))
    result['full_duration_timing_accepted'] = bool(
        metadata['count'] == count and metadata['phase'] == 5 and metadata['reason'] == 0
        and last['phase'] == 5 and 0 <= traces[0] <= 10000
        and 36000000 <= traces[-1] <= 36010000 and span >= 35990000
        and 36000 <= elapsed_ms <= 36001 and intervals
        and all(0 < interval <= 10000 for interval in intervals)
        and result['active_20ms_grid_bracketed'])
    return result


class Decoder(CDDecoder):
    BUILD = BUILD
    STREAM_VERSION = STREAM_VERSION
    MAX_RECORDS = EXPECTED_RECORDS = NOMINAL_RECORDS
    PHASE_OPS = PHASE_OPS
    PHASE_VALUES = PHASE_VALUES
    PROFILE_ID = 5
    PROFILE_AXIS = 5
    LIVE_TIMEOUT_S = 56
    PROFILE_PHASE_VALUES = (2,)
    PLAN_SHA256 = PLAN_SHA256
    REFERENCE_SET_LABEL = 'speed-reference S3'

    def _metadata(self, seq, raw):
        if self.profile is None:
            raise ValueError('metadata arrived before S3 speed profile')
        v = META.unpack(raw)
        m = dict(zip(('id', 'start_ms', 'build', 'count', 'period_ms', 'phase',
                      'reason', 'axis', 'version', 'setup'), v[:10]))
        m['configuration'] = dict(zip(VALUES, v[10:]))
        c = m['configuration']
        if (m['id'] != self.trial_id or m['build'] != self.BUILD or m['period_ms'] != 4
                or m['version'] != self.STREAM_VERSION or m['setup'] != 1
                or m['count'] > self.MAX_RECORDS or m['axis'] != self.PROFILE_AXIS
                or c['amplitude_deg'] != 0 or not all(math.isfinite(x) for x in v[10:])):
            raise ValueError('invalid S3 speed-reference metadata')
        if self.expected and (m['axis'] != self.expected[2]
                              or abs(c['amplitude_deg']-self.expected[3]) > 1e-4):
            raise ValueError('firmware request differs from expected S3 target')
        if not 0 < c['big_effort_limit'] <= 30 or not 0 < c['small_effort_limit'] <= 6:
            raise ValueError('unexpected output limits')
        if seq == 0 and (m['count'] or m['phase'] != 2 or m['reason']):
            raise ValueError('invalid initial S3 metadata')
        if seq == 1 and (m['phase'] not in (5, 6) or m['count'] < len(self.rows)):
            raise ValueError('invalid terminal S3 metadata')
        frozen = lambda item: {k: x for k, x in item.items()
                               if k not in ('count', 'phase', 'reason')}
        if self.initial and frozen(self.initial) != frozen(m):
            raise ValueError('S3 speed configuration or anchors changed')
        if self.initial is None:
            self.initial = m
        if seq == 1:
            if self.metadata is not None and self.metadata != m:
                raise ValueError('terminal S3 metadata changed')
            self.metadata = m
            self.groups.pop((0x39, 0), None)

    def _profile(self, raw):
        v = PROFILE.unpack(raw)
        p = dict(zip(('id', 'build', 'baud', 'samples', 'period_ms', 'capacity',
                      'duration_ms'), v[:7]))
        p.update(knots_ms=list(v[7:15]), big_peak=v[15], small_peak=v[16],
                 big_travel=v[17], small_travel=v[18], heading_travel=v[19],
                 center_deg=v[20], profile=v[21], reverse=v[22], version=v[23],
                 reserved=v[24])
        if (tuple(v[:7]) != (self.trial_id, self.BUILD, 460800, NOMINAL_RECORDS,
                             4, self.CAPACITY, DURATION_MS)
                or tuple(v[7:15]) != SPEED_KNOTS
                or tuple(v[15:21]) != (30., 60., 0., 20., 0., 10.)
                or p['profile'] != self.PROFILE_ID or p['reverse'] != 2
                or v[23:] != (self.STREAM_VERSION, 0)):
            raise ValueError('unexpected S3 speed-reference profile/limits/build')
        if self.expected and (p['profile'], p['reverse']) != self.expected[:2]:
            raise ValueError('firmware started a different speed phase set')
        if self.profile is not None and self.profile != p:
            raise ValueError('S3 speed profile changed midstream')
        self.profile = p

    def _trace_row(self, raw):
        super()._trace_row(raw)
        row = self.rows[-1]
        if not self.initial or not self.profile or row['phase'] in (5, 6):
            return
        elapsed = (row['tick_ms']-self.initial['start_ms']) & 0xffffffff
        recorded = (row['big_reference_offset_cdeg']/100,
                    row['small_heading_reference_offset_cdeg']/100)
        if not speed_reference_matches(elapsed, recorded, self.profile['reverse']):
            raise ValueError('recorded S3 speed reference differs from frozen waveform')

    def report(self):
        report = super().report()
        diagnostic_issues = list(report['quality_issues'])
        timing = s3_capture_timing_summary(
            self.rows, self.initial, self.metadata, self.profile)
        if timing['full_duration_timing_accepted']:
            allowed = {'sample_count_not_%d' % NOMINAL_RECORDS,
                       'irregular_sample_interval'}
            report['quality_issues'] = [issue for issue in report['quality_issues']
                                        if issue not in allowed]
        report['diagnostic_quality_issues'] = diagnostic_issues
        report['capture_timing'] = timing
        report.pop('offline_reference_acceleration_review_limit_dps2', None)
        report['frozen_reference_plan_sha256'] = self.PLAN_SHA256
        report['host_decoder_revision'] = HOST_DECODER_REVISION
        report['speed_reference_units'] = 'centidegrees_per_second'
        report['controller_interface'] = (
            'angle PIDs bypassed; existing speed PIDs and effort limits retained')
        report['hardware_takeover_allowed'] = False
        report['trace_semantics'] = (
            'prospectively frozen direct S3 big motor-speed and small inertial-yaw-rate '
            'references in centidegrees/second; time-coverage data gate; raw record count '
            'is diagnostic only; small-joint travel, measured-rate, feedback, remote-stop '
            'and heartbeat guards remain active; cable-free big-yaw and heading travel '
            'stops are disabled')
        return report


def request(op=PROBE, trial_id=0, axis=0, value=0):
    if op == SPEED_S3:
        if (type(trial_id) is not int or not 1 <= trial_id <= 0xffffffff
                or axis != 5 or value != 0):
            raise ValueError('S3 speed request needs nonzero ID, axis 5 and fixed amplitudes')
        return frame(0x36, struct.pack('<IHBBI', trial_id, 0, op, 5, MAGIC))
    if op == RECEIPT:
        if (type(trial_id) is not int or not 1 <= trial_id <= 0xffffffff
                or axis != 0 or type(value) is not int or not 1 <= value <= NOMINAL_RECORDS):
            raise ValueError('invalid S3 cumulative record receipt')
        return frame(0x36, struct.pack('<IHBBI', trial_id, value, op, 0, MAGIC))
    if op not in (PROBE, KEEPALIVE, CANCEL, RELEASE, BENCH):
        raise ValueError('operation is not supported by the S3 host tool')
    return legacy_request(op, trial_id, axis, value)


Decoder.REQUEST_FN = staticmethod(request)
