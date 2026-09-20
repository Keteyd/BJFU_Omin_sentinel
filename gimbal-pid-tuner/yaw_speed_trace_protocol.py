"""Strict trace-v5 decoder for direct dual speed-reference identification."""

import math
import struct

from yaw_capture_protocol import frame
from yaw_identification_protocol import MAGIC, META, VALUES
from yaw_slow_protocol import (BENCH, CANCEL, KEEPALIVE, PROBE, PROFILE, RECEIPT,
                               RELEASE, request as legacy_request)
from yaw_cd_trace_protocol import Decoder as CDDecoder


BUILD = 0x59490702
STREAM_VERSION = 5
SPEED_S1, SPEED_S2 = 15, 16
SPEED_KNOTS = (0, 3000, 31000, 36000, 0, 0, 0, 0)
NOMINAL_RECORDS = 9001
DURATION_MS = 36000
PLAN_SHA256 = '64E23589DE5FBE5E6673349BBABE4B709FC45B89C3016C1A64A7FAFEC1D46BAC'
HOST_DECODER_REVISION = '59490702'
PHASE_OPS = {'S1': SPEED_S1, 'S2': SPEED_S2}
PHASE_VALUES = {'S1': 0, 'S2': 1}

_HARMONICS = (
    ((5, 13, 29, 61), (8, 19, 41, 83)),
    ((7, 17, 37, 79), (11, 23, 47, 101)),
)
_AMPLITUDES = (
    ((4.4576823, 3.3432617, 1.6716309, .5572103),
     (5.6957129, 4.5565703, 2.2782852, .9113141)),
    ((3.2938282, 2.4703712, 1.2351856, .4117285),
     (4.5531931, 3.6425546, 1.8212773, .7285109)),
)
_PHASES = (
    ((.31, 1.47, 2.71, 4.19), (1.03, 2.33, 4.41, .57)),
    ((2.02, .63, 3.82, 5.21), (.49, 3.07, 1.58, 4.76)),
)


def speed_wave(drive_ms, axis, phase_set):
    """Mirror the frozen firmware reference in degrees/second."""
    if drive_ms <= 0 or drive_ms >= 28000 or axis not in (0, 1) or phase_set not in (0, 1):
        return 0.0
    duration = 28.0
    t = drive_ms * .001
    x = t / duration
    window = math.sin(math.pi*x)**2
    window_rate = math.pi/duration*math.sin(2*math.pi*x)
    position = position_rate = 0.0
    for harmonic, amplitude, phase in zip(
            _HARMONICS[phase_set][axis], _AMPLITUDES[phase_set][axis],
            _PHASES[phase_set][axis]):
        omega = 2*math.pi*harmonic/duration
        angle = omega*t+phase
        position += amplitude*math.sin(angle)
        position_rate += amplitude*omega*math.cos(angle)
    limit = (30.0, 60.0)[axis]
    return max(-limit, min(limit, window_rate*position+window*position_rate))


def speed_reference_matches(elapsed_ms, recorded, phase_set):
    """Allow the record timestamp to follow the applied control reference by 1 ms."""
    if not 3000 <= elapsed_ms < 31000:
        return all(abs(value) <= .04 for value in recorded)
    drive_ms = elapsed_ms-3000
    return any(all(abs(value-speed_wave(drive_ms-lag_ms, axis, phase_set)) <= .04
                       for axis, value in enumerate(recorded))
               for lag_ms in (0, 1))


class Decoder(CDDecoder):
    BUILD = BUILD
    STREAM_VERSION = STREAM_VERSION
    MAX_RECORDS = EXPECTED_RECORDS = NOMINAL_RECORDS
    PHASE_OPS = PHASE_OPS
    PHASE_VALUES = PHASE_VALUES
    PROFILE_ID = 5
    PROFILE_AXIS = 5
    LIVE_TIMEOUT_S = 56
    PROFILE_PHASE_VALUES = (0, 1)
    PLAN_SHA256 = PLAN_SHA256
    REFERENCE_SET_LABEL = 'speed-reference S1/S2'

    def _metadata(self, seq, raw):
        if self.profile is None:
            raise ValueError('metadata arrived before speed profile')
        v = META.unpack(raw)
        m = dict(zip(('id', 'start_ms', 'build', 'count', 'period_ms', 'phase',
                      'reason', 'axis', 'version', 'setup'), v[:10]))
        m['configuration'] = dict(zip(VALUES, v[10:]))
        c = m['configuration']
        if (m['id'] != self.trial_id or m['build'] != self.BUILD or m['period_ms'] != 4
                or m['version'] != self.STREAM_VERSION or m['setup'] != 1
                or m['count'] > self.MAX_RECORDS or m['axis'] != self.PROFILE_AXIS
                or c['amplitude_deg'] != 0 or not all(math.isfinite(x) for x in v[10:])):
            raise ValueError('invalid speed-reference metadata')
        if self.expected and (m['axis'] != self.expected[2]
                              or abs(c['amplitude_deg']-self.expected[3]) > 1e-4):
            raise ValueError('firmware request differs from expected speed target')
        if not 0 < c['big_effort_limit'] <= 30 or not 0 < c['small_effort_limit'] <= 6:
            raise ValueError('unexpected output limits')
        if seq == 0 and (m['count'] or m['phase'] != 2 or m['reason']):
            raise ValueError('invalid initial speed metadata')
        if seq == 1 and (m['phase'] not in (5, 6) or m['count'] < len(self.rows)):
            raise ValueError('invalid terminal speed metadata')
        frozen = lambda item: {k: x for k, x in item.items()
                               if k not in ('count', 'phase', 'reason')}
        if self.initial and frozen(self.initial) != frozen(m):
            raise ValueError('speed configuration or anchors changed')
        if self.initial is None:
            self.initial = m
        if seq == 1:
            if self.metadata is not None and self.metadata != m:
                raise ValueError('terminal speed metadata changed')
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
                or p['profile'] != self.PROFILE_ID
                or p['reverse'] not in self.PROFILE_PHASE_VALUES
                or v[23:] != (self.STREAM_VERSION, 0)):
            raise ValueError('unexpected speed-reference profile/limits/build')
        if self.expected and (p['profile'], p['reverse']) != self.expected[:2]:
            raise ValueError('firmware started a different speed phase set')
        if self.profile is not None and self.profile != p:
            raise ValueError('speed profile changed midstream')
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
            raise ValueError('recorded speed reference differs from frozen waveform')

    def report(self):
        report = super().report()
        report.pop('offline_reference_acceleration_review_limit_dps2', None)
        report['frozen_reference_plan_sha256'] = self.PLAN_SHA256
        report['host_decoder_revision'] = HOST_DECODER_REVISION
        report['speed_reference_units'] = 'centidegrees_per_second'
        report['controller_interface'] = 'angle PIDs bypassed; existing speed PIDs and effort limits retained'
        report['hardware_takeover_allowed'] = False
        report['trace_semantics'] = (
            'frozen direct big motor-speed and small inertial-yaw-rate references in '
            'centidegrees/second; attempted CAN command integral and raw current feedback; '
            'small-joint absolute/relative travel, measured-rate, feedback, remote-stop and '
            'heartbeat guards remain active; cable-free big-yaw and heading travel stops are disabled')
        return report


def request(op=PROBE, trial_id=0, axis=0, value=0):
    if op in (SPEED_S1, SPEED_S2):
        if (type(trial_id) is not int or not 1 <= trial_id <= 0xffffffff
                or axis != 5 or value != 0):
            raise ValueError('speed request needs nonzero ID, axis 5 and fixed amplitudes')
        return frame(0x36, struct.pack('<IHBBI', trial_id, 0, op, 5, MAGIC))
    if op == RECEIPT:
        if (type(trial_id) is not int or not 1 <= trial_id <= 0xffffffff
                or axis != 0 or type(value) is not int or not 1 <= value <= NOMINAL_RECORDS):
            raise ValueError('invalid speed cumulative record receipt')
        return frame(0x36, struct.pack('<IHBBI', trial_id, value, op, 0, MAGIC))
    if op not in (PROBE, KEEPALIVE, CANCEL, RELEASE, BENCH):
        raise ValueError('operation is not supported by the speed-reference host tool')
    return legacy_request(op, trial_id, axis, value)


Decoder.REQUEST_FN = staticmethod(request)
