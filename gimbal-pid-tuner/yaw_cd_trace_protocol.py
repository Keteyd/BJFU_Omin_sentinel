"""Strict 0x59490502 trace-v5 decoder for the frozen C/D profile."""

import math
import struct

from yaw_capture_protocol import frame
from yaw_identification_protocol import MAGIC, META, VALUES
from yaw_slow_protocol import (BENCH, CANCEL, KEEPALIVE, PROBE, RECEIPT, RELEASE,
                               PROFILE, request as legacy_request)
from yaw_dual_trace_protocol import (Decoder as TraceV4Decoder, TRACE_FIELDS,
                                     capture_timing_summary)


BUILD = 0x59490502
STREAM_VERSION = 5
CD_C, CD_D = 12, 13
CD_KNOTS = (0, 3000, 9000, 31000, 36000, 0, 0, 0)
NOMINAL_RECORDS = 9001
DURATION_MS = 36000
PLAN_SHA256 = '05DA0E5EAD289CBB3A16B4D243F13F449B523CE68B731799A768253A34DE09EF'


class Decoder(TraceV4Decoder):
    BUILD = BUILD
    STREAM_VERSION = STREAM_VERSION
    MAX_RECORDS = EXPECTED_RECORDS = NOMINAL_RECORDS
    PHASE_OPS = {'C': CD_C, 'D': CD_D}
    PROFILE_ID = 4
    PROFILE_AXIS = 4
    LIVE_TIMEOUT_S = 56
    PROFILE_PHASE_VALUES = (0, 1)
    PLAN_SHA256 = PLAN_SHA256
    REFERENCE_SET_LABEL = 'C/D'

    def _expected_phase(self, profile_elapsed_ms):
        return (2 if profile_elapsed_ms < 3000 else
                3 if profile_elapsed_ms < 31000 else
                4 if profile_elapsed_ms < 36000 else -1)

    def _metadata(self, seq, raw):
        if self.profile is None:
            raise ValueError('metadata arrived before profile')
        v = META.unpack(raw)
        m = dict(zip(('id', 'start_ms', 'build', 'count', 'period_ms', 'phase',
                      'reason', 'axis', 'version', 'setup'), v[:10]))
        m['configuration'] = dict(zip(VALUES, v[10:]))
        c = m['configuration']
        if (m['id'] != self.trial_id or m['build'] != self.BUILD or m['period_ms'] != 4
                or m['version'] != self.STREAM_VERSION or m['setup'] != 1
                or m['count'] > self.MAX_RECORDS
                or not all(math.isfinite(x) for x in v[10:])):
            raise ValueError('invalid C/D metadata')
        if m['axis'] != self.PROFILE_AXIS or c['amplitude_deg'] != 0:
            raise ValueError('C/D metadata axis/amplitude mismatch')
        if self.expected and (m['axis'] != self.expected[2]
                              or abs(c['amplitude_deg']-self.expected[3]) > 1e-4):
            raise ValueError('firmware request differs from expected C/D target')
        if not 0 < c['big_effort_limit'] <= 30 or not 0 < c['small_effort_limit'] <= 6:
            raise ValueError('unexpected output limits')
        if seq == 0 and (m['count'] or m['phase'] != 2 or m['reason']):
            raise ValueError('invalid initial C/D metadata')
        if seq == 1 and (m['phase'] not in (5, 6) or m['count'] < len(self.rows)):
            raise ValueError('invalid terminal C/D metadata')
        frozen = lambda item: {k: x for k, x in item.items()
                               if k not in ('count', 'phase', 'reason')}
        if self.initial and frozen(self.initial) != frozen(m):
            raise ValueError('C/D configuration or anchors changed')
        if self.initial is None:
            self.initial = m
        if seq == 1:
            if self.metadata is not None and self.metadata != m:
                raise ValueError('terminal C/D metadata changed')
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
                or tuple(v[7:15]) != CD_KNOTS
                or tuple(v[15:21]) != (3., 2., 25., 20., 20., 10.)
                or p['profile'] != self.PROFILE_ID or p['reverse'] not in self.PROFILE_PHASE_VALUES
                or v[23:] != (self.STREAM_VERSION, 0)):
            raise ValueError('unexpected C/D profile/limits/build')
        if self.expected and (p['profile'], p['reverse']) != self.expected[:2]:
            raise ValueError('firmware started a different C/D phase set')
        if self.profile is not None and self.profile != p:
            raise ValueError('C/D profile changed midstream')
        self.profile = p

    def report(self):
        report = super().report()
        report['trace_version'] = self.STREAM_VERSION
        report['reference_acceleration_runtime_abort'] = False
        report['offline_reference_acceleration_review_limit_dps2'] = 200.0
        report['frozen_reference_plan_sha256'] = self.PLAN_SHA256
        report['trace_semantics'] = (
            'attempted raw command integral; frozen %s references in centidegrees; ' %
            self.REFERENCE_SET_LABEL +
            'no runtime reference-acceleration abort; completion timestamps observe TSR flags; '
            'current is raw feedback, not torque'
        )
        return report


def request(op=PROBE, trial_id=0, axis=0, value=0):
    if op in (CD_C, CD_D):
        if (type(trial_id) is not int or not 1 <= trial_id <= 0xffffffff
                or axis != 4 or value != 0):
            raise ValueError('C/D request needs nonzero ID, axis 4 and fixed amplitudes')
        return frame(0x36, struct.pack('<IHBBI', trial_id, 0, op, 4, MAGIC))
    if op == RECEIPT:
        if (type(trial_id) is not int or not 1 <= trial_id <= 0xffffffff
                or axis != 0 or type(value) is not int or not 1 <= value <= NOMINAL_RECORDS):
            raise ValueError('invalid C/D cumulative record receipt')
        return frame(0x36, struct.pack('<IHBBI', trial_id, value, op, 0, MAGIC))
    if op not in (PROBE, KEEPALIVE, CANCEL, RELEASE, BENCH):
        raise ValueError('operation is not supported by the C/D host tool')
    return legacy_request(op, trial_id, axis, value)
