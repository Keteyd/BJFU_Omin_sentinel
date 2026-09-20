"""Strict trace-v5 decoder for the cable-free R1/F4/F5 amplitude ladder."""

import math
import struct

from yaw_capture_protocol import frame
from yaw_identification_protocol import MAGIC
from yaw_slow_protocol import (BENCH, CANCEL, KEEPALIVE, PROBE, PROFILE, RECEIPT,
                               RELEASE)
from yaw_cd_trace_protocol import (CD_C, CD_D, CD_KNOTS, DURATION_MS,
                                   NOMINAL_RECORDS, Decoder as CDDecoder,
                                   request as cd_request)


BUILD = 0x59490601
STREAM_VERSION = 5
CD_E = 14
PLAN_SHA256 = '1F78E05C659742F3A9282942A235E11509EAF4CABC40883F87A726FAADDF6789'
PHASE_OPS = {'R1': CD_C, 'F4': CD_D, 'F5': CD_E}
PHASE_VALUES = {'R1': 0, 'F4': 1, 'F5': 2}
PEAKS = {0: (3., 2.), 1: (12., 8.), 2: (15., 10.)}
MAXIMUM_RECORD_SHORTFALL = 45


def request(op=PROBE, trial_id=0, axis=0, value=0):
    if op in PHASE_OPS.values():
        if (type(trial_id) is not int or not 1 <= trial_id <= 0xffffffff
                or axis != 4 or value != 0):
            raise ValueError('ladder request needs nonzero ID, axis 4 and fixed amplitudes')
        return frame(0x36, struct.pack('<IHBBI', trial_id, 0, op, 4, MAGIC))
    if op not in (PROBE, KEEPALIVE, CANCEL, RELEASE, BENCH, RECEIPT):
        raise ValueError('operation is not supported by the cable-free host tool')
    return cd_request(op, trial_id, axis, value)


class Decoder(CDDecoder):
    BUILD = BUILD
    STREAM_VERSION = STREAM_VERSION
    PHASE_OPS = PHASE_OPS
    PHASE_VALUES = PHASE_VALUES
    PROFILE_PHASE_VALUES = (0, 1, 2)
    PLAN_SHA256 = PLAN_SHA256
    REFERENCE_SET_LABEL = 'cable-free R1/F4/F5 ladder'
    SAMPLE_SHORTFALL_LIMIT = MAXIMUM_RECORD_SHORTFALL
    LIVE_TIMEOUT_S = 56

    def _profile(self, raw):
        v = PROFILE.unpack(raw)
        p = dict(zip(('id', 'build', 'baud', 'samples', 'period_ms', 'capacity',
                      'duration_ms'), v[:7]))
        p.update(knots_ms=list(v[7:15]), big_peak=v[15], small_peak=v[16],
                 big_travel=v[17], small_travel=v[18], heading_travel=v[19],
                 center_deg=v[20], profile=v[21], reverse=v[22], version=v[23],
                 reserved=v[24])
        expected_peaks = PEAKS.get(p['reverse'])
        if (tuple(v[:7]) != (self.trial_id, self.BUILD, 460800, NOMINAL_RECORDS,
                             4, self.CAPACITY, DURATION_MS)
                or tuple(v[7:15]) != CD_KNOTS
                or expected_peaks is None
                or tuple(v[15:17]) != expected_peaks
                or tuple(v[17:21]) != (25., 20., 20., 10.)
                or p['profile'] != self.PROFILE_ID
                or v[23:] != (self.STREAM_VERSION, 0)
                or not all(math.isfinite(x) for x in v[15:21])):
            raise ValueError('unexpected cable-free ladder profile/limits/build')
        if self.expected and (p['profile'], p['reverse']) != self.expected[:2]:
            raise ValueError('firmware started a different ladder phase')
        if self.profile is not None and self.profile != p:
            raise ValueError('cable-free ladder profile changed midstream')
        self.profile = p

    def report(self):
        report = super().report()
        # The inherited C/D decoder publishes the old 200 dps^2 design-review
        # threshold.  It is not a runtime guard and does not apply to this
        # deliberately larger, separately frozen ladder.
        report.pop('offline_reference_acceleration_review_limit_dps2', None)
        phase_name = next((name for name, value in self.PHASE_VALUES.items()
                           if self.profile and value == self.profile['reverse']), None)
        report['frozen_reference_plan_sha256'] = self.PLAN_SHA256
        report['ladder_phase_name'] = phase_name
        report['reference_acceleration_runtime_abort'] = False
        report['trace_semantics'] = (
            'attempted raw command integral; exact phase-E reference shape scaled by '
            'the frozen cable-free R1/F4/F5 ladder; measured travel/rate, feedback, '
            'remote-stop and heartbeat guards remain active; current is raw feedback, not torque')
        return report


Decoder.REQUEST_FN = staticmethod(request)
