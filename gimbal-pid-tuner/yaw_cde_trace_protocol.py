"""Strict 0x59490503 trace-v5 decoder for the one-time phase-E validation."""

import struct

from yaw_capture_protocol import frame
from yaw_identification_protocol import MAGIC
from yaw_slow_protocol import (BENCH, CANCEL, KEEPALIVE, PROBE, RECEIPT, RELEASE)
from yaw_cd_trace_protocol import (CD_KNOTS, DURATION_MS, NOMINAL_RECORDS,
                                   Decoder as CDDecoder, request as cd_request)


BUILD = 0x59490503
STREAM_VERSION = 5
CD_E = 14
PLAN_SHA256 = 'F59EA206EBCBFE0914C10CE6619B49F5EF2E6257625EACA229A6BD9AAB0A84BE'
MAXIMUM_RECORD_SHORTFALL = 45


def request(op=PROBE, trial_id=0, axis=0, value=0):
    if op == CD_E:
        if (type(trial_id) is not int or not 1 <= trial_id <= 0xffffffff
                or axis != 4 or value != 0):
            raise ValueError('phase-E request needs nonzero ID, axis 4 and fixed amplitudes')
        return frame(0x36, struct.pack('<IHBBI', trial_id, 0, op, 4, MAGIC))
    if op not in (PROBE, KEEPALIVE, CANCEL, RELEASE, BENCH, RECEIPT):
        raise ValueError('operation is not supported by the phase-E host tool')
    return cd_request(op, trial_id, axis, value)


class Decoder(CDDecoder):
    BUILD = BUILD
    STREAM_VERSION = STREAM_VERSION
    PHASE_OPS = {'E': CD_E}
    PHASE_VALUES = {'E': 2}
    PROFILE_PHASE_VALUES = (0, 1, 2)
    PLAN_SHA256 = PLAN_SHA256
    REFERENCE_SET_LABEL = 'C/D/E'
    SAMPLE_SHORTFALL_LIMIT = MAXIMUM_RECORD_SHORTFALL
    LIVE_TIMEOUT_S = 56


Decoder.REQUEST_FN = staticmethod(request)
