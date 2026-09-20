#!/usr/bin/env python3
"""Trace-v5 C/D reference tool; no automatic release."""

from yaw_slow_cli import main
from yaw_cd_trace_protocol import Decoder, TRACE_FIELDS


if __name__ == '__main__':
    raise SystemExit(main(decoder_class=Decoder, fields=TRACE_FIELDS,
                          phase_sets=('C', 'D'),
                          dual_confirm='CD_REFERENCE_FIXED_CHASSIS_CLEAR_YAW',
                          actions=('probe', 'dual', 'cancel', 'release')))
