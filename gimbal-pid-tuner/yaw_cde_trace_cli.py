#!/usr/bin/env python3
"""Trace-v5 phase-E validation tool; no automatic release."""

from yaw_slow_cli import main
from yaw_cde_trace_protocol import Decoder
from yaw_dual_trace_protocol import TRACE_FIELDS


if __name__ == '__main__':
    raise SystemExit(main(decoder_class=Decoder, fields=TRACE_FIELDS,
                          phase_sets=('E',),
                          dual_confirm='E_REFERENCE_FIXED_CHASSIS_CLEAR_YAW',
                          actions=('probe', 'dual', 'cancel', 'release')))
