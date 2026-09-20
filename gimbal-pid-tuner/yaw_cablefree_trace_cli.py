#!/usr/bin/env python3
"""Cable-free R1/F4/F5 trace-v5 collection; no automatic release."""

from yaw_slow_cli import main
from yaw_dual_trace_protocol import TRACE_FIELDS
from yaw_cablefree_trace_protocol import Decoder


if __name__ == '__main__':
    raise SystemExit(main(
        decoder_class=Decoder,
        fields=TRACE_FIELDS,
        phase_sets=('R1', 'F4', 'F5'),
        dual_confirm='CABLE_FREE_FIXED_CHASSIS_CLEAR_YAW',
        actions=('probe', 'dual', 'cancel', 'release'),
    ))
