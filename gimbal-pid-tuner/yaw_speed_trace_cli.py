#!/usr/bin/env python3
"""Collect frozen S1/S2 direct speed-reference traces at 460800 baud."""

from yaw_dual_trace_protocol import TRACE_FIELDS
from yaw_slow_cli import main
from yaw_speed_trace_protocol import Decoder


if __name__ == '__main__':
    raise SystemExit(main(
        decoder_class=Decoder,
        fields=TRACE_FIELDS,
        phase_sets=('S1', 'S2'),
        dual_confirm='SPEED_REFERENCE_FIXED_CHASSIS_CLEAR_YAW',
        actions=('probe', 'bench', 'dual', 'cancel', 'release')))
