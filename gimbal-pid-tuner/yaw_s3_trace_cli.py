#!/usr/bin/env python3
"""Collect the one-time frozen S3 direct speed-reference trace at 460800 baud."""

from yaw_dual_trace_protocol import TRACE_FIELDS
from yaw_slow_cli import main
from yaw_s3_trace_protocol import Decoder


if __name__ == '__main__':
    raise SystemExit(main(
        decoder_class=Decoder,
        fields=TRACE_FIELDS,
        phase_sets=('S3',),
        dual_confirm='S3_REFERENCE_FIXED_CHASSIS_CLEAR_YAW',
        actions=('probe', 'dual', 'cancel', 'release')))
