#!/usr/bin/env python3
"""Trace-v4 dual-reference firmware tool; no automatic release."""

from yaw_slow_cli import main
from yaw_dual_trace_protocol import Decoder, TRACE_FIELDS


if __name__ == '__main__':
    raise SystemExit(main(decoder_class=Decoder, fields=TRACE_FIELDS))
