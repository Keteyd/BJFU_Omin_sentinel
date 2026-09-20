#!/usr/bin/env python3
"""CAN trace firmware tool. Same explicit bench/trial confirmations as slow CLI."""
from yaw_slow_cli import main
from yaw_can_trace_protocol import Decoder, TRACE_FIELDS

if __name__ == '__main__':
    raise SystemExit(main(decoder_class=Decoder, fields=TRACE_FIELDS))
