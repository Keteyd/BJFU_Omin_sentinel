#!/usr/bin/env python3
"""Read-only build/status probe for the deployed speed-reference MPC firmware."""

from yaw_slow_cli import main
from yaw_slow_protocol import Decoder as LegacyDecoder, FIELDS


class Decoder(LegacyDecoder):
    BUILD = 0x59490903
    # The production MPC build keeps the current CAN/history trace ABI even
    # though this utility only performs read-only build/status probes.
    RECORD_BYTES = 152


if __name__ == "__main__":
    raise SystemExit(main(
        decoder_class=Decoder,
        fields=FIELDS,
        actions=("probe", "cancel", "release"),
    ))
