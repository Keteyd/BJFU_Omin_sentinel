"""Historical trace-v4 decoder for firmware 0x59490401 captures."""

from yaw_dual_trace_protocol import *  # Reuse the unchanged trace-v4 layout.
from yaw_dual_trace_protocol import Decoder as CurrentDecoder


BUILD = 0x59490401


class Decoder(CurrentDecoder):
    BUILD = BUILD

