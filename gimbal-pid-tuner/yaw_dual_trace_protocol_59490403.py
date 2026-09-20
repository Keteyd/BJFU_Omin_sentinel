"""Historical trace-v4 decoder for firmware 0x59490403 captures."""

from yaw_dual_trace_protocol import *
from yaw_dual_trace_protocol import Decoder as CurrentDecoder


BUILD = 0x59490403


class Decoder(CurrentDecoder):
    BUILD = BUILD
