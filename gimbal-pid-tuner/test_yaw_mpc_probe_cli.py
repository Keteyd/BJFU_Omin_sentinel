import struct
import unittest

from yaw_capture_protocol import frame
from yaw_mpc_probe_cli import Decoder


class MpcProbeDecoderTests(unittest.TestCase):
    def test_accepts_deployed_build_and_current_record_size(self):
        decoder = Decoder()
        decoder.feed(frame(0x3B, struct.pack("<IIHH", 0x59490903,
                                             0x31444959, 1, 152)))

        self.assertEqual(decoder.info["build"], 0x59490903)
        self.assertEqual(decoder.info["record_bytes"], 152)

    def test_rejects_legacy_record_size(self):
        decoder = Decoder()

        with self.assertRaisesRegex(ValueError, "need firmware 0x59490903"):
            decoder.feed(frame(0x3B, struct.pack("<IIHH", 0x59490903,
                                                 0x31444959, 1, 48)))


if __name__ == "__main__":
    unittest.main()
