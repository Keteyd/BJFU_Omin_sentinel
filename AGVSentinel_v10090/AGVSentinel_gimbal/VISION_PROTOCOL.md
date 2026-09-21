# Gimbal Vision UART Protocol

The gimbal PC communicates with the gimbal controller over USART1 at 115200 baud, 8 data bits, no parity, and 1 stop bit.

Both directions use a fixed 15-byte frame:

| Offset | Length | Description |
| --- | --- | --- |
| 0 | 1 | Header, `0x53` |
| 1..12 | 12 | Direction-specific payload |
| 13 | 1 | CRC8 over bytes `0..12` |
| 14 | 1 | End marker, `0x45` |

CRC parameters:

- Width: 8 bits
- Polynomial: `0x31`
- Initial value: `0x00`
- Input/output reflection: disabled
- Final XOR: `0x00`

The controller discards a received frame when its length, header, end marker, or CRC is invalid. Invalid frames do not refresh the 100 ms vision online timeout.
