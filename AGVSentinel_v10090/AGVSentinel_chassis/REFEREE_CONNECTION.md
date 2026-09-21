# Referee System Connection

The chassis controller owns the referee-system UART and forwards the required summary to the gimbal controller over the board CAN link.

UART assignment:

| Device | Chassis UART | Pins | Configuration |
| --- | --- | --- | --- |
| Referee system | USART1 | PB7 RX, PA9 TX | 115200, 8N1 |

USART6 is no longer used by the chassis firmware. PC control is connected to USART6 on the gimbal controller.

The referee receiver validates the standard 5-byte header CRC8 and the complete-frame CRC16 before updating referee data. Valid referee summaries are transmitted from chassis to gimbal with CAN IDs `0x340` and `0x341`.
