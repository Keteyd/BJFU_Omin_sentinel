# Gimbal PC and Board Communication

## Control ownership

The gimbal PC is the only onboard PC. The gimbal controller combines remote,
decision-PC, and vision inputs, controls the gimbal and shooter, and sends the
final chassis motion command over CAN.

The chassis controller executes chassis motion, receives the referee system,
and sends referee status back to the gimbal controller.

## UART assignment

| Device | Port | Purpose | Format |
| --- | --- | --- | --- |
| Gimbal controller | USART6 | Decision and vehicle control | 115200, 8N1 |
| Gimbal controller | USART1 | Vision/auto-aim | 115200, 8N1 |
| Gimbal controller | USART3 | Remote receiver | 100000, 8E1 |
| Chassis controller | USART1 | Referee system | 115200, 8N1 |
| Chassis controller | USART6 | Unused | - |

The gimbal PC therefore needs two serial links: one to USART6 for decision
commands and one to USART1 for vision data.

## PC control frame

The USART6 frame remains 16 bytes:

    [0xFF][command][12-byte payload][CRC-8][0x0D]

CRC-8 uses polynomial 0x31 and initial value 0x00, calculated over bytes 0
through 13. Multi-byte values use little-endian byte order.

| Command | Meaning |
| --- | --- |
| 0x01 | Gimbal and shooter command |
| 0x02 | Direct chassis command |
| 0x03 | Navigation chassis command |

The existing payload definitions in periph_pc_comm.h are unchanged.

### Command 0x01 payload

| Payload offset | Type | Meaning |
| --- | --- | --- |
| 0 | int16 | Yaw rate in 0.01 deg/s |
| 2 | int16 | Pitch rate in 0.001 rad/s |
| 4 | uint8 | Fire request, 0 or 1 |
| 5 | uint8 | bit0 enables gimbal; bit1 enables auto-aim |
| 6 | 6 bytes | Reserved, set to zero |

### Command 0x02 payload

| Payload offset | Type | Meaning |
| --- | --- | --- |
| 0 | int16 | Forward velocity vx in mm/s |
| 2 | int16 | Rightward velocity vy in mm/s |
| 4 | int16 | Angular velocity wz in mrad/s |
| 6 | uint8 | 0 stop; 1 body coordinates; 2 gimbal coordinates |
| 7 | 5 bytes | Reserved, set to zero |

Command 0x03 contains three little-endian float values: linear_x, linear_y,
and angular_z. Commands 0x01, 0x02, and 0x03 have independent 100 ms
timeouts. If 0x02 and 0x03 are both current, the newest one wins.

## Inter-board CAN

| CAN ID | Direction | Payload |
| --- | --- | --- |
| 0x310 | Gimbal to chassis | vx, vy, wz, chassis mode, sequence |
| 0x311 | Gimbal to chassis | heartbeat and input-source status |
| 0x314 | Gimbal to chassis | gimbal yaw and pitch feedback |
| 0x320 | Chassis to gimbal | heartbeat, command status, referee status |
| 0x340 | Chassis to gimbal | basic referee data |
| 0x341 | Chassis to gimbal | referee limits and power buffer |

The chassis stops motion if the command frame is older than 100 ms or the
gimbal heartbeat is older than 200 ms. The gimbal reports referee data offline
when the chassis heartbeat is stale or the chassis reports the referee link
offline.
