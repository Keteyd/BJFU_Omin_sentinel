# Gimbal PC and Board Communication

Current identification build `0x59490201` uses **USART1 460800 8N1**.
Old 115200 tools must not be used unchanged. Slow streaming host support and
physical link validation are pending; see [slow-stream firmware notes](YAW_SLOW_STREAM.md).
The sections below retain the existing application protocols.

## Control ownership

The gimbal PC is the only onboard PC. The gimbal controller combines remote,
decision-PC, and vision inputs, controls the gimbal and shooter, and sends the
final chassis motion command over CAN.

The chassis controller executes chassis motion, receives the referee system,
and sends referee status back to the gimbal controller.

## UART assignment

| Device | Port | Purpose | Format |
| --- | --- | --- | --- |
| Gimbal controller | USART1 | Vision, decision control, and PID tuning | 115200, 8N1 |
| Gimbal controller | USART6 | No physical cable; legacy reservation only | - |
| Gimbal controller | USART3 | Remote receiver | 100000, 8E1 |
| Chassis controller | USART1 | Referee system | 115200, 8N1 |
| Chassis controller | USART6 | Unused | - |

The gimbal PC uses the single physically connected USART1 link. The firmware
multiplexes the existing 15-byte vision frames and 16-byte PC control frames by
their frame headers and lengths.

## PC control frame

The PC control frame on the shared USART1 link remains 16 bytes:

    [0xFF][command][12-byte payload][CRC-8][0x0D]

CRC-8 uses polynomial 0x31 and initial value 0x00, calculated over bytes 0
through 13. Multi-byte values use little-endian byte order.

| Command | Meaning |
| --- | --- |
| 0x01 | Gimbal and shooter command |
| 0x02 | Direct chassis command |
| 0x03 | Navigation chassis command |
| 0x20 | Small-yaw PID tuning parameters |
| 0x21 | Small-yaw primary telemetry, controller to PC |
| 0x22 | Small-yaw auxiliary telemetry, controller to PC |
| 0x23 | Small-yaw PID reset or step-test action |
| 0x24 | PID tuner session heartbeat |
| 0x25 | Pitch IMU loop and gravity-feedforward tuning |
| 0x26 | Pitch primary telemetry, controller to PC |
| 0x27 | Pitch auxiliary telemetry, controller to PC |
| 0x28 | Pitch reset, step-test, and DM4310 MIT-gain action |

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

PID tuning frames use integer-scaled fields so their payload remains portable.
They do not refresh any vehicle motion-command timeout. The browser-based
tuning client is in the workspace `gimbal-pid-tuner` directory. While its
heartbeat is current, normal vision telemetry is paused; it resumes within
750 ms after the tuner disconnects.

After IMU stabilization is enabled, small-yaw telemetry commands `0x21` and
`0x22` report the world-frame IMU yaw and IMU Z-axis yaw rate. Motor current
and motor-online status still come from the small-yaw GM6020 feedback.

Pitch tuning uses signed `gravity_effort`: the controller sends
`gravity_effort * cos(IMU Roll)` as the DM4310 MIT torque feedforward. A
positive or negative value selects the compensation direction. Disabling the
Pitch IMU loop sends zero feedforward and holds the current DM4310 position.
During an active Pitch tuning session, normal remote, vision, and PC Pitch
target increments are ignored. Auxiliary telemetry reports the step magnitude
actually applied by the MCU after decoding and safety limiting.
Pitch step actions are applied relative to the current measured IMU attitude, so a
raw target accumulated while the outer loop was disabled cannot be released
as an unintended jump.

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
