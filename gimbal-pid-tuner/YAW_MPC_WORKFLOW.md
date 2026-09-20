# Dual-yaw MPC: capture and identification

## Status

2026-09-08: capture, 500 Hz buffered acquisition and a host-triggered powered
small-yaw release have been checked. The user reports manual operation fully
normal and explicitly requests returning to MPC instead of further opposite-
direction PID troubleshooting. That diagnostic branch is paused.

An offline six-state coupled MPC prototype, synthetic closed-loop scenarios,
tests and an excitation-data audit now live in `mpc/`; see `mpc/README.md`.
There is still NO identified A/B model, live optimizer, automatic excitation
or motor takeover. Synthetic matrices are not hardware parameters.

Firmware remains unchanged: big-yaw inhibition, small-yaw IMU angle/rate
loops, Pitch and chassis behavior are preserved. No new flash is needed for
the offline MPC work. The next hardware dependency is deliberate dual-input
model identification, not repeating the already accepted small-yaw check.

## Run on the vehicle PC

The current read-compatibility capture firmware has already been flashed and
validated; the following is the existing recorder usage, not a new flashing
request. Close browser serial sessions and vision UART
consumers. Place these four files in the same directory on the vehicle PC:
`yaw_capture_cli.py`, `yaw_capture_protocol.py`, `pitch_tune_cli.py`, and this guide.
Python 3.10+ on Linux is sufficient; the recorder uses standard-library serial
handling and the existing serial configuration helper.

From that directory:

```bash
python3 yaw_capture_cli.py \
  --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  --duration 20 --output captures/yaw_stationary_01
```

Use a new output directory for every recording. Existing evidence is never
overwritten. The program refuses an occupied port, holds an exclusive handle,
sets 115200/8N1 raw mode with VMIN=1, and releases the handle on exit. It sends
only the non-locking observation heartbeat, never PID/target/enable commands.
The underlying helper flushes old serial buffers on initial connection.

Outputs:

- `samples.csv`: complete records only, with MCU time, sequence, quality and units.
- `raw.bin`: received bytes for replay and parser investigations.
- `report.json`: errors, parser statistics, finite value ranges and metadata.

Ctrl+C retains collected data and produces a report. After three seconds with
no complete record, live capture fails with an explicit firmware/connection
message. Older firmware does not support this capture stream. Serial or file
failure returns nonzero; invalid individual observations remain in CSV marked
`valid=0`. A successful process exit is not proof of adequate identification data.

Offline replay also works on Windows and never opens a serial port:

```bash
python yaw_capture_cli.py --replay captures/yaw_stationary_01/raw.bin \
  --output captures/yaw_stationary_01_replay
```

Replay has no recorded host receive timestamps; `host_s=0` there. Use MCU
`tick_ms` and `delta_ms` as the time base. Do not interpolate across reset
segments or missing data without explicitly documenting it.

## Observation protocol V1

Request: existing 16-byte PC frame command `0x24`, payload `A5 01 D1 01`
followed by eight zero bytes. Refresh every 200 ms. This is an observation-only
session; it does not refresh motion-command deadlines or claim tune control.
The existing session expires 750 ms after its last session/tuning packet.
An ordinary session heartbeat clears the capture request. During capture,
legacy tuning telemetry is paused, and the existing tune-session behavior
also suppresses outgoing vision frames. Other producers may still use UART;
capture does not claim deterministic or exclusive firmware TX bandwidth.

Response: command `0x32`, payload `<uint16 sequence, uint8 part, uint8 version,
8 record bytes>`. There are 11 ordered parts, indexes 0..10, each protected
by the existing CRC8 (poly 0x31, initial zero), SOF 0xFF and EOF 0x0D.
The little-endian 88-byte record is `<IIHHHhHH17f>`:

| Offset | Fields | Units |
| --- | --- | --- |
| 0 | tick_ms, imu_sequence | uint32 ms / published IMU update counter |
| 8 | imu_age_ms, big_age_ms, small_age_ms | uint16 ms, saturated at 65535 |
| 14 | rc_yaw | signed raw receiver channel 2 |
| 16 | flags, skipped | uint16 bitfield / modulo-65536 missed sample slots |
| 20 | imu_dt_s | actual interval passed into EKF, float32 seconds |
| 24 | yaw_deg, roll_deg | continuous IMU yaw and INS.Roll, degrees |
| 32 | gyro_x_rad_s, gyro_y_rad_s, gyro_z_rad_s | corrected INS gyro, rad/s |
| 44 | big_encoder_deg, small_encoder_deg | raw joint encoder angles, 0..360 degrees |
| 52 | big_rpm, small_rpm | motor encoder velocity, rpm |
| 60 | big_command, small_command | software motor drive command; multiply by 1000 for CAN units |
| 68 | yaw_target_deg | small-yaw controller heading target, degrees |
| 72 | big_current_raw, small_current_raw | unconverted DJI current feedback register values |
| 80 | pitch_deg, small_speed_target_rpm | INS.Pitch degrees / limited small-yaw speed demand rpm |

Flags: bit 0 fresh receiver UP; 1 INS ready; 2 big online; 3 small online;
4 shared yaw output state; 5 big controller active; 6 small enable setting;
7 big output inhibited at compile time; 8 small angle-open diagnostic mode;
bits 9..13 existing small-yaw limit status. Bit 6 is a setting, not proof that
all small-yaw feedback/enable gates passed. SAFE false can also mean receiver
offline. No single flag should replace examining feedback age and output values.

`big_command` and `small_command` are software commands sampled after the yaw
output function. They are NOT measured torque, calibrated amperes, bus voltage,
or evidence that a CAN frame was accepted by the motor. Actuator identification
must account for voltage command dynamics and CAN timing. INS.Gyro[2] is a body
axis rate, not universally equal to Euler yaw rate when the platform is tilted;
all three gyro axes and attitude channels are retained for frame conversion.

## Timing and completeness

The nominal record rate is 40 Hz (one attempt per 25 ms). Eleven 16-byte frames
per record require 7040 bytes/s, about 61% of 115200/8N1 line capacity. A record
is frozen before transmission; a busy UART retries its current part. New
sample slots while a record is pending are counted as skipped, never mixed into
that record. The sample sequence advances per record captured, not per skipped
slot; host sequence gaps and MCU `skipped` measure different losses.

INS publishes a coherent completed observation; the capture copies that and
latest CAN feedback while interrupts are masked briefly. This is a common
snapshot, not synchronized physical sensor sampling. Data ages expose skew.
PC reassembles only matching, ordered parts within 250 ms of host time and
discards partial/corrupt groups. MCU tick/sequence wrap is handled, and detected
backward resets start a new segment. Finite, fresh observations with positive
EKF dt <=10 ms and valid encoder ranges receive `valid=1`. This flag alone
does not certify sensor calibration, correct orientation or a suitable model.

40 Hz is for initial sign/unit checks and low-frequency coupling observations.
It cannot resolve high-frequency oscillation or establish a high-bandwidth
motor model. Before powered model-identification experiments, implement MCU
buffered high-rate capture with sample-time timestamps and overrun reporting,
then download the buffer after the experiment. Do not raise MPC bandwidth on
the strength of a 40 Hz log.

## Experiment and implementation sequence

1. Record stationary SAFE data. Check dt, IMU counter, data ages, receiver
   center bias, voltage commands zero, valid frames and actual sample intervals.
2. In SAFE, record separate slow yaw and pitch hand movements of the IMU-bearing
   assembly. Check signs, encoder wrap, and whether the IMU sees the intended
   moving body. Convert small encoder angle with calibrated center raw 6816;
   retain asymmetric soft bounds approximately -56.678..+29.080 degrees.
3. Resolve any stale feedback, incorrect sign or unintended changing target.
   Instrument high-rate acquisition, then specify time-bounded excitation
   for each actuator with motor-output, travel and abort limits. Both motor
   responses must be recorded during each experiment. Do not infer two-actuator
   input dynamics from a zero-big-command or purely passive recording.
4. Fit coupled discrete input/state dynamics to measured data. Validate on
   separate trajectories, both movement directions and relevant pitch/joint
   configurations. A four-mechanical-state starting model may need actuator,
   friction, delay or disturbance states; do not force the order beforehand.
5. Simulate constrained linear MPC: heading tracking, big-yaw movement penalty,
   small-yaw asymmetric travel, braking feasibility, actuator bounds and input
   changes. Evaluate saturation, model errors, sensor dropout, solver timeout
   and infeasible constraints. Use validated fallback behavior, not unlimited
   reuse of the last control output.
6. Measure worst-case solve time and memory under AC6 on the MCU. Run the MPC
   in shadow mode against recorded/live observations, with no actuator control,
   before a separately scheduled real-vehicle takeover test.

Current next dependency: identify both actuator input directions and their
coupling using a separately reviewed powered procedure. The existing record
with big command zero cannot supply that input column. SSH to 192.168.1.113
has been restored; no new network setup or static baseline repetition is
required solely to run the offline MPC prototype.
