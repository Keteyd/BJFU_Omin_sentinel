# Buffered yaw diagnostics, version 2

## SPI compatibility revision (current)

The initial burst build failed its first live static validation. Its IMU
sequence stayed at 90 over a 5-second recording, all 200 records were stale,
and flags were 205 (UP, but INS not ready). Both yaw commands remained zero.
The high-rate test was not started. Evidence:
`NoMachineTemp/yaw-burst-first-live-20260908-01/`.

Runtime reads now restore the legacy byte-at-a-time SPI timing and transmitted
dummy values, while retaining status checks and all-or-nothing sample update.
CS remains asserted across each register transaction. Each one-byte HAL call
has a 10 ms timeout instead of the initial burst's 1 ms whole-transaction
timeout. This is not a guaranteed loop deadline. Chip-ID validation, readiness
streak and dt rejection remain enabled. The exact cause of the first live
failure is not proven; the new diagnostic frames distinguish the failure paths.

During a capture monitor session, two CRC-protected diagnostic frames alternate
at 250 ms intervals. No extra diagnostic transmissions occur while the v2 RAM
buffer is still acquiring. `0x34` payload `<IIBBBB>`: failed reads, successful
reads, last completed attempt's failed stage, HAL status, last read accel ID,
last read gyro ID. Stages: 0 success, 1 accel ID, 2 accel data, 3 gyro ID/data,
4 temperature. A bad chip ID may have HAL status 0. IDs can remain from an
older attempt if the corresponding transaction failed. Driver diagnostics are
published under a short critical section, not while transferring SPI bytes.
`0x35` payload `<IIf>`: INS read rejection count, dt rejection count, most
recent attempted dt. Counts are cumulative uint32 values. The collector saves
these packets with host receipt times in `report.json:imu_diagnostics`; times
are not hardware sample timestamps, and separate frames are not simultaneous.

Current remote tools: `/home/nuc11--02/yaw-capture-v2-spi-GJD2AaFl`.
Use this directory in the commands below instead of the original v2 directory.
Current AXF SHA256:
`DE874990D693BD5FE6B5727A0E9F53EF303AEF44C13D747EDE18A10260611365`.
AC6: 0 errors, 69 warnings; Code 108004, RO 4124, RW 328, ZI 108260 bytes.
Log: `NoMachineTemp/yaw_burst_spi_byte_rebuild.log`.
Windows: 13 protocol tests plus actual-driver fault-injection tests pass;
NUC: 11 protocol tests pass, 2 Windows fixtures skipped. Tests now verify
byte ordering, continuous chip select, and failure at every gyro-transfer byte.
BMI088 private declarations were moved from the public header to the source,
and the register-header include case was corrected, reducing header warnings.
The user subsequently flashed this revision and its UP-only static live and
buffered validation passed: 200 live records and 512 burst records, no invalid
records or read failures. Burst intervals were all 2 ms across 1.022 seconds.
See `YAW_IDENTIFICATION_LOG.md` for evidence and timing caveats. Dynamic
stability and persistent heading drift remain unresolved; no powered test
has been performed by the agent.

## Current checkpoint

This is instrumentation, not an MPC controller or an excitation generator.
No PID gain, motor polarity, soft limit or big-yaw inhibition setting changed.
Do not enable motors for the first validation. User flashes the gimbal build,
keeps the right switch UP, performs Reset then Run, and leaves the vehicle
stationary. Confirm IMU and safety telemetry before any later powered test.

The prior UP-only receiver test recorded both stick directions returning to
exact zero. Persistent yaw drift remains unresolved. Neither the drift nor
the earlier enabled-loop runaway is claimed to be fixed by this release.

## Runtime IMU validity

The synchronous BMI088 sample now uses checked HAL SPI transfers for the
accelerometer ID/data, gyro ID/data and temperature. All transfers must return
HAL_OK and both chip IDs must match before sensor fields and successful-read
timestamp are updated. On failure the driver reports ERROR and leaves previous
sensor values and timestamp unchanged. See the current revision above for the
checked byte-wise transfer timeout and first-version hardware failure.

Accelerometer transactions retain their extra dummy byte, unlike the gyro.
This layout follows section 6.1.2 of the
[Bosch BMI088 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf).
Register-level mocks verify driver behavior; physical SPI operation must still
be confirmed after flashing. This change covers the scheduled synchronous
read path, not unused DMA callbacks or a redesign of initialization self-tests.
Successful register reads do not prove a new hardware conversion each time;
there is no sensor-DRDY synchronization added in this build.

INS seeds the DWT interval before its loop. A failed sample, nonfinite or
nonpositive dt, or dt greater than 10 ms is not integrated/published and resets
the valid streak. INS readiness requires 20 consecutive accepted updates and
the existing age/state checks. Old observation counters/timestamps remain
visible while unavailable; the INS-ready flag is clear. This rejects timing
gaps rather than reconstructing missing angular motion. After a debugger pause
or large gap, do not assume heading continuity or resume a powered test; use
UP and Reset/Run to reestablish the test baseline. RTOS recovery behavior has
not been validated on the vehicle yet.

## Acquisition and wire format

- Existing v1 live capture remains 40 Hz with heartbeat `0x24 A5 01 D1 01`.
- V2 uses monitor heartbeat `0x24 A5 01 D2 02` every 200 ms. Like v1, it neither
  enables/disables motors nor changes references or gains. Right switch UP
  remains the operator's safety prerequisite, not a effect of the heartbeat.
- One v2 session acquires 512 records into RAM, with a target interval of 2 ms
  (nominal 500 Hz). Sampling runs in the gimbal task, not an independent timer
  ISR. Actual timestamps and task dt must be inspected; 500 Hz is not guaranteed.
- Once full, the buffer is immutable and is downloaded. Acquisition does not
  continue while downloading. On an undelayed schedule its first-to-last span
  is 1.022 seconds. This is a short burst, not a 20-second high-rate recording.
- Frames still use command `0x32`, 16-byte framing and the same CRC. Payload is
  sequence uint16, part uint8, version uint8 (=2), then 8 bytes of record data.
  Twelve ordered parts form each 96-byte record. Sequence is 1..512.
- The first 88 bytes equal the v1 record; appended float32 fields are
  `control_dt_s` and `gimbal_dt_s`. Wire struct is `<IIHHHhHH19f>`.
- Task dt fields measure time between successive task-loop starts. Control dt
  is the latest published value, not a synchronized control-step sample or its
  execution duration. Control integration and existing per-call PID/filter
  calculations have not been changed to use these diagnostic dt values.
- Buffer storage is 49152 bytes plus 12 bytes bookkeeping. No heap allocation.
  Late intervals increment the skipped-slot counter; timestamps are authoritative.
- A failed UART enqueue does not advance the part cursor. Session expiry or
  switching back to v1 discards any pending burst and resets its state. A
  completed session does not rearm on identical heartbeats.

## PC collector

Independently deployed to `/home/nuc11--02/yaw-capture-v2-MoZJI3sA` on
`192.168.1.113`. Older capture and tuning directories are unchanged.

For the first static UP-only validation:

```sh
python3 /home/nuc11--02/yaw-capture-v2-MoZJI3sA/yaw_capture_cli.py \
  --buffered --duration 25 \
  --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  --output /home/nuc11--02/yaw-capture-v2-MoZJI3sA/captures/static_01
```

`--duration` is acquisition/download timeout, not the burst span. The tool
requires a free port, opens it exclusively, waits one second without a
heartbeat to expire any prior session, flushes stale incoming bytes, then
starts the burst. Download may take roughly 12-20 seconds depending on task
and UART timing. Stops early after 512 records. The output directory must not
already exist. No automatic retry or motor command is sent.

No records within 5 seconds is reported as an error. Missing records or a
partial download produce a nonzero exit status while preserving evidence.
Inspect report validity counts, flags, skipped slots and task intervals even
if the transfer succeeds. V2 validity checks require finite positive dt,
control dt <=20 ms and gimbal dt <=10 ms, in addition to the v1 checks.
These are diagnostic validity thresholds, not a stability guarantee.

For offline replay, pass `--buffered --replay raw.bin --output NEW_DIRECTORY`.
Do not use a human chat cue to synchronize a one-second burst with powered
motion. A separately designed event trigger or scheduled test procedure is
still needed for enabled stick-release/system identification experiments.

## Initial build verification (superseded)

- Windows: 12 capture/burst tests passed, including actual C record fixtures,
  fragmentation, corrupt/missing data, full replay and partial-burst rejection.
- C buffer test covers wraparound timestamps, missed slots, immutable download,
  retry without advancing, capacity, completion and reset.
- Actual BMI088 driver with mocked registers passes default/calibration tests,
  all four runtime transfers failing with ERROR/BUSY/TIMEOUT, wrong IDs and
  subsequent valid recovery. On failure timestamps and gyro/accel stay unchanged.
- NUC Python 3.10: 10 tests passed, 2 Windows C fixture tests skipped. No serial
  port was opened during deployment/testing.
- Broad Windows `test_yaw_*.py` discovery additionally hits an existing
  `test_yaw_limits_cli` import failure because `termios` is Linux-only. The
  explicit 12-test capture suite passes; the old platform issue was not edited.
- AC6 full build: 0 errors, 85 warnings. Code 107364, RO 4124, RW 332,
  ZI 108208 bytes. Map regions: RW_IRAM1 0x19400 / 0x1C000;
  RW_IRAM2 0x13FC / 0x4000. Link fit does not prove runtime task stack margin.
- Log: `NoMachineTemp/yaw_burst_rebuild.log` in the workspace root.
- Artifact: `AGVSentinel_v10090/AGVSentinel_gimbal/MDK-ARM/Objects_Gimbal/AGVSentinel_Gimbal.axf`.
- SHA256: `0732D8A1D16D441D331C43C33BF1A5F7123E35FC622B2C539ADA508EADDC57FF`.

The agent has not flashed or run this build on the MCU. First next checkpoint
is static UP-only telemetry and burst verification, not powered motion.
