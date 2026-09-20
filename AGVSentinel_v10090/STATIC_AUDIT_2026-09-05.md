# Static audit and Pitch commissioning

Scope: active gimbal startup/control/DM adapter, PC tuner/protocol, chassis
command execution/board link, and referee RX/CRC path. `trash`, bundled
vendor drivers, and the incense simulator are excluded. This is a focused
source review, not a claim that the entire robot is free of bugs.

## Fixed findings

- P1: DM first-feedback synchronization ran in RX interrupt context, while a
  precomputed task target could overwrite the synchronized position before TX.
  The first MIT transmission now re-captures actual position and clears
  velocity/feedforward; RX feedback reads and MIT transmission are protected
  from concurrent CAN RX updates. Wrong embedded motor IDs are rejected.
- P1: Pitch limits remained on the MCU's initial motor revolution after a DM
  restart. Feedback loss or a revolution discontinuity now resets alignment
  and target capture. Alignment accepts only the measured physical interval
  (with 0.02 rad measurement tolerance), rejects nonfinite values, invalid
  intervals and branches beyond MIT PMAX. Invalid alignment inhibits the
  IMU outer loop; it does not cut all motor power.
- P1: Active CLI tests could silently retain enable after loss of telemetry
  or an I/O exception with `--leave-enabled`. A 0.5-second data/aux watchdog,
  preflight validation, and exception cleanup now attempt outer-loop disable.
- P2: Monitor-only selected a tuning axis and refreshed the control lock,
  suppressing the remote commands being measured. A capability-acknowledged
  monitor session now separates observation from tuning ownership.
- P2: The host had no actual runtime limits and interpreted deadbanded zero
  error as exact tracking. Packet 0x29 exposes limits/validity/DM state;
  the CLI reports raw attitude error and attitude span as well.
- P2: Referee DMA idle handling discarded trailing partial frames and skipped
  complete frame lengths on bad CRC. A bounded stream parser retains partial
  frames and resynchronizes byte by byte after CRC failures. Both project
  copies are updated; the physical referee UART is on the chassis board.
- P2: CRC-valid navigation packets containing NaN/Inf could reach float-to-int
  conversion. The gimbal PC decoder now rejects them before refreshing the
  motion-command timestamp.

## Accepted tuning retained

Pitch IMU Kp=14, Kd=0.30, integrated motor-target rate limit=2 rad/s;
MIT Kp=50, Kd=3.2; gravity feedforward=0.30; IMU enabled by default.
These are control parameters, not a guaranteed physical speed/torque limit.
Physical stop samples: lower -0.455 rad, upper 0.259 rad. Target margin
0.005 rad gives -0.450..0.254 rad plus the selected 2*pi revolution offset.
The algorithm assumes reboot offsets are whole revolutions in the reported
motor coordinate. This assumption still requires a real power-cycle check;
arbitrary encoder-zero changes require recalibration.

## Verification

- GCC C11 -Wall -Wextra -Werror: actual limit helper and actual referee stream
  parser/CRC implementation; lower/middle/upper at k=-1,0,1; invalid phase,
  NaN/Inf, reversed/overwide limits, out-of-PMAX branch; every split point,
  corrupt CRC, noise, concatenated packets, maximum 300-byte frame. Passed.
- Five Python unittest cases with mocked serial I/O: monitor packet semantics,
  missing capability, telemetry loss, serial exception cleanup, preflight
  rejection, and raw-vs-deadbanded error. Passed on the NUC, no hardware I/O.
- AC6 full builds recorded in `../NoMachineTemp/static_audit_gimbal.log`
  and `../NoMachineTemp/static_audit_chassis.log`.

## Commissioning continuation

Builds do not flash hardware. Flash the audited gimbal target, reset and run
using the user's working Keil Debug/F5 procedure. First verify actual limits
and DM state using the updated monitor protocol, then capture manual motion
and chassis-tilt rejection without retuning. Recheck after a power cycle at
a different physical Pitch angle. Flash the chassis build to use the referee
RX fix. Hardware startup timing, physical overshoot at limits, gravity load
over the full travel, DMA saturation and referee protocol-version compatibility
are not established by these static checks.

Old README statements about USART6, startup-relative Pitch limits or Pitch
defaulting off predate this firmware. Actual PC/vision/debug transport is
USART1. Tuner and vision processes must still coordinate ownership of the
single serial device.
