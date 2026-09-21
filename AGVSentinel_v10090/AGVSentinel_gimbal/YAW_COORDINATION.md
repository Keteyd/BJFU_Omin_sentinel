# Dual-yaw control revisions

## Current: operator-selected MPC/PID backends (2026-09-13)

Build `0x59490910` retains the accepted MPC in right-switch MIDDLE and the
small-yaw-frame chassis translation added in `0x59490909`. Left-switch DOWN
selects the original cascaded PID yaw backend without changing the MIDDLE
chassis frame. Left UP/MIDDLE selects MPC. Right DOWN uses the same PID backend
with the original chassis path; right UP disables yaw output.

The PID backend restores `YAW_COORD_IMMEDIATE_FOLLOW=1`: every enabled tick sets
the big-yaw reference to `wrap(big_encoder_deg + relief_direction *
small_joint_deg)`. Big-yaw angle PID is 50/.001/4, speed PID .6/0/0, effort 30,
and speed-filter tau .030 s. Backend transitions clear PID/filter and MPC
history before re-entry. The previous left-DOWN direct-big-rate request is no
longer published by the control application.

Native controller/protocol regression tests pass. ARM Compiler 6 links with
0 errors and 0 warnings; Code 151584, RO 4772, RW 352, ZI 109360. No automatic
flash or powered test was performed.

## Historical: MPC preparation, coherent observation capture (2026-09-08)

Stage 1 adds a 40 Hz requested observation stream to the existing USART1 PC
protocol. IMU publishes coherent completed observations; each record contains
that observation and latest CAN state with data ages, sequence, receiver input,
targets and software output commands. Eleven CRC-protected parts reconstruct
one immutable 88-byte record. Busy TX does not mix samples: missed acquisition
slots are counted. The recorder sends only a non-locking monitor heartbeat.
Existing big-yaw inhibition, small-yaw IMU controller and limits are retained.

See `gimbal-pid-tuner/YAW_MPC_WORKFLOW.md` for protocol, units, capture commands,
limits of 40 Hz observation, and the progression to high-rate identification,
simulation and controller takeover. No model has yet been fitted and no MPC
output is connected. This does not resolve the previously reported runaway.

AC6: 0 errors, 85 warnings; Code 106336, RO 4092, RW 332, ZI 59028.
Log: `NoMachineTemp/yaw_capture_rebuild.log`.
AXF SHA256: `855D5C03F672513AE32DD2EB3920CD2DEF43DBF7C3FB21A4B5BD687D8A5110E2`.
Eight host tests pass, including C/Python wire compatibility, fragmentation,
corruption, missing/reordered packets, wrap/reset handling and CLI replay.
Existing actuator-isolation and soft-limit tests pass. Serial integration,
actual acquisition rate, interrupt latency and vehicle dynamics remain to be
measured. No firmware flash, remote deployment or powered test was performed.

## Historical test: big-yaw output inhibited, small-yaw IMU loop active (2026-09-07)

The user reports big yaw keeps rotating after stick release; receiver UP stops
it. Root cause is not yet established. `BIG_YAW_OUTPUT_INHIBIT_TEST=1` in
`module_big_yaw_tune.h` isolates the big axis without retuning either controller.
Big-yaw boot enable is zero; each control tick forces its enable flag to zero,
and the enable predicate independently rejects this build. The existing stop
path clears PID/coordinator state and diagnostics. The actuator is disabled
at initialization and again before group flush, with an explicit zero-effort
write. This preserves feedback while preventing active big-yaw drive through
the application path, even if its debugger enable variable is changed.

Small yaw retains the saved IMU angle/rate loops, gains, direction, encoder
soft limits and SAFE (`SMALL_YAW_ANGLE_OPEN_TEST=0`,
`BIG_YAW_PASSIVE_SMALL_TEST=0`). Pitch and chassis remain unchanged. The saved
big-yaw immediate-follow policy and gains 50/.001/4, .6/0/0, effort 30, tau .030
remain in source but cannot actuate big yaw in this build. Restore the inhibit
macro to 0 only after validation, then rebuild/reflash; do not use the browser
to bypass this isolation.

Small-yaw measured attitude and raw gyro speed diagnostics now update before
the motor-enable gate, including SAFE, when INS is ready and the values finite.
Invalid/not-ready observations report zero. SAFE still clears angle error,
speed reference, filtered controller speed and effort; those zero values are
not raw sensor measurements. Watch `GimbalYaw_DiagSmallFdbDeg`,
`GimbalYaw_DiagSmallRawSpeedRpm`, `INS.Gyro[2]` and `QEKF_INS.dt` for the passive
check. Angle/reference diagnostics may differ while the controller is disabled;
the reference is recaptured on enable. Existing auxiliary telemetry carries
raw gyro speed without adding a protocol or automatically opening the UART.

Keep UP during flashing and Reset/Run. First verify that gently moving the
IMU-bearing assembly changes angle and raw rate continuously while big effort
and active state remain zero. Only then test small-yaw stick response/holding
in MANUAL with adequate clearance and return UP on abnormal motion. Big yaw
is passive, not mechanically locked: external forces or small-yaw reaction
torque can still move it. Small-yaw stability is not yet physically verified.

AC6: 0 errors, 85 warnings; Code 105120, RO 4092, RW 328, ZI 58888.
Log: `NoMachineTemp/yaw_big_inhibit_small_imu_rebuild.log`.
AXF SHA256: `CBFE8CB03B05F154E3391F3F4962F47CA286BFC9F8F8C57721F42186959C2E6C`.
New tests using actual axis/actuator code verify zero big-axis group output
under repeated nonzero requests while the small axis remains independent.
Recompiled PID tests verify the inhibit and saved gains; existing rate-helper
and soft-limit tests pass. No automatic flash or live motor test performed.

## Historical: small-yaw IMU angle loop restored with immediate follow (2026-09-07)

At the user's request, `SMALL_YAW_ANGLE_OPEN_TEST=0` restores the original
small-yaw IMU angle outer loop and retains the gyro speed inner loop. Manual
and PC rate inputs once again integrate the desired angle; releasing the input
retains that target rather than using the rate-only diagnostic path. Initial
enable captures measured IMU attitude. Saved small-yaw gains, encoder soft
limits, feedback/IMU validity checks and SAFE are unchanged. Yaw vision angle
corrections and tuning steps are restored under their original command gates.

Big yaw remains immediate-follow (`YAW_COORD_IMMEDIATE_FOLLOW=1`) with
angle PID 50/.001/4, speed PID .6/0/0, effort 30 and filter tau .030.
`BIG_YAW_PASSIVE_SMALL_TEST=0` keeps both controllers and the normal interlock
enabled. Pitch and chassis are unchanged. The rate-only helpers remain for
future comparison but no longer produce the small-yaw speed reference.

AC6: 0 errors, 85 warnings; Code 105288, RO 4092, RW 336, ZI 58880.
Log: `NoMachineTemp/yaw_follow_imu_restored_rebuild.log`.
AXF SHA256: `12686784FCF2E490A83B3CDC73DE01790267B0BB4ED11D38BF635F9A7A20B1A1`.
Recompiled small-yaw rate tests verify the diagnostic flag is disabled and
the retained helper's behavior. Existing immediate-follow, PID and soft-limit
regression executables pass. The restored angle path was inspected and compiled;
physical dual-axis stability has not been retested. No automatic flash or live
parameter write was performed. Keep the receiver UP during flash/Reset/Run,
then test away from mechanical stops and use SAFE if oscillation persists.

## Historical: immediate follow restored, small yaw still rate-only (2026-09-07)

Restored the user-requested immediate-follow policy and its former saved PID:
angle 50/.001/4, speed .6/0/0, effort 30, speed-filter tau .030. Each enabled
tick, including the first, sets the big-yaw reference to
`wrap(big_encoder_deg + relief_direction * small_joint_deg)`.
The small-joint center remains calibrated raw encoder 6816. There is no entry
threshold, dwell, prediction, moving-reference lead cap or velocity feedforward
in this policy. Centered mode means zero joint-centering error, not a captured
big-yaw encoder HOLD target. Validation and SAFE interlocks remain unchanged.

`YAW_COORD_IMMEDIATE_FOLLOW=1` selects the restored policy. Setting it to 0
and rebuilding selects the retained HOLD/relief implementation; the PID defaults
must also be reviewed when switching policies. Both policies have host tests.

Small yaw remains in the previously requested angle-open / gyro-speed-closed
test mode (`SMALL_YAW_ANGLE_OPEN_TEST=1`, `BIG_YAW_PASSIVE_SMALL_TEST=0`).
Its rate timeout, limits, yaw-vision/angle-step suppression and gains are
unchanged. Pitch and chassis are unchanged. This combination is not the full
historical firmware: the earlier stable immediate-follow run had small-yaw
angle holding, so combined stability must be checked again.

AC6: 0 errors, 85 warnings; Code 104576, RO 4092, RW 336, ZI 58880.
Log: `NoMachineTemp/yaw_follow_small_rate_rebuild.log`.
AXF SHA256: `5D0162CE6453336C36A60D6E1E2BF4D37D8092C01ECBACF6FFB128C1F4656B41`.
Recompiled immediate-follow, alternate HOLD/relief and PID tests pass; existing
small-yaw rate tests also pass. No flash, live parameter write or physical test.
Keep the receiver UP during flash/Reset/Run, read back gains before enabling,
start near the small-yaw center and return to SAFE if oscillation persists.

## Historical test: small-yaw angle open, gyro speed loop retained (2026-09-07)

At the user's request, `SMALL_YAW_ANGLE_OPEN_TEST=1` in
`Inc/Modules/module_small_yaw_rate.h` bypasses only the small-yaw IMU angle
PID. Manual yaw rate (or the existing PC rate command) goes directly to the
speed reference in rpm: deg/s divided by 6. The existing manual step cap is
converted back using the 2 ms control period; no new cruise-rate cap is added.
The original gyro speed feedback/filter, speed PID, effort budget, encoder
soft-limit speed/effort guards, online/IMU checks and SAFE remain active.

Yaw vision angle corrections and small-yaw angle tuning steps are ignored in
this build. Pitch vision handling is unchanged. Small-yaw tuning-control lock
requests zero rate rather than leaving an old command active. Released controls
request zero speed; the 20 ms command watchdog also clears an expired command.
SAFE or invalid/disabled small-yaw control clears the pending rate and step.
Rate and timestamp publication/consumption is atomic between the two tasks.

Angle diagnostics track measured attitude with zero angle error; this is a
bypassed angle controller, not evidence of perfect position tracking. A moving
base near a joint boundary can change the zero-speed target through the existing
soft-limit guard. Gyro feedback still resists angular motion, and the small yaw
can still react against big yaw. This test does not isolate all IMU influence.

Big yaw retains HOLD/relief with angle PID 7/.001/4, speed PID .6/0/0,
effort 30 and filter tau .030. `BIG_YAW_PASSIVE_SMALL_TEST` remains 0.
Pitch and chassis behavior are not intentionally changed. Set the angle-open
macro to 0 and rebuild/reflash to restore the original small-yaw angle loop,
vision correction and angle steps with its saved gains intact.

AC6: 0 errors, 85 warnings; Code 105988, RO 4092, RW 336, ZI 58880.
Log: `NoMachineTemp/small_yaw_angle_open_rebuild.log`.
AXF SHA256: `34C01D2E58E6D10FCD1C45CCA456522804FF2843BECE898E837CDA7D192EF4FA`.
New host tests pass for rate units/sign, release, timeout, tick wrap, invalid
input, reset and existing soft-limit composition. Existing PID, coordinator
and soft-limit regression executables pass. Physical timing and dual-axis
stability still require testing. No flash, serial command or motion performed.
Keep the receiver UP for flash and Reset/Run; test in MANUAL/DOWN with small
yaw near center and return to SAFE if sustained oscillation occurs.

## Historical: saved Kp 7 and small-yaw stabilization restored (2026-09-07)

Persisted the user's latest screenshot readback: angle PID 7/.001/4,
speed PID .6/0/0, effort 30, speed-filter tau .030. Only angle Kp changed
from the previous boot defaults. `BIG_YAW_PASSIVE_SMALL_TEST=0` restores
small-yaw enable at boot, its original IMU angle/rate loops and soft-limit
guards, and the requirement for an active small-yaw controller before big
yaw can run. Small-yaw gains and calibration are unchanged.

Big yaw still uses captured encoder HOLD and boundary-triggered relief,
not immediate centering. Pitch and chassis are unchanged. The supplied
gains were measured with small yaw passive; stability with both loops active
still requires a physical test. Keep the receiver UP while flashing and
using Reset/Run, verify browser readback, and return to SAFE if it oscillates.

AC6: 0 errors, 85 warnings; Code 106452, RO 4092, RW 336, ZI 58872.
Log: `NoMachineTemp/yaw_kp7_small_restored_rebuild.log`.
AXF SHA256: `E5F1390A0BB9A596FEA1E0BE1F87FC87DF5F0F632D685EB99BFE3C6919D90135`.
Recompiled PID regression verifies all boot values and the disabled isolation
flag. Existing HOLD/relief and soft-limit regression executables also pass.
No online parameter write, automatic flash or physical test was performed.

## Historical test: passive small yaw, saved big-yaw PID (2026-09-07)

The user requests removing the small-yaw closed loop to observe big-yaw HOLD
with the former immediate-follow gains. `BIG_YAW_PASSIVE_SMALL_TEST=1` in
`Inc/Modules/module_big_yaw_tune.h` forces small-yaw enable and effort to zero.
Its existing disabled path clears controller state and pending steps. A second
zero-effort guard runs immediately before the motor group is transmitted.
No encoder-only holding or speed damping replaces the IMU loop.

Only this test build bypasses the requirement that the small-yaw controller
be active before big yaw runs. IMU readiness, valid/fresh small and big motor
feedback, direction checks, remote SAFE and the output gates remain required.
Big-yaw HOLD/relief and all gains are unchanged: angle 50/.001/4, speed .6/0/0,
effort 30, speed-filter tau .030. Pitch and chassis are not changed.

This is not a validated stable preset. Keep the receiver UP during flash and
Reset/Run. Place small yaw near its measured center before enabling, keep clear
of moving parts, and observe big-yaw mode HOLD. Small yaw has no active holding
or soft-limit braking in this build; external motion can move it into a stop.
Boundary-triggered big-yaw relief is still enabled, so this is not a forced
HOLD-only test. Use the existing browser to read back the saved gains before
testing; do not reapply older unsaved edits automatically. Stop via SAFE if
oscillation occurs. Set the test macro to 0 and rebuild/reflash to restore the
original small-yaw loop and big-yaw dependency without changing small-yaw gains.

AC6: 0 errors, 85 warnings; Code 106484, RO 4092, RW 332, ZI 58884.
Log: `NoMachineTemp/yaw_passive_small_rebuild.log`.
AXF SHA256: `55021B0FAA2AE4857834E39B26349B9C8568C6FDA907E1A9D0BDE676572C6B5E`.
Recompiled host PID and HOLD/relief regression tests pass. The isolation gates
were inspected and compiled, but have not been exercised on the physical robot.
No remote connection, serial write, flash or motion was performed.

## Historical policy: HOLD/relief restored for user retuning (2026-09-07)

The user confirms oscillation disappeared with immediate following, then asks
to restore the less-moving policy so they can retune its PID themselves.
Restored the same captured encoder HOLD, boundary-triggered relief, dwell,
prediction, hysteresis, trajectory smoothing and moving-target lead guard.
Small-yaw IMU control and limits still use the measured center, raw count 6816.

No PID or effort change was made: angle 50/.001/4, speed .6/0/0, effort 30,
filter tau .030. These gains were tuned in immediate-follow mode and have
already produced oscillation with this policy. They are not a validated
HOLD/relief preset. Keep the receiver UP while flashing and using Reset/Run;
adjust gains in the existing browser and verify readback before enabling.
RAM edits still revert to these saved defaults on reset until explicitly saved.

AC6: 0 errors, 85 warnings; Code 106452, RO 4092, RW 336, ZI 58872.
Log: `NoMachineTemp/yaw_hold_retune_rebuild.log`.
AXF SHA256: `C05F293080C808456C35A1A6A61C9938DFACBE3B807DDD3436957BBCC15E7F5A`.
The artifact matches the previous saved-PID HOLD build. Restored coordinator
tests and existing saved-PID, UART mux and soft-limit tests pass. No serial
session, parameter write, automatic flash or physical test was performed.

## Historical policy: immediate-follow rollback, saved PID retained (2026-09-07)

The user reports high-frequency oscillation after switching to HOLD/relief
and requests the previous tuning-time logic. Restored immediate calibrated
recentring: each enabled update uses
`wrap(big_encoder_deg + relief_direction * small_joint_deg)`.
Joint zero remains raw encoder 6816. Removed captured HOLD, boundary entry/exit,
dwell, trajectory filtering/lead clipping and relief velocity feedforward.
Big yaw does not directly overwrite the small-yaw IMU reference.

Saved defaults are unchanged: angle PID 50/.001/4, speed PID .6/0/0,
effort 30, speed-filter tau .030. Small yaw, Pitch, soft limits, SAFE and
communication repairs remain unchanged. No live tuning or flash was performed.
Changing HOLD to joint-following changes the effective feedback path; this
may explain why the same gains behave differently, but the reported physical
oscillation has not been captured or conclusively attributed.

AC6: 0 errors, 85 warnings; Code 105032, RO 4092, RW 336, ZI 58864.
Log: `NoMachineTemp/yaw_saved_pid_follow_rollback.log`.
AXF SHA256: `3C18B29A8DB0A121ABCA4BA4171B97A53D9695C66F6F4A42D4541EC324930CAC`.
Immediate-follow C tests passed, including both signs, first-update response,
zero-offset target recapture, wrap, convergence and invalid/SAFE reset. Existing
saved-PID, UART mux and soft-limit regression binaries also passed. Physical
retest after manual gimbal flash is pending. Keep receiver UP for Reset/Run.

## Historical policy: saved PID with demand-based relief (2026-09-07)

At the user's request, the latest firmware restores the original HOLD/relief
policy and saves the supplied browser readback as boot defaults:

| Setting | Default |
| --- | --- |
| Angle Kp / Ki / Kd | 50 / 0.001 / 4 |
| Speed Kp / Ki / Kd | 0.6 / 0 / 0 |
| Effort limit | 30 |
| Speed filter tau | 0.030 s |

All six gains are initialized from the constant parameter arrays; I/D are no
longer cleared to zero during boot. Full-PID RAM writes remain available and
volatile. These defaults require a new gimbal flash, not a webpage refresh.

After leaving SAFE, capture and hold the current big-yaw encoder angle. Small
yaw keeps its existing IMU aiming loop and asymmetric soft limits referenced
to raw center 6816. Normal entry thresholds are about +20.36 / -39.67 degrees,
with 120 ms confirmation; prediction and imminent boundary cases can enter
earlier. Relief brakes after crossing +13.09 / -25.50 degrees toward center,
then holds the resulting big-yaw angle instead of chasing small-joint zero.
The original smoothing, hysteresis and moving-reference 4-degree lead guard
are restored. There is no fixed RPM ceiling on PID correction. HOLD still
actively rejects disturbance; 'less movement' does not mean disabling torque.

Small yaw, Pitch, chassis, SAFE guards, full-float protocol, circular USART1 RX
and browser serial repairs are unchanged. The user validated these gains with
immediate following; the combined gains and restored relief need a physical
transition check. No online parameter write, flash or motion was performed.

AC6: 0 errors, 85 warnings; Code 106452, RO 4092, RW 336, ZI 58872.
Log: `NoMachineTemp/yaw_saved_pid_hold_rebuild.log`.
AXF SHA256: `C05F293080C808456C35A1A6A61C9938DFACBE3B807DDD3436957BBCC15E7F5A`.
Actual PID/default tests and restored coordinator tests pass, including held
targets, dwell, prediction, hysteresis, symmetric relief, reversal and wrap.

## Historical policy: immediate calibrated-center following

This earlier user request temporarily superseded the demand-based policy below.
Every enabled control update sets big-yaw reference to
`wrap(big_encoder_deg + relief_direction * small_joint_deg)`.
Small joint zero remains the measured visual center, encoder count **6816**,
not the midpoint of the asymmetric mechanical stops. Any nonzero measured
offset is followed immediately, including the first enabled update. There
is no entry threshold, dwell, predictive relief, trajectory slew or 4-degree
lead clip. The angle/speed PID and its output clamp remain responsible for
the actual response; this does not guarantee instantaneous physical tracking.

Small-yaw IMU aiming and soft limits are unchanged. Its world reference is
not overwritten to force a joint angle. Big yaw still uses motor feedback,
not its own IMU loop. Relative small-motor rate is not fed forward because
it includes the big axis' own movement. SAFE, invalid feedback and offline
inhibition remain in effect. No PID gain or output default was increased.

The former position-hold behavior is intentionally gone: at zero small-joint
error the big axis has no separate captured chassis-relative angle target.
Both manual and automatic enabled modes use this new recentering policy.
Keep the receiver UP during flashing/reset; leaving SAFE can now command
immediate recentering if small yaw is displaced.

USART1 RX also now uses continuous circular DMA with HT/TC/IDLE draining and
a persistent CRC-checked frame mux. It fixes exact-64-byte write loss and
partial-frame loss. Browser/CLI writes are additionally spaced by 20 ms.
See the tuner README for the reception regression and verification limits.

AC6: 0 errors, 85 warnings; Code 105008, RO 4092, RW 336, ZI 58864.
Log: `NoMachineTemp/yaw_follow_uart_rebuild.log`.
AXF SHA256: `ABE424DFBAC5A9C214695FC376193CC6CEA26A41F2988721597CDDF653933142`.
C UART/mux/full-PID staging, recentering, PID and soft-limit tests passed.
69 browser assertions and 18 Python tests passed. No flash, online parameter
write or physical motion test was performed; hardware verification is pending.

## Historical demand-based policy

## Scope

The user reports the asymmetric small-yaw soft limits work on the vehicle.
This earlier revision preserves the endpoints, small-yaw PID gains and Pitch
tuning, but removes the small-yaw fixed speed plateau as described below.
It replaces continuous big-yaw recentering with encoder position hold and
on-demand joint-travel relief. The policy runs in both enabled manual and
automatic modes; visual or PC yaw targets still enter the existing small-yaw
IMU loop. Big yaw does not change the small-yaw world target.

`GimbalYaw_DiagBigEnable` now defaults to **1**. Upper remote SAFE still
zeros both yaw outputs. After leaving SAFE with fresh feedback and a ready
small-yaw loop, big yaw captures its current encoder orientation and holds
it. If small yaw is already near a boundary, relief will start automatically.
Start the first commissioning test with small yaw near visual center.

## Policy

Angles below are small-yaw joint angles relative to user-confirmed visual
center, raw encoder count 6816. Positive is toward the measured left stop.

| Side | Soft boundary | Normal entry | Exit into braking |
| --- | ---: | ---: | ---: |
| Left | +29.080 deg | +20.356 deg | <= +13.086 deg |
| Right | -56.678 deg | -39.674 deg | >= -25.505 deg |

- HOLD keeps a fixed big encoder target; it does not chase small-yaw zero.
- Ordinary entry requires 120 ms continuously beyond an actual or predicted
  entry threshold. Returning inside clears that side's pending timer.
- Prediction uses filtered small-motor joint speed (50 ms time constant),
  a 200 ms horizon, and an 8-degree maximum forecast displacement.
- Within 3 degrees of a soft boundary, or if the forecast reaches that
  boundary, relief bypasses the ordinary confirmation delay.
- LEFT/RIGHT move big yaw to restore small-yaw travel. Desired speed uses
  relief distance normalized by each side's travel from visual center. The
  longer right side is the reference scale, preserving its existing response.
  The exit pad is 2 degrees on the right and proportionally smaller on the
  left. Equal relative deflections now give equal steady relief demands;
  the shorter left side no longer produces an inherently weaker request.
- Actual small-yaw position crossing the exit threshold enters BRAKE.
  Once planned speed reaches zero, the final big target is held. Neither
  this final pose nor small yaw is forced to visual center.
- The latest revision removes the initial 36 deg/s planner/final-command
  caps and the angle PID's 6 rpm output cap. It also removes the 90 deg/s^2
  slew limiter from feedback correction. Planned velocity alone uses a
  40 ms first-order smoothing time constant, with no fixed velocity ceiling.
  This is not unlimited physical performance: relief gain, lead guard,
  output bounds and motor capability still determine the response.
- Moving position targets have a 4-degree lead guard relative to feedback.
  HOLD retains its fixed orientation under disturbance. The latest stiffness
  revision passes its full wrapped position error to the PID, without the
  former 4-degree error clip.

## Control and failure behavior

The coordinator is isolated in `Inc/Modules/module_yaw_coordinator.h`.
`module_gimbal.c` executes its target through the existing encoder angle and
speed PIDs, with applied-reference velocity feedforward and a 30 ms speed-feedback
filter time constant. Following the high-frequency oscillation report, default
big-yaw gains are restored to angle Kp 2.60 and speed Kp 0.60, with Ki/Kd zero
in both loops. The two axes keep
separate parameter arrays and controller states; this is not live tuning
coupling. Big yaw still uses encoder feedback, not the small-yaw IMU loop. The existing
speed PID output limit defaults to 4 independent driver-effort units (not Nm),
with an accepted tuning maximum of 8. The angle
PID uses FLT_MAX as its generic library output bound, removing its artificial
rpm cap. Moving-reference lead limiting remains in the planner, but HOLD
position error is no longer clipped. Actual motor speed is still constrained
by available drive output, load and feedback response.

Feedforward derives from actual reference advance after the lead guard,
bounded by planned velocity and smoothed over 10 ms to suppress encoder
quantization pulses. A blocked, stationary reference no longer retains a
nonzero velocity feedforward indefinitely. This smoothing only acts on
feedforward, not on PID error correction.

The initial big-yaw gains were angle Kp 0.50 and speed Kp 0.80. The first change
increases the angle gain but decreases the speed gain; it is not a uniform
increase in stiffness. The subsequent removal of fixed speed caps and
feedback slew limiting keeps those gains, the position-lead guard and
30 ms speed filter unchanged. Larger inertia alone does not establish
which gain or bound should increase. Compare position error, speed reference,
speed feedback and effort during commissioning before further tuning. See
the chronological build records below for the subsequent stiffness revision.

The planner uses wrapped absolute big encoder angles with shortest local
errors, not the driver's accumulated turn count. Its state stays bounded
through repeated revolutions. This holds encoder orientation relative to
the chassis, not an IMU/world heading.

SAFE, disabled big yaw, unavailable small-yaw stabilization, invalid
feedback/configuration or stale (>50 ms) big feedback stop big output and
reset the planner. Reentry captures current orientation instead of restoring
an old target. A control interval over 50 ms also resets it.

`GimbalYaw_BigReliefDirection` defaults to +1, assuming positive big encoder
motion reduces positive small-joint deflection while its IMU holds the view.
This geometric relationship needs physical verification. If opposite, change
this variable to -1 **in SAFE**, then retest. Do not reverse the native
encoder servo polarity to reverse relief: `GimbalYaw_DiagBigDirection` must
remain +1; other values now inhibit big output to avoid positive feedback.
Setting `GimbalYaw_DiagBigEnable=0` disables big hold and relief together.

Parallel but offset axes also translate the camera when big yaw rotates.
This revision does not implement finite-distance parallax compensation.
Small-yaw soft limits still take precedence over an unreachable view target.
Forecasts and output guards cannot guarantee tracking of arbitrary fast motion.

## Telemetry and validation

Read-only monitor packet 0x2C is `<4h4B`: big position error (centidegrees),
speed reference (centi-rpm), filtered speed feedback (centi-rpm), effort
(milli driver units), mode, actually active, relief direction negative,
protocol version (1). Modes: 0 DISABLED, 1 HOLD, 2 LEFT, 3 RIGHT, 4 BRAKE.
Packet 0x2B's big-enable byte is a request, not proof of an active servo;
use 0x2C for the actual coordinator state.

`yaw_limits_cli.py --observe --label check --duration 30 --json run.json`
records motion without locking RC or sending movement/parameter commands.
It requires fresh 0x2B/0x2C telemetry. Its accepted flag means capture quality,
not a successful or safe control response. Without --observe the utility
still requires stationary SAFE data for landmark acceptance.

Host tests in `Tests/test_yaw_coordinator.c` cover HOLD, short excursions,
asymmetric hysteresis, urgent/predicted entry, bounded velocity changes,
direction reversal, multiple revolutions, blocked-target lead limiting,
disable/re-enable, nonfinite inputs and relief settling in ideal kinematics.
These are not a physical plant, friction model or hardware safety proof.

For first use: support Pitch, select upper SAFE, place small yaw near visual
center, flash only the gimbal target, Reset then Run while still in SAFE.
Verify the new state packet before enabling. Then test big HOLD and a single
slow left excursion, followed by right. If big motion pushes small yaw
further toward a stop, return to SAFE immediately and check relief direction.
Do not begin with high-speed visual tracking or force either joint by hand
while its motor is enabled.

## Initial coordination build record

- AC6 V6.22 full gimbal rebuild: 0 errors, 85 warnings.
- Log: `NoMachineTemp/yaw_coordinator_rebuild.log`.
- AXF SHA256:
  `3E550021BD4BDC462A693CF1D265C4804E987F6F73F49E46346614F29E6F6FBB`.
- GCC C11 `-Wall -Wextra -Werror` coordinator tests passed. Ideal kinematic
  relief settled near -25.413 / +12.994 degrees and returned to HOLD.
- Existing small-yaw soft-limit and Pitch/referee regression executables
  passed again. No changes were made to their underlying guard algorithms.
- After explicit user authorization, all 8 Python observation/packet tests
  passed in `/tmp/sentinel-static-audit` on the vehicle PC. The read-only
  tool was deployed to `/home/nuc11--02/gimbal-pid-tuner/yaw_limits_cli.py`.
  Tests used mocked serial I/O; deployment did not open the live serial port.
- No firmware download, reset or motor command was issued during this work.
  No instrumented hardware validation was performed in that implementation turn.

## Big-yaw gain alignment

The user subsequently reported testing the coordination firmware and finding
big yaw too soft, then requested reuse of the small-yaw PID gains. Only the
two big-yaw Kp defaults were changed for this iteration. Small-yaw gains,
Pitch, coordination thresholds, velocity/acceleration caps and output limits
are unchanged. Matching gains is a starting point, not validation on the
heavier axis. This iteration requires a new gimbal flash and low-speed
hardware testing; no live parameter or motor command was sent.

- AC6 full rebuild: 0 errors, 85 warnings.
- Log: `NoMachineTemp/yaw_big_gain_alignment_rebuild.log`.
- AXF SHA256:
  `4F7D22C3FD72B1B182A4D834654FEFDB24FD43898B665961E6703AD5D7EAD62B`.
- Unchanged coordinator and small-yaw soft-limit regression executables
  passed again. Those tests do not simulate the big motor's new PID response.

## Feedback-path revision after oscillation report

The user reported large low-frequency big-yaw oscillation after gain alignment
and requested removal of restrictive speed caps. This source revision removes
the planner/final 36 deg/s caps, the angle PID's 6 rpm cap and feedback-command
acceleration slew. The old slew delayed reversal of position corrections and
is a suspected contributor, not a proven sole cause of the oscillation.

Planned velocity uses 40 ms smoothing instead of a fixed acceleration bound.
Actual-reference velocity feedforward is clipped to achievable reference
advance and smoothed over 10 ms. The PID correction bypasses both smoothers.
Gain defaults remain 2.60/0.60, the feedback filter remains 30 ms and maximum
motor effort remains 4.0. The small-yaw soft guard, reference lead guard,
SAFE, offline shutdown and nonfinite-command rejection remain enabled.
Do not interpret removal of explicit speed caps as unlimited motor speed,
increased available torque or verified stability.

- AC6 full rebuild: 0 errors, 85 warnings.
- Log: `NoMachineTemp/yaw_feedback_unpaced_rebuild.log`.
- AXF SHA256:
  `E6889248D451FB9D58F3DF55FAB853CE0B04C3B6A4D98C4787E233A10560FED7`.
- Updated coordinator tests verify planning above the old 36 deg/s ceiling,
  finite settling and vanishing feedforward under a stationary clipped target.
  Small-yaw soft-limit and existing Pitch/referee regressions passed again.
- No active motor command or firmware download was issued. The real low-
  frequency oscillation has not been reproduced or shown to be fixed by this
  revision; verify holding stability before high-speed tracking.

## Symmetric relief and optional rate caps (2026-09-06)

The user now reports that low-frequency oscillation is gone, right rotation
is faster and left rotation is still slow. The absolute-distance relief law
was one identifiable asymmetry in code, not proof of the only physical cause.
Normalize left relief distance by `abs(soft_min) / soft_max` (about 1.949),
including a proportionally scaled exit pad. Right-side demand, both PID gain
sets, feedback filters and motor output bounds are unchanged.

Small yaw now boots with no fixed angle-PID rpm cap or command-step cap.
The vision step's former 0.05 deg/tick clamp is also removed; its gain,
deadband and filter remain. The remote mapping remains 132 deg/s at 660
counts: this is command sensitivity, not a clamp on stabilization response.
The PC signed centidegree/s field is unchanged. No fixed big-yaw speed or
feedback-acceleration cap has been reintroduced. Lead guards, trajectory
smoothing and effort limits remain; this does not promise infinite response.

`GimbalYaw_TuneSmallSpeedLimitRpm` and
`GimbalYaw_TuneSmallManualMaxStepDeg` default to **-1**, the explicit no-cap
sentinel. Zero retains its previous meaning; positive values still enable
optional tuning caps. Command 0x20 encodes -1 as **0xFFFF** in the respective
uint16 fields. Ordinary existing values retain their units and meaning.
The local browser tuner exposes checked "no fixed cap" checkboxes and the
CLI uses `--speed-limit -1 --manual-step -1`. Old tools explicitly sending
24 / 0.30 will restore those caps. Use these sentinel values only with the
new firmware; old firmware does not implement this convention.

The mechanical speed envelope is deliberately still active. Without a fixed
cap it is `[-3*(joint-soft_min), +3*(soft_max-joint)]` rpm inside the soft
range, transformed between joint and world speed with base-motion feedback.
The old 24 rpm plateau is gone, while the previous braking slope within
8 degrees of either stop is unchanged. Outward speed is zero at/beyond its
boundary; inward recovery and the effort-direction guard remain. A larger
allowed approach speed still needs physical braking validation: the envelope
is not a demonstrated stopping-distance model.

- AC6 full rebuild: 0 errors, 85 warnings; Code 98456, RO 4088, RW 336, ZI 58600.
- Log: `NoMachineTemp/yaw_symmetric_uncapped_rebuild.log`.
- AXF SHA256:
  `7DB90832C3008D76332B0D03DDCAA2ECE98C043273BA5B67D4D854EA785B0F16`.
- GCC tests passed for matched relative left/right requests, unlimited/capped
  speed behavior, boundary slope, moving-base compensation and wire decoding.
  Existing Pitch/referee regression executable passed again.
- Browser JavaScript syntax and both cap-encoding branches passed offline
  checks. Browser rendering was not tested. Added Python CLI codec tests
  were not run: no local Python runtime was available.
- No firmware flash, motor command or remote tool deployment was performed.

## Big-yaw stiffness commissioning (2026-09-06)

The user reports vehicle motion displacing big yaw and asks for stronger
holding stiffness and rapid correction. This revision targets encoder-relative
disturbance rejection. It does not add big-yaw IMU stabilization: a rotating
chassis still changes big yaw's world direction, and small yaw must compensate.

| Setting | Previous | New default |
| --- | ---: | ---: |
| Angle Kp, rpm/deg | 2.60 | 4.00 |
| Speed Kp, effort/rpm | 0.60 | 1.20 |
| Speed-feedback filter time constant | 30 ms | 8 ms |
| Maximum drive effort magnitude | 4.0 | 8.0 |
| HOLD position error input | clipped to +/-4 deg | full wrapped error |
| Fixed speed/feedback slew limit | none | none |

The unsaturated zero-speed proportional effort per degree changes from
1.56 to 4.8 (about 3.08 times). This is a controller arithmetic ratio, not
a measured mechanical stiffness or a stability guarantee. Ki/Kd stay zero;
no disturbance integrator or chassis acceleration feedforward is introduced.
Planner thresholds, symmetric relief, smoothing, applied-reference
feedforward and the moving-reference 4-degree lead guard remain unchanged.
Small-yaw gains, its boundary guard and Pitch settings are unchanged.

Big effort no longer passes through the small-yaw positive tuning cap.
However, a zero/nonfinite small-yaw effort budget still inhibits big yaw,
preserving the zero-effort abort interlock. SAFE, stale feedback, invalid
direction, unavailable stabilization and delayed control ticks still inhibit
output. Invalid big tuning values inhibit output instead of falling back to
an unexpectedly strong default. A zero big effort budget disables big control.

The driver encodes effort multiplied by 1000, so 8.0 sends at most magnitude
8000 in the CAN command. This is not 8 amperes or 8 Nm. The GM6020 voltage
command format and feedback temperature byte are documented on pages 6-7
of the [DJI GM6020 user guide](https://www.mouser.com/datasheet/2/744/RoboMaster_GM6020_Brushless_DC_Motor_User_Guide-1551074.pdf).
The protocol's numerical maximum is not a safe sustained loaded output.
The GM6020 decoder previously forced temperature to zero; it now reads byte 6.
This adds temperature visibility, not a new firmware thermal shutdown policy.

### Watch parameters

The following volatile variables are applied by the big-yaw loop independently
of small-yaw command 0x20. Edit them in SAFE, then enable to evaluate the change:

- `GimbalYaw_TuneBigAngKp`: default 4.0, accepted 0 through 8.
- `GimbalYaw_TuneBigSpdKp`: default 1.2, accepted 0 through 5.
- `GimbalYaw_TuneBigEffortLimit`: default 8.0, accepted 0 through 8.
- `GimbalYaw_TuneBigSpeedFilterTauS`: default 0.008, accepted 0 through 0.1.

These bounds validate settings; they are not speed caps or verified stable
tuning ranges. Runtime edits are not persisted across restart. To reproduce
the old gains/filter/output budget, use 2.6, 0.6, 4.0, 0.030 respectively;
this does not restore the removed HOLD error clip.

Existing packet 0x2C and `yaw_limits_cli.py --observe` remain compatible.
Watch adds `GimbalYaw_DiagBigSaturated` (speed PID output clipping),
`GimbalYaw_DiagBigCurrent` (raw feedback current, no amp conversion), and
`GimbalYaw_DiagBigTemperatureC`. Current/temperature expose the most recent
CAN sample, including in SAFE; check motor age/online state before interpreting
them. The new fields are not sent by the existing serial packet.

First validate stationary HOLD near small-yaw center, then record controlled
vehicle start/stop disturbances while staying clear of powered joints. Large
error with effort pinned at +/-8 calls for examining available drive output,
load and supply before raising gains further. Alternating large efforts and
oscillation call for damping/filter/gain reassessment. Return to SAFE if
sustained oscillation or abnormal heating occurs; do not hand-load the rotor.
Output saturation indication alone does not diagnose the cause of displacement.

### Build and test record

- AC6 full rebuild: 0 errors, 85 warnings; Code 99220, RO 4088, RW 336, ZI 58624.
- Log: `NoMachineTemp/yaw_big_stiffness_rebuild.log`.
- AXF SHA256:
  `D66ECF439E57AA7A585B43E91A43C2BF7E87508554AB1592568407C246FD698C`.
- `Tests/test_big_yaw_pid.c` passed against the real `alg_pid.c`, `alg_math.c`
  and `alg_filter.c`. Tests cover default arithmetic, correction/damping signs,
  saturation and immediate reversal, zero budget, reset and tune rejection.
  The host build needs `-Wno-error=parentheses` for two existing unrelated
  bitwise-OR condition warnings in `alg_math.c`; those sources were not edited.
- Existing coordinator, small-yaw soft-limit and Pitch/referee regression
  executables passed again. No physical plant simulation was substituted for
  vehicle testing. The changed CAN temperature decoding was source-reviewed
  and AC6-compiled but not exercised with an injected CAN frame.
- No flash, reset, live parameter command or motor movement was performed.
  These are stronger commissioning defaults, not a completed hardware PID tune.

## PC tuning and oscillation rollback (2026-09-06)

The user reported strong high-frequency oscillation with the high-stiffness
defaults and confirmed upper SAFE. Defaults are now 2.6 / 0.6 / 4.0 / 0.030 s
for angle Kp, speed Kp, effort and feedback filter time constant. These restore
the prior parameter values, not every detail of the older firmware: full HOLD
error correction and independent big effort remain. No fixed speed/slew cap
is reintroduced. Small yaw, Pitch and coordination geometry are unchanged.

USART1 now supports a separate big-yaw parameter command and readback. The
tool is deployed as `/home/nuc11--02/gimbal-pid-tuner/big_yaw_tune_cli.py`.
Command details and invocation are in the PC tool README. Default invocation
only requests monitoring and reads configuration. Writes require explicit
`--apply --arm I_CONFIRM`; omitted fields retain the fresh firmware readback,
not preset host defaults. The transaction sends one write and requires matching
request ID, accepted status and all four returned parameters. An ACK timeout
is an unknown result, not proof of rollback; query before attempting another write.

Firmware accepts a write only if the receiver is connected/fresh and right
switch UP both at reception and application, and yaw output state plus both
motor commands are zero at application. Writes older than 100 ms are rejected.
The IRQ mailbox is consumed atomically and rejected writes are never held for
later execution. No command enables an axis, changes the target, sends a step,
or refreshes normal PC motion-command liveness. Parameters are RAM-only and
reset to defaults after restarting. Use SAFE to write, then change the remote
mode manually for a separate response test.

- AC6 rebuild: 0 errors, 85 warnings; Code 101060, RO 4092, RW 336, ZI 58664.
- Log: `NoMachineTemp/yaw_big_pc_tune_rebuild.log`.
- AXF SHA256:
  `5202D20627AACE551C0B5314E76425661A2C275CAD7C7A0AA1881E21DAA04123`.
- Real PID C tests passed with rollback defaults and receive/apply SAFE,
  outputs-off, expiry and invalid-value policy cases. Existing coordinator,
  small-limit and Pitch/referee executables also passed.
- All 6 Python codec/transaction tests passed on the vehicle PC using fake
  frames, including read-only behavior, partial updates, unsafe-write refusal,
  rejected/mismatched ACKs and CRC/fragmentation. Deployed `--help` passed.
- No serial port was opened, parameter applied, motor moved or firmware flashed
  during implementation. End-to-end readback from the new firmware remains to
  be checked after the user flashes it, still in SAFE. Existing 0x2C observation
  is approximately 10 Hz; it cannot resolve arbitrary high-frequency vibration.

## Task-rate peak diagnostic revision (2026-09-06)

Added monitor-only packet 0x2F with per-control-iteration enabled big-yaw
angle-span/error/raw-speed/command-effort peaks and saturation indication.
Window time and sample count expose acquisition coverage; stale feedback,
task gaps over 10 ms and overflow invalidate the window. See
`gimbal-pid-tuner/README.md` in the workspace for the six-word wire format
and `yaw_limits_cli.py --observe --require-peaks` for capture validation.
Control laws, default gains, limits and other axes are unchanged.

AC6 rebuild: 0 errors, 85 warnings; Code 102540, RO 4092, RW 336, ZI 58704.
Log: `NoMachineTemp/yaw_big_peaks_rebuild.log`.
AXF SHA256: `B37157F8B2025856416E63B301B79642C7DB70A0C9571D816E79787B0096EEBB`.
Pure C peak tests and 21 offline Python tests passed. Awaiting manual flash
and SAFE telemetry verification; no claim of on-hardware timing validation.
Restart defaults are still 2.6/0.6/4.0/0.030, not the last RAM-only trial.

## Full-effort range revision (2026-09-06)

At the user's request, BIG_YAW_EFFORT_MAX is now 30.0 instead of 8.0.
GM6020 CAN voltage commands have full scale +/-30000; the existing motor
serializer multiplies effort by 1000. The bound therefore preserves the motor
protocol range and signed 16-bit command/telemetry representation. This is
not a current/torque rating or a validated continuous-duty output limit.

0x2E flags bit 5 now advertises the 0..30 effort range. Updated browser and
CLI retain 0..8 for older firmware and do not transmit above-8 settings until
this capability is present. Packet layout, default 4.0 effort, Kp defaults,
I/D=0, SAFE rules, offline guards and small-yaw/Pitch control are unchanged.
No extra speed cap was added. This build also includes the previous 0x2F
task-rate peak diagnostics, which had not yet been flashed by the user.

AC6 rebuild: 0 errors, 85 warnings; Code 102540, RO 4092, RW 336, ZI 58704.
Log: `NoMachineTemp/yaw_big_full_effort_rebuild.log`.
AXF SHA256: `494B8A853BF8A11E36C53DD556B6D44FD0EB82385874B23FE3636BF5FAC326A1`.
C real-PID tests cover +/-12, +/-30, zero and out-of-range rejection, including
the +/-30000 conversion. 56 mock-browser checks and 15 offline Python tests
passed. No live output change or firmware download was performed. Manual
flash and fresh readback remain necessary; reset restores 2.6/0.6/4.0/0.030.

## Complete big-yaw PID revision (2026-09-06)

Both big-yaw loops now use independently adjustable Kp/Ki/Kd. The float32
0x30 transaction / 0x31 snapshot protocol replaces the legacy fixed-point
ceiling for capable clients; 0x2E bit 6 advertises support. All six gains and
filter tau accept finite nonnegative float32 values without arbitrary upper
limits. Effort is still bounded by the GM6020 CAN full scale of 30 units.
See the tuner README for field ordering, flags, atomicity and timeout rules.

The existing discrete per-update Ki/Kd convention is retained. Conditional
integration prevents either loop accumulating into downstream saturation;
big-yaw resets clear all PID and filter histories. Non-finite arithmetic
stops big-yaw output. No fixed speed ceiling, defaults change, other-axis
change, online parameter write or automatic flash was introduced.

AC6 rebuild: 0 errors, 85 warnings; Code 105580, RO 4092, RW 336, ZI 58848.
Log: `NoMachineTemp/yaw_big_full_pid_rebuild.log`.
AXF SHA256: `D78E772A5BC066FFD53E145E5C61AE544C262DB33763CCCFEA5A07DD9B2664FC`.
Real-PID and atomic-staging C tests passed, including I/D response, reset,
anti-windup, numerical overflow rejection and incomplete/unsafe transactions.
68 mock-browser assertions and 17 offline Python tests passed. Manual flash
and hardware verification remain pending. Boot defaults remain angle Kp 2.6,
speed Kp 0.6, effort 4, tau 0.030, all four Ki/Kd zero, not the latest RAM trial.
