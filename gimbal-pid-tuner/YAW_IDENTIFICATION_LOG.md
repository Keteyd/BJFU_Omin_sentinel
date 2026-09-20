# Dual-yaw identification observations

## 2026-09-08: first live capture

Recorder deployed independently to
`/home/nuc11--02/yaw-capture-v1-MHFlYj6t` on `192.168.1.113`.
Seven Linux offline tests passed; the Windows C executable fixture test was
skipped on Linux (previously passed locally). The initial `probe_01` received
zero bytes. After the user confirmed the capture firmware was running,
`captures/stationary_01` successfully recorded 400 complete observations.

Local evidence: `NoMachineTemp/yaw-capture-stationary-20260908-01/` contains
`raw.bin`, `samples.csv` and `report.json`. No motor command, enable request
or parameter update was sent. Only the observation heartbeat was transmitted.
No MPC model has been fitted to this passive recording.

| Observation | Result |
| --- | --- |
| Complete records | 400, all decoder validity checks passed |
| MCU observation interval | 25 ms for every adjacent complete record |
| Span between first and last record | 9.975 seconds |
| CRC / missing groups / incomplete groups | 0 / 0 / 0 |
| Initial non-PC bytes | 15 discarded bytes, consistent with one pre-session vision frame |
| MCU skipped observation slots | 0 |
| IMU and both CAN ages | 0..1 ms |
| IMU sequence growth | 9290 updates over 9.975 seconds, about 931 Hz |
| Sampled EKF dt | 0.000957827..0.002003815 seconds |
| Yaw first / last | -86.732918 / -86.011383 degrees |
| Net yaw change | +0.721535 degrees, average +0.072334 deg/s |
| Mean INS gyro X / Y / Z | -0.00212110 / -0.00300032 / +0.000576308 rad/s |
| Big software drive command | exactly zero throughout |
| Small software drive command | exactly zero throughout |
| Receiver yaw channel | zero throughout |
| Raw flags | 206 throughout |

Flags 206 indicate INS ready, both motor online bits, small-yaw enable setting
and compile-time big-yaw inhibition. Shared yaw output and big controller
active bits are clear. Crucially the receiver-SAFE bit is also clear: fresh
receiver UP has NOT been verified. Zero receiver channel is not proof of a
connected centered receiver. Ask the user to confirm receiver power/link and
switch position before any powered experiment; do not infer SAFE from zero
motor commands alone.

The capture was requested with no movement; actual stationarity still needs
user confirmation. The net heading change warrants a stationary-bias follow-up,
not an immediate PID change. Body gyro Z must not be equated to Euler yaw rate
at this approximately -27-degree roll attitude. This recording demonstrates
IMU publication and angle updates, but does not validate dynamic direction,
IMU-to-joint mounting, motor feedback scaling or closed-loop stability.

Next: confirm receiver state and stationarity, then schedule a separate
hand-moved, unpowered capture of the IMU-bearing assembly. High-rate buffered
capture and bounded excitation remain prerequisites for actuator identification.

## 2026-09-08: confirmed stationary, receiver UP

The user subsequently reported that the receiver had only just been switched
on, explaining the missing receiver-SAFE confirmation in the first recording.
After explicit confirmation of UP and stationarity, `captures/stationary_02`
was recorded using only monitor heartbeats. No actuation, tuning, reset or
flash was performed. The serial handle was closed after collection.

Local evidence: `NoMachineTemp/yaw-capture-stationary-20260908-02/`.

| Observation | Result |
| --- | --- |
| Complete / invalid records | 400 / 0 |
| CRC / discarded bytes / missing / incomplete groups | 0 / 0 / 0 / 0 |
| MCU observation interval / span | 25 ms / 9.975 seconds |
| MCU skipped observation slots | 0 |
| Raw flags | 207 throughout: receiver UP now confirmed |
| IMU and both CAN ages | 0..1 ms |
| IMU sequence growth | 8702 updates, about 872 Hz |
| Sampled EKF dt | 0.000955048..0.001996744 seconds |
| Yaw first / last | -54.255413 / -53.609848 degrees |
| Net yaw change | +0.645565 degrees, average +0.0647183 deg/s |
| Mean INS gyro X / Y / Z | -0.00172429 / -0.00182853 / +0.00239525 rad/s |
| Roll range | -14.056676..-13.993701 degrees |
| Big / small encoder span | 0.043945 / 0.043945 degrees |
| Big / small software drive command | exactly zero throughout |
| Receiver yaw channel | zero throughout |

Flags retain big-yaw inhibition and no shared yaw output. The IMU publishes
fresh data, but heading drift persists during user-confirmed stationarity.
This does not establish the cause of the earlier stick-release runaway, nor
does it validate the dynamic feedback sign. Do not subtract a constant fitted
from this one short recording or alter PID gains based on it. The two captures
have different starting attitudes and are not a continuous stationary record.

Next: a separately coordinated hand-moved capture with the receiver kept UP,
checking joint/IMU signs and return-to-rest behavior without powered tests.
Investigate stationary gyro bias and estimator timing before model fitting.

## 2026-09-08: passive small-yaw hand motion

After the user confirmed readiness, recorded 40 seconds to remote
`captures/manual_small_01`; local evidence is
`NoMachineTemp/yaw-capture-manual-small-20260908-01/`. The start cue was sent
after the recorder reported that it had started. Only monitor heartbeats were
sent; the port was closed at completion. No firmware or controller changes.

| Observation | Result |
| --- | --- |
| Complete / invalid records | 1600 / 0 |
| CRC / missing / incomplete groups | 0 / 0 / 0 |
| Initial discarded bytes | 15 |
| Span / skipped slots | 39.975 seconds / 0 |
| Flags | 207 throughout, receiver UP |
| Both software drive commands | zero throughout |
| IMU and both CAN ages | 0..1 ms |
| Small encoder range | 269.472656..324.975586 degrees |
| Big encoder range | 352.221680..353.540039 degrees |
| IMU yaw range | -83.681091..-28.165585 degrees |
| Small encoder speed | -6..7 rpm |

For 250 ms increments, let x be the sum of big and small encoder changes and
y be the IMU yaw change. Selecting abs(x) > 0.5 degrees gives 530 overlapping
increments. Least-squares y = slope*x + intercept gives slope 0.999154,
intercept 0.025808 degrees per increment, and Pearson correlation 0.999546.
No encoder wrap occurred in this recording. These overlapping samples are
descriptive, not independent observations for confidence bounds. Subject to
the requested stationary chassis, the result supports matching kinematic
angle signs and approximately unit scale at this attitude. The small motion
of the big joint was included rather than assumed to be zero.

Sampled body gyro Z and small encoder rpm also have matching signs during
motion (correlation 0.985673 for abs(small_rpm) >= 2, 290 samples). This is
not a gyro scale calibration: raw rpm is quantized, body Z is not Euler yaw
rate at the measured roll, and the big joint moved slightly.

Actual small-joint span was 55.502930 degrees, greater than the requested
approximately 10-degree excursion. Motion was still present late in the
recording, so it is not a completed return-and-rest experiment. Feedback is
not frozen or sign-reversed during these hand motions, but commanded motor
polarity, closed-loop stability and inertial coupling remain untested.
Do not fit actuator dynamics to this externally hand-driven, zero-command
recording. Continue with bias/timing review and high-rate capture preparation
before a separately authorized bounded actuator identification experiment.

## 2026-09-08: offset initialization follow-up

Targeted code check found an unread Flash buffer used as calibration data in
`BMI088_GetOffset`. Replaced it with existing deterministic defaults and stopped
`BMI088_Init(1)` overwriting an explicit calibration result. Current startup
remains `BMI088_Init(0)`; neither a new bias fit nor online calibration was
enabled. No claim that this fixes measured drift or the earlier runaway.

Actual-driver mocked register tests and all eight capture tests passed. AC6
rebuilt with zero errors and 85 warnings, but has NOT been flashed by the agent.
Artifact hash: `A9A4DFE4D64464DE8A5FB97D6C13417B4AD264815F6D37BB016ECF9DA5E84FAA`.
See `AGVSentinel_v10090/AGVSentinel_gimbal/YAW_STATIC_CHECK_2026-09-08.md` for
the reference-path review, unresolved validity/timing risks and next checkpoint.

## 2026-09-08: stationary baseline after offset initialization fix

User confirmed the repaired firmware was running, receiver UP and stationary.
Captured remote `captures/stationary_offset_fix_01`, copied to local
`NoMachineTemp/yaw-capture-stationary-offset-fix-20260908-01/`.
Only monitor heartbeats were sent. Serial handle closed normally. Firmware
identity is based on the user's flash confirmation, not device hash readback.

| Observation | Result |
| --- | --- |
| Complete / invalid records | 400 / 0 |
| CRC / missing / incomplete groups | 0 / 0 / 0 |
| Initial discarded bytes | 10 |
| MCU interval / span / skipped slots | 25 ms / 9.975 seconds / 0 |
| Flags | 207 throughout: receiver UP, big output inhibited |
| Receiver yaw channel | zero throughout |
| Both software drive commands / encoder rpm | zero throughout |
| Big encoder range | 352.705078 degrees, unchanged |
| Small encoder range | 306.562500..306.606445 degrees |
| IMU age / big CAN age / small CAN age | 0..2 / 0 / 0..1 ms |
| IMU sequence growth | 7646 updates, about 767 publications/s |
| Sampled EKF dt | 0.000921738..0.002005649 seconds |
| Yaw first / last | -87.329613 / -86.784103 degrees |
| Net yaw change | +0.545509 degrees, average +0.0546877 deg/s |
| Mean INS gyro X / Y / Z | -0.00147129 / -0.00295238 / +0.0000729703 rad/s |
| Roll range | -14.192080..-14.126345 degrees |

Drift persists. The prior confirmed stationary capture changed +0.645565
degrees in the same span, but boot age/temperature and starting conditions
are not controlled, so this difference is not evidence of improvement from
the fix. Do not subtract average body-Z gyro as an Euler heading correction.
Observed INS publication frequency is not sensor hardware ODR, nor control
loop frequency; v1 only records sampled INS dt and lacks hardware read-success
proof as described in the static check. Zero channel here does not exclude
a residual after stick release, because the stick was not exercised.

Next: separately coordinate an UP-only receiver stick/release capture, leaving
the vehicle stationary and both yaw outputs off. No powered test or PID change
is justified by this baseline alone.

## 2026-09-08: receiver release capture, no stick movement observed

Following readiness confirmation, a 40-second UP-only session was started as
remote `captures/rc_release_01`. The movement cue was sent after recorder
startup. Local evidence: `NoMachineTemp/yaw-capture-rc-release-20260908-01/`.
No actuation or tuning command was sent; the port closed normally.

- 1599 complete valid records spanning 39.95 seconds, adjacent ticks 25 ms.
- Zero CRC errors, missing complete groups, duplicates or MCU skipped slots.
- One incomplete group and 15 initially discarded bytes were reported; do
  not describe this recording as having no incomplete data.
- Flags 207 throughout; both yaw software commands stayed zero.
- Channel 2 was exactly zero in every complete sample. No release event can
  be identified, so this session does not validate return-to-center behavior.
- Big encoder stayed at 352.705078 degrees; small encoder span was only
  0.087891 degrees. Yaw changed from -73.710121 to -71.037140 degrees.

Source check: capture reads `Remote_GetRemoteDataPtr()->remote.ch[2]` directly.
The receiver decoder subtracts the channel midpoint; UP does not itself clear
this field. Resetting all channels is used on initialization or invalid remote
data, not as an UP-only operation. Confirm action timing and which stick was
moved before scheduling a repeat or diagnosing the receiver. Do not assume
the user failed to move, that a different channel was used, or that the
telemetry proves a healthy release; these remain alternatives to distinguish.

### User reports physical movement after this capture

The user reports moving the left-hand stick and seeing the small yaw move
back and forth. This contradicts the captured stationary encoder values,
zero software outputs and UP flags if both refer to the same time interval.
Treat the discrepancy as unresolved, not evidence that the user did not move
the stick or that SAFE definitely failed. Paused further stick/powered tests,
asked for UP with sticks released and confirmation both yaw axes have stopped;
if motion persists, disconnect motor power. No remote commands were sent in
response to this report.

Local review confirms `PC_Comm_IsRemoteSafe` and `Control_UpdateTargets` both
use `remote.s[1]`; UP decoding is not remapped in `Remote_ToSwitchState`.
Capture uses channel 2 and the corresponding motor objects directly. This
static consistency does not prove actual on-device state, physical switch
mapping, delivered CAN commands or synchronized action timing. Clarify the
switch position during the reported movement and whether it overlapped the
40-second recording before interpreting the release test or changing code.

### Clarification and repeat: rc_release_02

The user clarified that the reported physical motion occurred with the right
switch DOWN, having misunderstood the instruction, and both yaw axes stopped
when switched UP. This does not establish a SAFE failure. The exact overlap
with rc_release_01 remains unknown; that session cannot be used as a release
test. After a new readiness confirmation, explicitly requested that the right
switch stay UP while only the left-hand stick was moved.

Recorded remote `captures/rc_release_02` for 40 seconds, using monitor
heartbeats only. Local evidence:
`NoMachineTemp/yaw-capture-rc-release-20260908-02/`. Serial closed normally.

| Observation | Result |
| --- | --- |
| Complete / invalid records | 1600 / 0 |
| CRC / missing / incomplete groups / skipped slots | 0 / 0 / 0 / 0 |
| Initial discarded bytes | 15 |
| Flags | 207 throughout, receiver UP |
| Big and small software commands | zero throughout |
| Receiver yaw channel range | -235..369 |
| Big encoder | 354.243164 degrees throughout |
| Small encoder range | 316.801758..316.889648 degrees |
| Yaw first / last | -38.176636 / -35.613350 degrees |
| Net yaw change | +2.563286 degrees over 39.975 seconds |

Exact-zero/nonzero run segmentation of the sampled receiver channel:

| Relative MCU time (s) | Samples | Channel range |
| --- | --- | --- |
| 0.000..26.975 | 1080 | 0 |
| 27.000..31.850 | 195 | -235..-1 |
| 31.875..35.950 | 164 | 0 |
| 35.975..38.450 | 100 | 2..369 |
| 38.475..39.975 | 61 | 0 |

Both sampled excursions return to exact zero, with about 4.1 seconds and
1.5 seconds of zero-channel observations afterward. No persistent center
residual was observed in this run, so do not introduce a new deadband as a
claimed fix for runaway. This UP-only test does not exercise enabled target
integration, commanded motor polarity or closed-loop stability. Persistent
heading drift is still visible despite stationary joint encoders.

Next engineering work: hardware-read validity propagation and high-rate
buffered reference/feedback/output/control-dt diagnostics before a separately
authorized powered identification test. The input mapping and return-to-zero
observation do not substitute for that enabled-loop evidence.

## 2026-09-08: buffered diagnostics prepared, not flashed

Implemented checked synchronous BMI088 reads and an INS validity streak/dt
guard, plus a one-shot 512-record nominal-500-Hz buffer that downloads after
acquisition. V1 live capture remains supported. Actual task intervals are
recorded, but PID timing/gains and output isolation remain unchanged.

Details, wire format, verification caveats and command:
`gimbal-pid-tuner/YAW_BURST_WORKFLOW.md`.
PC deployment: `/home/nuc11--02/yaw-capture-v2-MoZJI3sA`.
Windows 12 protocol tests and driver fault-injection tests passed. NUC offline
suite: 10 passed, 2 Windows fixture tests skipped. No serial access this turn.
AC6: zero errors, 85 warnings. Artifact SHA256:
`0732D8A1D16D441D331C43C33BF1A5F7123E35FC622B2C539ADA508EADDC57FF`.
First hardware verification must remain UP and stationary. No powered test,
new calibration fit, MPC model, flash or reset performed by the agent.

## 2026-09-08: burst firmware first hardware check failed

User confirmed the high-rate diagnostic build running, UP and stationary.
Read v1 telemetry for five seconds before attempting v2. Remote capture:
`/home/nuc11--02/yaw-capture-v2-MoZJI3sA/captures/static_live_01`.
Local evidence: `NoMachineTemp/yaw-burst-first-live-20260908-01/`.

- 200 complete records, all 200 invalid; zero CRC/missing/incomplete groups.
- MCU tick 39009..43984; IMU sequence 90 throughout.
- IMU age 582..5557 ms; no accepted INS publications in this interval.
- Flags 205 throughout: UP and motor-online bits, INS-ready clear.
- Both yaw software commands and both motor rpm zero throughout.
- Heading -90.039330 degrees throughout; not evidence of improved drift.

Stopped the planned high-rate test. No actuation/tuning command was sent.
Sensor faults, SPI transfer compatibility and dt rejection were not separately
observable in this build. Do not label the stopped update as hardware failure
or claim an exact software cause from the capture alone.

Prepared an unflashed compatibility revision: restore legacy byte-at-a-time
SPI transactions with checked HAL results (10 ms per-byte timeout), retaining
ID checks, valid streak, dt guards and output isolation. Added periodic read
failure/success/stage/ID and INS rejection diagnostics to the capture stream.
Updated tools independently at `/home/nuc11--02/yaw-capture-v2-spi-GJD2AaFl`.
See `YAW_BURST_WORKFLOW.md` for formats and caveats.

Final AC6 rebuild: zero errors, 69 warnings. Driver fault injection and 13
Windows protocol tests passed; NUC 11 passed with 2 Windows fixtures skipped.
AXF SHA256: `DE874990D693BD5FE6B5727A0E9F53EF303AEF44C13D747EDE18A10260611365`.
Only the user can perform the next flash/Reset/Run; keep UP and stationary,
recheck live IMU before proceeding to buffered capture or powered experiments.

## 2026-09-08: byte-wise revision passes static live and burst checks

User confirmed the read-compatibility firmware running, UP and stationary.
First captured five seconds with the v1 interface; after checking its validity
and safety flags, captured one v2 burst with the same UP-only conditions.
Both sessions used monitor heartbeats only and closed the serial port normally.
No flash, reset, parameter write or motor command was sent by the agent.

Remote evidence under `/home/nuc11--02/yaw-capture-v2-spi-GJD2AaFl/captures/`:
`static_live_01` and `static_burst_01`. Both copied to
`NoMachineTemp/yaw-spi-byte-static-20260908-01/` with matching subdirectories.

| Observation | Live | Buffered |
| --- | --- | --- |
| Complete / invalid records | 200 / 0 | 512 / 0 |
| CRC / missing / incomplete / discarded bytes | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 |
| MCU first..last tick | 64781..69756 | 102153..103175 |
| Adjacent complete-record intervals | all 25 ms | all 2 ms |
| First-to-last span | 4.975 s | 1.022 s |
| INS sequence first..last | 31819..34307 | 50505..51016 |
| INS publication rate over span | about 500 Hz | 500 Hz |
| IMU age | 1..2 ms | 2 ms |
| Flags | 207 throughout | 207 throughout |
| Both yaw software commands | zero | zero |
| Receiver yaw channel | zero | zero |
| MCU skipped slots | zero | zero |
| Read failures / INS read rejections / dt rejections | 0 / 0 / 0 | 0 / 0 / 0 |
| Accel ID / gyro ID | 0x1E / 0x0F | 0x1E / 0x0F |
| First / last yaw (deg) | -85.863594 / -85.522865 | -83.300262 / -83.232574 |
| Net heading change (deg) | +0.340729 | +0.067688 |

The v2 download finished about 13.412 seconds after collector start; those
host times are not the 1.022-second acquisition timeline. Diagnostic frames
received during the download describe live counters after acquisition and
must not be aligned as if they were simultaneous with each buffered sample.

Sampled task intervals during the burst:

| Task | Min / mean / max (ms) |
| --- | --- |
| Input/control | 1.985667 / 2.000000 / 2.015208 |
| INS | 1.971530 / 1.999961 / 2.030066 |
| Gimbal | 0.939357 / 0.952055 / 0.968220 |

These fields are the latest task intervals sampled at 2 ms. In particular,
not every gimbal iteration is recorded: do not infer its full jitter
distribution or exact loop rate from this subsampled set. The snapshot sees
one new INS publication per burst record with a consistent 2 ms age. This
does not mean hardware conversions are synchronized or that all control
iterations receive a new IMU observation.

Static read continuity and the high-rate acquisition/download path are now
demonstrated for this run. This supports the byte-wise compatibility change,
but does not retrospectively identify which original burst transaction or
timing guard caused the prior failure. Persistent heading drift remains:
about +0.06849 deg/s over the live interval and +0.06623 deg/s over the short
burst. No drift correction, PID retune or claim of dynamic stability follows.

Next: prepare a synchronized small-yaw-only enabled-loop diagnostic with
explicit arming/motion authorization, before fitting a model. A one-second
burst cannot be reliably aligned with human chat prompts; plan a suitable
event trigger or scheduled procedure first. Keep the right switch UP until
that procedure is ready. Big-yaw inhibition remains in place.

## 2026-09-08: host release trigger prepared, no powered test yet

Added `yaw_release_trigger.py`, `yaw_release_capture_cli.py` and focused tests.
See `YAW_RELEASE_WORKFLOW.md`. A centered SAFE baseline and a real enabled
stick excursion must precede the zero-stick release event. Invalid data,
gaps, isolation mismatch and SAFE transitions cannot trigger a burst.
The host switches existing monitor heartbeats from v1 to v2, logs the
40 Hz history plus 512 buffered samples, and reports MCU alignment delay.
No firmware, PID, soft limit, flashing or motor-command changes were made.

Deployed separately to `/home/nuc11--02/yaw-release-zXa3AHXz` on the NUC.
Windows: 24 tests, 23 passed, one Linux-only skip. NUC: 24 tests, 22 passed,
two Windows C-fixture skips. NUC pseudo-terminal integration verified the
actual CLI handoff/download and monitor-only transmitted packets. No real
serial device was opened in this preparation turn.

Important: the buffered phase has no live safety telemetry. A host reminder
is not an emergency stop, and ending a monitor session does not disable RC
control. Operator readiness must be confirmed before a powered trial; until
then keep UP. Next is one brief small-yaw release diagnostic, not a coupled
MPC excitation or an automatic motor test. Previous static pose was joint
+17.3 degrees, not centered; confirm clearance before the trial.

## 2026-09-08: first host-triggered powered small-yaw release captured

User confirmed readiness. Started monitor-only capture, verified centered UP
baseline, then instructed ONE brief operator-controlled DOWN/manual excursion
toward available center-side travel, release and return UP. No actuator,
parameter, reset or flashing commands were sent. The tool detected enabled
center, stick excursion and release. Its serial session ended successfully.
This was not an autonomous excitation and no second trial was started.

Remote: `/home/nuc11--02/yaw-release-zXa3AHXz/captures/release_01`.
Local: `NoMachineTemp/yaw-release-20260908-01/` containing raw RX, live/burst
CSV and report. Release MCU tick 1211887; first buffered tick 1211912, a
25 ms delay from the first observed zero-channel sample. This is not the
unknown delay from the exact physical stick-release instant.

- Live: 1176 valid records, 25 ms spacing, zero CRC errors/missing groups.
  Fifteen initial discarded bytes were reported, not lost complete records.
- Operator enabled interval observed before release: 420 records, 10.475 s.
  Nonzero channel: 48 records spanning 1.175 s, channel range 0..67 overall.
  This was longer than a minimal impulse; do not label it an ideal step.
- Burst: 512 valid records, every 2 ms, 1.022 s span; zero CRC errors, missing,
  incomplete, discarded or invalid records, no skipped sample slots.
- All buffered rows: channel zero, flags 222, big output zero and isolation
  present. Small loop remained enabled throughout the buffered interval.
- IMU publications advance 605385..605896; age 1 ms. Read/INS rejection
  counters remain zero in the auxiliary diagnostics. Later diagnostic
  timestamps must not be aligned with historical burst samples.
- Target stays exactly -18.828403 degrees throughout the burst. No continued
  target integration after release was observed in this window.

| Relative to detected release | IMU yaw deg | Target error deg | Small encoder deg | Small output |
| --- | --- | --- | --- | --- |
| 0 ms (live) | -18.149727 | -0.678677 | 306.079102 | -0.507674 |
| 25 ms | -18.272491 | -0.555912 | 305.947266 | -0.462986 |
| 125 ms | -18.447525 | -0.380878 | 305.815430 | -0.558728 |
| 225 ms | -18.459099 | -0.369305 | 305.727539 | -0.549000 |
| 525 ms | -18.477211 | -0.351192 | 305.727539 | -0.524995 |
| 1047 ms | -18.468163 | -0.360241 | 305.639648 | -0.581447 |

Release-to-last encoder change is -0.439453 degrees. Big encoder only varies
by one raw count (0.043945 degrees). Final 112 burst records report zero
integer motor rpm, but encoder moves by one count; do not claim exactly zero
physical speed. In that final interval, IMU yaw ranges -18.471573..-18.457100
degrees and average software output is -0.564581. No output saturation
appears (burst effort range -0.640588..-0.424152 versus saved limit 6).
The roughly 0.36-degree residual error and continuing small effort are
consistent with imperfect low-speed tracking/friction, not proof of its
cause. Do not infer torque or resolve historical coupled-yaw runaway from
this short, single-direction isolated trial.

After download, a separate 3-second live monitor confirmed current UP:
120 valid rows, flags 207, channel zero, both yaw outputs and integer speeds
zero, all parser error counters zero. Saved remotely as
`release_01_post_safe`, locally `NoMachineTemp/yaw-release-20260908-01-post-safe/`.
This check occurred later, not immediately at the burst end; it cannot
establish the operator's exact UP transition time. Both sessions are closed.

Conclusion: this observed release decelerates and does not reproduce the
reported continuing acceleration. No PID or firmware changes follow. Next
candidate is a separately confirmed opposite-direction release to check
asymmetry, followed by a deliberate identification design if appropriate.
Keep UP meanwhile; this is not yet a fitted model or validated MPC.

## 2026-09-08: user closes troubleshooting, returns to MPC mainline

User reports manual tests fully normal and declines the proposed opposite-
direction release comparison. Do not resume that PID troubleshooting branch
without a new reason/request. Work now proceeds in `mpc/`: a sparse OSQP
prototype with a synthetic coupled six-state plant, asymmetric travel bounds,
big-motion penalties, finite effort, terminal small-rate braking constraint,
explicit rejected-solve results, and offline regression scenarios.

No MCU source, tuning parameter or vehicle deployment was changed in this
stage. There is no live optimizer output path. Synthetic simulation is not
parameter identification. Before vehicle takeover, the big input column and
cross-axis dynamics need dedicated identification; zero-big-command traces
cannot establish them. See `mpc/README.md` for assumptions and next gates.

Offline verification completed: 13 MPC tests passed, covering six closed-loop
scenarios, failure/invalid/stale handling, continuous full-turn coordinates,
input-excitation rejection and big-motion cost. Existing recorder tests also
passed (24 tests: 23 passed, Linux pseudo-terminal skipped on Windows).

The first boundary regression exposed exact-boundary roundoff causing the
next initial state to be rejected. Future prediction nodes now reserve an
additional 0.1 degree inside the measured soft range, without changing the
firmware limits or relaxing the initial-state check. Added soft comfort-zone
cost outside 70% of each asymmetric travel side so relief restores useful
range rather than permanently parking at the edge. Only comfort has slack;
hard travel does not. This is still not robust-invariance/viability proof.

Final synthetic evidence: `NoMachineTemp/yaw-mpc-sim-20260908-02/`, six cases
of 200 steps, all completed, no intermediate simulated soft-limit violation.
Includes both boundaries, initial rate disturbance, +25% inertia, heading
step and unwrapped full-turn crossing. Boundary final small angles +22.72
and -44.12 degrees. Model is explicitly synthetic and unvalidated.

For the synthetic 10-degree step, big-motion cost reduces peak big displacement
19.89 -> 3.14 degrees. Final run peak solve time 7.61 ms; earlier prototype
run `yaw-mpc-sim-20260908-01` had a 48.52 ms outlier. Do not claim 20 ms or
AC6 real-time suitability. Audit of actual `release_01/burst.csv`: input rank
1, big command zero; no big-input dynamics were identified or invented.

Numerical packages were installed only in `NoMachineTemp/yaw-mpc-venv`.
No vehicle SSH/serial, motor command, firmware edit, flash, PID change or
powered experiment was performed in this MPC implementation turn. Next is
the dual-input identification design and then measured-model validation,
shadow operation and separately authorized takeover, not more PID retuning.

## 2026-09-08: current no-camera load accepted; standalone scheduler prepared

User confirms the chassis can be fixed and Pitch is at the usual operating
pose. Camera is not installed; user elects to proceed without it. Record this
as a load-specific condition, NOT proof that the missing camera is negligible.
`mpc/identification_setup.json` stores setup `fixed_chassis_no_camera_v1`,
operator-reported rather than freshly measured, with no hardware execution
authorization. Camera installation requires checking model applicability;
do not block current preparation on an unknown camera mass or mounting offset.

Added independent C99 `module_yaw_identification.h` and
`Tests/test_yaw_identification.c`. It generates reference offsets only, with
UP-only arming, distinct DOWN transition, immutable per-trial request, replay
rejection, heartbeat/feedback/timing/travel checks and latched completion or
abort. Baseline 0.5 s, windowed 1/2/4 Hz multisine 3 s, post-window 0.5 s.
One axis is selected per request; big-joint and small-inertial-heading anchors
are sampled when DOWN starts the trial. Offset zero on the other axis does
not imply zero actual motor drive. Reference return to zero is not proof of
physical settling. No output stop/hold policy is implemented by this helper.

This header is NOT included by firmware application sources or registered in
the Keil project. No runtime motion/start protocol, longer synchronized
capture or output arbitration is implemented yet. Do not claim this is a
ready-to-flash identification mode. Integration must disable competing RC,
vision and old follower reference writers, govern both output layers, freeze
tuning and implement/test finish and abort behavior with the actual plant.
Details and limitations: `mpc/IDENTIFICATION_CORE.zh-CN.md`.

GCC `-std=c99 -Wall -Wextra -Werror` compile and executable tests passed.
AC6 6.22 Cortex-M4 standalone compile with `-ffp-mode=full` and warnings-as-
errors passed; no AC6 linking or board execution occurred. Initial default
AC6 compile rejected NaN/Inf test values; preprocessor reports
`__FINITE_MATH_ONLY__=1` under that default. New header rejects finite-only
compilation. Global firmware compiler flags remain unchanged; before runtime
integration, select appropriate floating-point settings and verify timing.
Do not infer that this discovered build concern caused the past vehicle fault.

Possible capture planning: 250 Hz over 4 s requires 1001 points; a yet-to-be-
designed 48-byte record would use 48048 bytes, compared with current 49152-byte
burst record storage. This is only a memory calculation: quantization, exact
fields, metadata, union exclusivity and linker placement remain unimplemented.
Original 500 Hz/512-point capture is unchanged.

No SSH, real serial, motor command, parameter write, reset or flash occurred.
Existing AXF SHA256 remains
`DE874990D693BD5FE6B5727A0E9F53EF303AEF44C13D747EDE18A10260611365`.
Next: integrate explicit identification protocol/capture/ownership, test the
actual control call paths, then schedule a separately confirmed powered trial.

## 2026-09-08: identification integration built, no powered execution

Integrated `app_yaw_identification.c` into the gimbal target. Request/ACK/status
and frozen metadata/sample download use dedicated USART1 commands 0x36..0x3B.
PROBE reports build and preflight flags, checked by the host before ARM.
Default host action is PROBE only. ARM needs fresh UP, neutral controls, valid
observations, zero yaw commands and a new ID; physical DOWN starts one bounded
4-second reference trial. Normal big-yaw compile inhibition remains enabled.
No saved PID gains or soft limits were changed. Small yaw remains IMU-closed.

Identification owns both references and Pitch hold, blocks legacy PC writes,
commands chassis STOP and zeros the shooter/feeder at their output gates.
The router suspends task scheduling through ownership check/reference writes,
without masking hardware IRQs, to prevent preempted old-reference publication.
The anti-jam feeder path is also blocked during ownership. Terminal state
disables yaw/Pitch and remains locked until explicit RELEASE in fresh UP.
Pitch can sag on termination: prepare noninterfering fall protection.

The new 48-byte, 1001-sample buffer shares a union with the previous burst
storage. Terminal captures freeze actual samples, including partial aborts;
they are never padded. The 192-byte metadata snapshots the trial and 32 key
settings. Samples contain latest sensor snapshots and software voltage-command
integers, not simultaneous hardware sampling, measured torque, current or CAN
acceptance. See `mpc/IDENTIFICATION_CORE.zh-CN.md` for exact limitations.

Tests: production supervisor with mocked hardware passed both axes, wrap,
abort/freeze/release and failure injection; C wire layout replayed by Python.
15 new host/static-contract tests passed. Combined host suite: 39 run, 38 passed, 1 existing
Linux-only test skipped on Windows. MPC: 13 passed. Seven existing/core C
executables (scheduler, burst, capture, limits, coordinator, follow, UART mux)
also compiled with warnings-as-errors and passed.

AC6 6.22 full rebuild: 0 errors, 69 existing warnings. Code 134408, RO 4156,
RW 332, ZI 108728 bytes. Added global `-ffp-mode=full` for valid NaN/Inf checks;
board timing and safety behavior require validation, not inferred from linking.
Build log: `NoMachineTemp/ident-integration-build.log`.
AXF SHA256: `CA930EFC35DE471031D588DAFE73D64EE8EC17D48B3CB8EFE966AA29EA8C08B2`.
Previous known AXF backed up as `NoMachineTemp/pre-ident-integration-DE874990.axf`.

No SSH, live serial, NUC deployment, reset, flash, parameter write or real
excitation occurred. Next: operator flashes gimbal only and confirms UP/static;
verify firmware/observations first, then request distinct per-trial permission.
Load remains `fixed_chassis_no_camera_v1`; camera installation needs model
revalidation. No physical model has yet been fitted.

## 2026-09-09: operator flashed; first identification PROBE, no ARM

Operator reports firmware flashed and running. SSH reached
`nuc11--02@192.168.1.113` (host `nuc1102-NUC11PAHi5`). CP2102 stable device:
`/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`.
The ownership check reported no occupying process; no processes/services were stopped.

Deployed isolated tools to `/home/nuc11--02/yaw-ident-v1-RRPZqyUo`.
Remote Python 3.10.12 protocol/mock-serial tests: 12 run, 11 passed, one local
C-fixture test skipped. These tests did not open the real serial device.
Then ran exactly `--action probe`, with no ARM/KEEPALIVE/CANCEL/RELEASE,
PID write, motor command, flash or reset by the agent.

Live status: ID 0, count 0, phase IDLE, reason 0, flags 248. INFO build
0x59490101, protocol 1, record size 48 matched. Yaw software commands zero;
yaw/IMU feedback fresh, neutral inputs, centered/slow joints, configuration
valid. `remote_up_fresh=false`: do not arm; ask operator to switch the
receiver/transmitter on and put the right switch UP, then repeat PROBE.
No inference of mechanical stopping, dynamic IMU accuracy or real-time sample
period from this single status snapshot. No identification data were collected.

Remote evidence: `captures/probe_01` under the new tool directory.
Local evidence: `NoMachineTemp/yaw-ident-probe-20260909-01` (raw/events/report/setup).
Serial was closed by the query tool on exit. Await fresh UP/static confirmation.

## 2026-09-09: second PROBE confirms UP, operator-neutral gate fails

Operator confirmed UP and stationary. Ran only PROBE again in the same isolated
remote tool directory, output `captures/probe_02`. Matching INFO; ID/count 0,
IDLE, reason 0, flags 220. Remote UP/fresh now true; yaw commands zero,
yaw/IMU feedback fresh, joints centered/slow and configuration valid remain true.
`operator_neutral=false`. The current firmware checks all five analog channels
within [-10,10] plus mouse x/y and left/right buttons; the status packet does
not expose which input failed. Do not assume a particular channel or relax
the threshold. Asked operator to release both sticks and check any auxiliary
wheel/knob center while keeping the right switch UP.

Local evidence: `NoMachineTemp/yaw-ident-probe-20260909-02`.
No ARM, motor command, PID change, reset or flash. Query closed its serial FD.

## 2026-09-09: third PROBE still neutral-false; per-input diagnostics built

After the operator confirmed centered controls, `captures/probe_03` again
reported IDLE, flags 220, fresh UP, zero yaw commands and valid feedback,
but operator-neutral false. Local copy: `NoMachineTemp/yaw-ident-probe-20260909-03`.
Do not reinterpret the earlier verified ch2 return-to-exact-zero result as
a failed yaw stick; the combined five-channel/mouse gate cannot isolate it.

Built additive 0x3C remote diagnostics with a frozen 24-byte snapshot across
three sequence-tagged fragments. All five decoded channels, mouse x/y and
buttons, tick, receiver age and a nine-bit failure mask are visible. The same
mask implements the original neutral boolean, preserving exact [-10,10]
channel thresholds and zero mouse requirements. No PID or motor logic changed.
PROBE only, no snapshots during owned trials; ARM clears pending diagnostic
transmission. New build tag 0x59490102. New CLI `--remote-detail` is probe-only;
old build read-only probes/replays remain supported, trial requires current build.

GCC production-supervisor tests passed including every input bit, boundaries,
frozen snapshot under backpressure, wire fields and no ownership/enable by PROBE.
20 local identification tests passed; combined old-capture suite 44 run,
43 passed and one Linux-only skip. NUC mock/protocol suite 16 run, 14 passed,
two local C-fixture skips; no real serial was opened by those tests.

AC6 rebuild: 0 errors, 69 existing warnings. Code 134824, RO 4156, RW 340,
ZI 108740. Log `NoMachineTemp/ident-remote-diag-20260909.log`.
New AXF SHA256 `FF33D09F622E64C8D7E15D67E91070DF1F48E537741ADDF52DB436942FFA08C7`.
Prior AXF preserved as `NoMachineTemp/pre-remote-diag-CA930EFC.axf`.
New tools deployed separately to `/home/nuc11--02/yaw-ident-remote-zoAnGkOF`.
The new firmware has NOT been flashed by the agent. No ARM, motor command,
PID write or reset. Await operator flash/run/UP, then use only remote-detail
PROBE to identify the specific blocking input before changing any guard.

## 2026-09-09: remote-detail identifies only auxiliary ch4

Operator confirms diagnostic firmware running, right switch UP. Executed only
`--action probe --remote-detail` in `yaw-ident-remote-zoAnGkOF`, output
`captures/remote_01`. INFO tag 0x59490102 matches. IDLE, ID/count zero, flags 220.
Snapshot sequence 1 at MCU tick 53652 ms, receiver age 12 ms:
ch0=ch1=ch2=ch3=0, ch4=-660; mouse x/y and buttons all zero.
failed_mask=16, failed_inputs=[ch4]. UP fresh, zero yaw commands and feedback
checks pass. No ARM or motion, PID write, flash or reset by the agent.

Source audit: app_control.c uses ch4 only for fire request (>100), conditional
on manual mode and the other switch's shooter enable. It does not command yaw,
Pitch or chassis motion. Current -660 does not request fire. The identification
gate nonetheless requires all five channels within [-10,10]. This explains
the neutral-false flag without contradicting the previously verified ch2
return-to-zero. Physical wheel rest behavior is not yet established; do not
claim a receiver fault or silently relax the gate from one snapshot.

Local evidence: `NoMachineTemp/yaw-ident-remote-20260909-01`.
Serial closed normally. Keep UP; clarify whether the auxiliary wheel returns
to center or retains an endpoint before changing its identification condition.

## 2026-09-09: auxiliary wheel no-fire gate, build 0x59490103

Operator confirms the wheel can center but its spring is obstructed and it is
normally unused. The earlier -660 observation is not a yaw joystick failure.
Changed only identification intent gating: ch0..3 retain [-10,10]; ch4 must
not request fire (>100); mouse x/y/buttons must remain zero. Receiver range
validation is unchanged. Normal control and identification share the fire
threshold in module_remote_intent.h; normal fire behavior is unchanged.
Shooter inhibit, UP/timeout/feedback/limit safeguards and existing PID remain.
Mechanical spring trouble is not repaired by this software change.

Remote diagnostic fragments now explicitly carry version 2, with bit4 meaning
wheel fire request. Decoder retains version 1's all-five-neutral semantics
for old recordings and rejects mixed-version fragments. Sample/meta/status
layouts remain version 1. Trial requires the current firmware build.

Production C supervisor tests pass, including -660 arming/start, +100 allowed,
+101 abort with outputs zero and ARM rejection, other joystick and mouse
rejection. Python combined suite: 46 tests, 45 passed, one platform skip.
NUC offline protocol/mock-serial suite: 17 tests, 15 passed, two C-fixture skips.
Mock ARM messages are simulated; no real serial was opened this turn.

AC6 rebuild: 0 errors, 69 existing warnings. Code 134856, RO 4156, RW 340,
ZI 108740. Log: NoMachineTemp/ident-wheel-policy-20260909.log.
AXF SHA256: 29BB7B2DE0D4CFA571789D09378EC757429EB943416D1DE93CDFDABA3796E436.
Previous FF33D09F build: NoMachineTemp/pre-wheel-policy-20260909-130200.axf.
Tools deployed separately to /home/nuc11--02/yaw-ident-wheel-R6NQqStf.
New firmware not flashed by the agent; no reset, live PID write or motion.
Await operator flash/Reset/Run/UP, then read-only build and gate verification.

## 2026-09-09: wheel-policy firmware verified, all preflight flags pass

Operator reports wheel-check build running. Read-only PROBE with remote-detail
confirms build 0x59490103. IDLE, trial ID/count zero, flags 252: fresh remote UP,
zero yaw software commands, fresh yaw/IMU feedback, operator-neutral, centered
slow joints and valid configuration. Ownership and yaw permission are false.
Remote snapshot: tick 29714 ms, sequence 1, version 2, receiver age 13 ms;
ch0..3=0, ch4=-660, mouse x/y/buttons zero, failed_mask=0, failed_inputs=[].
This resolves the auxiliary wheel gate; no need to repeat the yaw-stick test.

Remote evidence: /home/nuc11--02/yaw-ident-wheel-R6NQqStf/captures/remote_01.
Local evidence: NoMachineTemp/yaw-ident-wheel-20260909-01.
Probe exited normally and closed serial. No ARM, PID write, reset or motion.
Snapshot validity is not a dynamic safety proof. Proposed next trial: big-axis
reference amplitude 1 degree, 4 seconds total; both yaw loops participate.
Await specific trial consent and fixed chassis/clearance/Pitch support confirmation.

## 2026-09-09: first authorized trial expired before motion

Operator explicitly confirmed the first trial. Executed exactly one trial
command: big axis, amplitude 1 degree, 4 seconds, output captures/big_01 in
/home/nuc11--02/yaw-ident-wheel-R6NQqStf. Fresh preflight flags 252 passed;
ARM ID 1 accepted. Tool returned buffered output after a 10-second execution
wait, consuming much of the 15-second physical switch window. The assistant
relayed DOWN after seeing acceptance; subsequent status showed expiry.
Do not repeat this chat-dependent timing workflow for the next attempt.

Terminal state ABORTED, reason 6 (YAW_IDENT_ARM_EXPIRED), count 0, start_ms 0.
Events show ARMED through host_s 15.009 and ABORTED at 15.109, remote UP fresh.
No baseline/excitation samples or dynamic identification results were obtained.
Download completed with zero CRC errors; the empty capture is explicitly
marked trial_not_completed and sample_count_not_1001, not valid model data.
Final flags 253: owned, yaw disabled, UP fresh, yaw commands zero. No RELEASE
or automatic retry was sent. Serial process exited and closed its descriptor.

Local copy: NoMachineTemp/yaw-ident-big-20260909-01. Existing PID unchanged.
Keep UP and support Pitch. Before retry, obtain confirmation for terminal
release/re-arming and arrange an operator-visible NUC terminal for immediate
ARM/DOWN prompts rather than forwarding time-critical prompts through chat.

## 2026-09-09: two authorized retries, middle-switch startup defect fixed

Operator said the first DOWN was forgotten, then authorized retry. RELEASE 1
accepted after UP/zero-output/terminal checks, then ARM 2 (big, 1 degree, 4 s).
Used hidden SSH with redirected live output to avoid the prior 10-second wait.
ID 2 aborted OPERATOR_STOP before baseline, count/start_ms 0. Final neutral
flag false. Operator then reported moving a stick to check remote operation
because Pitch was unpowered and yaw motion was not visible, and authorized
another attempt. Pitch is deliberately disabled in ARMED/terminal states.

RELEASE 2 accepted in UP; ARM 3 preflight flags 252. ID 3 remained ARMED through
host_s 8.605 (flags 253), then aborted reason 1 at 8.705 (flags 249), neutral
still true, count/start_ms 0. Operator returned UP; download completed.
Final flags 253, ownership retained, yaw disabled; no RELEASE 3 sent. Both
captures have zero CRC errors but zero samples and are invalid for fitting.
SSH PID 5008 exited; no active collection process remains.
Remote: yaw-ident-wheel-R6NQqStf/captures/big_02 and big_03.
Local: NoMachineTemp/big_02 and NoMachineTemp/big_03.

Source defect: YawIdent_Step rejected both UP=false/DOWN=false even in ARMED,
so a valid three-position switch passing through MIDDLE aborted before DOWN.
Reproduced with the actual application supervisor in the C fixture (assertion
at switch_transition), then fixed. Added explicit connected MIDDLE observation;
unknown/disconnected switch is still invalid. ARMED can wait in UP or MIDDLE
with zero outputs, within the original 15-second deadline and heartbeat.
Only DOWN starts; running MIDDLE/UP still stops immediately. No PID, travel,
input-neutral/fire, capture layout or normal-control behavior changes.
Telemetry supports this explanation but does not capture every receiver frame.

Build 0x59490104; host permits old archive replay but trial needs current build.
Core and production-app C tests pass, including transition, active-middle stop,
invalid receiver/switch, stick/wheel, output fault, cancel, and middle timeout.
Python: 46 tests, 45 pass, one platform skip. NUC offline protocol/mock suite:
17 tests, 15 pass, two local fixture skips. These do not operate real serial.
AC6: 0 errors, 69 existing warnings. Code 134888, RO 4156, RW 340, ZI 108740.
Log NoMachineTemp/ident-switch-transition-20260909.log.
AXF SHA256 90BFC59E230E016C98D7CEAB090D02D12F80A827C93B124622C8AABD41D16C87.
Backup NoMachineTemp/pre-switch-transition-20260909-132012.axf.
New tools: /home/nuc11--02/yaw-ident-switch-s7aSyRTP.
No flash/reset by agent; await user flash/run/UP, then read-only preflight.

## 2026-09-09: switch-transition build running, read-only preflight passes

Operator reports transition build running. Only PROBE with remote-detail was
executed in yaw-ident-switch-s7aSyRTP, output captures/remote_01. INFO confirms
0x59490104. IDLE, id/count 0, flags 252: unowned, yaw disabled, fresh remote UP,
zero yaw commands, fresh yaw/IMU, neutral inputs, centered/slow joints, valid
configuration. MCU restart cleared the previous session; do not release old ID 3.
Snapshot tick 32392 ms, seq 1, remote wire v2, age 11 ms; ch0..3 zero, ch4=-660,
mouse x/y/buttons zero, failed_mask=0. Local evidence:
NoMachineTemp/yaw-ident-switch-20260909-01. Probe exited and closed serial.
No ARM, motor reference, parameter write or release this turn. These are
snapshots, not validation of the repaired physical switch transition. Await
specific motion confirmation for the same big-axis 1-degree/4-second trial.

## 2026-09-09: baseline entered, dynamic Pitch commands trip config guard

Operator explicitly confirmed collection. Executed one big-axis 1-degree/4-s
trial on build 0x59490104 in yaw-ident-switch-s7aSyRTP/captures/big_01.
Preflight flags 252; ARM 1 accepted. Trial entered BASELINE at tick 172216,
then ABORTED CONFIG_CHANGED (8) at tick 172217. Exactly two samples downloaded,
CRC errors zero, no excitation; short 1-ms terminal interval is retained and
flagged, not padded. First small command -52 raw, next command zero; big zero.
Final UP fresh, flags 253, yaw disabled and commands zero; ownership retained.
No RELEASE or auto-retry. SSH PID 26284 exited normally. Local evidence:
NoMachineTemp/yaw-ident-switch-big-20260909-01.

ARM metadata captured Pitch ctrl.kp_set/kd_set=0. Source audit: DM disable
clears these command fields; RobotActuator_DmEnable restores them from
s_pitch_context. The identification comparator incorrectly used command fields
instead of persistent settings. Added a lifecycle mock to the production-app
fixture; unmodified supervisor reproducibly aborted on the first enabled cycle.
This is a confirmed software defect consistent with the live timing, but no
per-field live change record exists, so exact live field attribution is unproven.

Fixed by adding read-only RobotActuators_GetPitchMitGains and using its stored
settings in the 32-field config snapshot/comparator. No gain value changed.
New build 0x59490105 binds new trials; older archives remain readable. Metadata
pitch_mit_kp/kd now represent persistent settings, unlike old disabled-command
zeros. No sample/meta layout changes. Actual setting changes are still guarded.

Production-app test passes complete 1001-sample lifecycle and rejects Kp/Kd
changes during ARMED and BASELINE. Python 47 tests: 46 pass, 1 platform skip.
NUC offline protocol/mock tests 17: 15 pass, 2 fixture skips. These mocks do
not validate physical DM/CAN timing. AC6 0 errors, 69 existing warnings;
Code 134944, RO 4156, RW 340, ZI 108740.
Log NoMachineTemp/ident-pitch-config-20260909.log.
AXF SHA256 039EA5980E00F1F2162D5348E93ABAA19A887FE272422698EF0C8F3DFF95AABB.
Backup NoMachineTemp/pre-pitch-config-20260909-133220.axf.
New host tools /home/nuc11--02/yaw-ident-config-4grH57N3, no serial opened in tests.
Await user gimbal-only flash/Reset/Run/UP and read-only preflight. No firmware
download, reset, PID write or further motion by agent. MPC remains inactive.

## 2026-09-09: persistent-config build verified with read-only PROBE

Operator reports configuration-check build running. Only probe/remote-detail
executed in yaw-ident-config-4grH57N3, captures/remote_01. Build 0x59490105,
IDLE, id/count 0, flags 252: unowned, yaw disabled, fresh UP, zero yaw commands,
fresh yaw/IMU, neutral inputs, centered/slow joints, valid configuration.
Snapshot tick 35403, seq 1, diagnostic v2, receiver age 9 ms; ch0..3=0,
ch4=-660, mouse x/y/buttons zero, failed_mask=0. Serial closed normally.
Local evidence NoMachineTemp/yaw-ident-config-20260909-01.
Restart cleared previous ownership; do not release the old boot's ID 1.
No ARM, PID write or actuation this turn. Static preflight does not establish
that the real enabled Pitch lifecycle now completes; await trial consent.

## 2026-09-09: config-build trial expires while awaiting DOWN

Operator explicitly confirmed collection. Executed exactly one big-axis
1-degree/4-second trial on 0x59490105, ID 1, under
/home/nuc11--02/yaw-ident-config-4grH57N3/captures/big_01.
Preflight flags 252; ARM accepted. State remained ARMED with flags 253 through
host_s 15.0035, then ARM_EXPIRED (reason 6) at 15.1035. Start_ms/count 0.
No baseline/excitation occurred. Download completed, CRC errors 0, record count
0; marked trial_not_completed and sample_count_not_1001, not valid model data.
Final state owned/disabled/UP fresh/zero yaw output; no RELEASE or retry sent.
SSH PID 25096 exited, serial closed. Local evidence:
NoMachineTemp/yaw-ident-config-big-20260909-01.

The command tool returned the initial ARM text, which was relayed in chat,
but chat/tool timing does not guarantee that the operator receives a usable
15-second switch window. Do not repeat chat-only countdown coordination.
Before any further ARM, arrange an operator-visible NUC terminal for direct
prompts and fresh consent; release this boot's terminated ID 1 only under UP
and zero-output checks. Firmware/PID unchanged; no new flash needed. The
persistent-Pitch-config fix still awaits an actual enabled-cycle trial.

## 2026-09-09: operator terminal capture, growing big-yaw oscillation

User reports capture finished. Only remote directory listing and file copy
performed; no serial query/write, release, ARM or PID change by agent.
Latest capture is yaw-ident-config-4grH57N3/captures/big_20260909_134608,
trial ID 2, build 0x59490105. Local archive:
NoMachineTemp/yaw-ident-terminal-big-20260909-134608.
Offline raw replay into the sibling -replay directory agrees with report.

Downloaded 150 records: 125 BASELINE, 24 EXCITE, one ABORTED. start_ms 481792,
first sample 481793, terminal 482387 (595 ms after start). CONTROL_FAILED (9),
no configuration-change or switch-transition abort. CRC errors 0; 148 sample
intervals 4 ms, final early-stop interval 2 ms. Preserved quality flags:
irregular_sample_interval, sample_count_not_1001, trial_not_completed.
Final report flags 253, UP fresh, yaw disabled/commands zero, ownership retained.

Baseline big command and rpm are identically zero. At excitation onset the
quantized reference stays 0..1 centidegree, while big rpm alternates with growing
amplitude: -2,+2,-8,+8,-13,+13,-22,+22,-46. Big software command spans -4034 to
28247 raw, versus nominal full-scale 30000. Big encoder 8104..8129; small encoder
6806..6808, small rpm -1..0 and commands -73..45. Yaw spans about 0.033 degrees.
Feedback age maxima: IMU 4 ms, big CAN 1 ms, small CAN 2 ms.

Terminal -46 rpm = -276 deg/s exceeds the unchanged 180-deg/s trial guard.
YawIdentApp_AllowYaw calls InsideTravel, so that snapshot alone is sufficient
to trigger the output validator's CONTROL_FAILED path. Reason 9 aggregates
multiple predicates and no per-predicate mask/Pitch diagnostic is captured;
do not claim that all simultaneous causes are excluded. Data strongly suggests
unstable big-yaw response in the identification HOLD loop, not an adequate
excitation/model dataset. No precise oscillation frequency or unique mechanical,
filter/PID cause inferred from these sparse samples.

Keep UP and support Pitch. Do not increase safety thresholds or automatically
repeat trials. Next investigate the big-yaw HOLD-loop dynamics before further
identification. Small yaw/Pitch tuning unchanged. No new build/flash needed
from this read-only analysis; MPC model remains unidentified and inactive.

## 2026-09-09: persist operator-selected big-yaw Kp 10

User supplied browser screenshot and requested firmware defaults plus terminal
identification commands. Screenshot: angle PID 10/.001/4, speed PID .6/0/0,
effort limit 30, speed filter tau .03 s. Changed only BIG_YAW_ANGLE_KP 50 -> 10
in module_big_yaw_tune.h; all seven other selected settings already matched.
sys_const and GimbalYaw initialization use this macro-backed parameter table.
Added static regression for all eight selected defaults and Kp init path.
No online PID write, UART operation, flash/reset or trial by agent.
Small yaw, Pitch, coordinator, output isolation and identification safeguards
unchanged. Operator expects stability; it has NOT been demonstrated here.

AC6 complete rebuild: 0 errors, 69 existing warnings; Code 134944, RO 4156,
RW 340, ZI 108740. Python combined suite 48: 47 pass, one platform skip.
Log NoMachineTemp/ident-big-kp10-20260909.log.
New AXF SHA256 5FA00AF1DECFBD0BB54672C39C7B3444ED720C910A32023746663E47BDAEB33C.
Prior artifact NoMachineTemp/pre-big-kp10-20260909-140923.axf.
Protocol/layout unchanged; tag stays 0x59490105. Distinguish artifacts by hash
and actual RAM readback/configuration metadata, not the tag alone.
Existing NUC tools yaw-ident-config-4grH57N3 remain compatible. Chinese guide
contains the post-flash/Reset/Run single big-axis 1-degree/4-second command.
Await user flash and terminal test; no automatic second-axis trial or RELEASE.

## 2026-09-09: Kp10 trial confirms settings, still trips rate guard

Operator supplied terminal report. Retrieved only completed files from
yaw-ident-config-4grH57N3/captures/big_kp10_20260909_141355.
Local archive NoMachineTemp/yaw-ident-big-kp10-20260909-141355;
raw replay into sibling -replay directory reproduces 401 records and flags.
No live serial access, release, ARM, parameter write or firmware change.

ID 1, build 0x59490105, configuration confirms angle 10/.001/4, speed .6/0/0,
effort 30, tau .03; other-axis parameters match prior capture. Started 57847,
terminal 59446 (1599 ms), phase ABORTED reason TRAVEL (4). 126 baseline,
274 excite and one terminal samples. 399 intervals of 4 ms and final 3-ms
early termination interval; CRC errors 0. No padding or model fitting.

Big rpm -26..36, final +36 = 216 deg/s > 180-deg/s guard. Last big delta
from anchor +1.274414 degrees, small delta 0, IMU yaw delta +.033195,
roll delta +.009365, pitch delta +.024333 degrees. Recorded small encoder
6820..6821, far from software travel boundaries. Evidence supports rate
guard, not hitting an angle/mechanical stop. InsideTravel includes both
rate and position checks; reason 4 is not exclusively an angle-limit code.
Prior reason 9 can arise when the output validator sees the same violation
before the core state machine; the difference alone does not prove a new fault.

Big command -2095..3733 raw, reference -.46..+.58 degrees. Tail rpm alternates
-13,-4,+18,+8,-26,-9,+36. Lower sampled effort and longer duration than the
Kp50 capture are encouraging observations, not proof of stability or a unique
PID cause. Feedback age maxima IMU 4 ms, big 1 ms, small 2 ms. Small rpm -1..0.
Hold UP, keep Pitch supported; final report owned, yaw disabled/commands zero.
Do not repeat the same trial or raise guards. Investigate loop/feedback/filter
dynamics before another independently approved test; PID and MPC unchanged.

## 2026-09-09 RC left-switch yaw selection (firmware only)

User requested manual big-yaw operation without changing small-yaw control.
Right UP remains SAFE; right DOWN + left DOWN selects the new big-yaw
encoder-position reference integrator. Left UP/MIDDLE retain the original
small-yaw heading input. AUTO does not select manual big yaw. The small IMU
loop, soft limits, Pitch and vision handling remain intact; only its RC yaw
increment is removed while big yaw is selected. Thus small yaw can counteract
big-yaw movement and reach its existing limits. This is not passive-small
plant identification. The old left-switch RC shooting association is removed;
explicit PC shooting behavior is unchanged.

Manual entry captures current encoder position and clears big PID history;
stick center holds reference. Actual tick delta integrates the rate; the
new big-only center deadband is 2 deg/s (10 RC counts). Reference wraps at
360 degrees without accumulating turns. Command/receiver age is limited to
50 ms, with current right/left switch and tune-owner checks at calculation
and final output. Invalid feedback or output states revoke the manual gate.
Measured speed above 30 rpm latches a manual fault, cleared only by fresh
right UP. The normal coordinator stays inhibited; identification owns both
axes ahead of this manual path. No guards relaxed and no PID changes.

Verification: manual helper tests cover entry, signed/time-scaled increments,
center hold/deadband, expiration, off/on generation, multi-turn wrap, both
overspeed signs and latch, invalid floats/delta, and tick overflow. Existing
small-yaw rate, coordinator, full tuning, soft limit and identification core
C suites pass, as does the production identification supervisor fixture.
Python integration/protocol suite: 49 tests, 48 passed and 1 skipped.
Source-contract checks are not an RTOS or physical-motion simulation.

AC6 rebuild: 0 errors, 69 existing warnings; Code 137016, RO 4156, RW 336,
ZI 108792. Log: NoMachineTemp/manual-yaw-selector-20260909.log.
AXF SHA256: BC2404AA921E413EB057E693DDB980ED0FFED8CDF295C42FEC51D3F96D598590.
Protocol format/build tag remain 0x59490105; no host script redeployment needed.
Existing AXF backed up before compilation. No flashing, reset, live parameter
write, serial acquisition, ARM or RELEASE was performed. Hardware validation
is pending; current PID must not be described as proven stable.

## 2026-09-09 Remove manual big-yaw speed trip at user's request

Removed the 30-rpm trip from both BigYawManual_Step and final yaw output.
Finite measured speed alone no longer faults or revokes manual big control.
Right-switch SAFE, receiver/command freshness, tuning and identification
ownership, invalid-number/timing faults and small-yaw limits remain intact.
Identification's own speed/travel guards are unchanged. PID gains, output
budget, RC rate scaling and small-yaw/Pitch control are unchanged.
Manual emergency-stop response cannot guarantee prevention of fast oscillation.

Updated manual C tests pass: +/-30.1 and +/-300 rpm accepted, permission
loss at high speed stops control, invalid sensor values still latch a fault.
Python suite: 49 tests, 48 passed, 1 skipped. AC6: 0 errors, 69 warnings;
Code 136928, RO 4156, RW 336, ZI 108792.
Log: NoMachineTemp/manual-yaw-no-speed-trip-20260909.log.
AXF SHA256: F5B05C00F1D62C47240981DB6AB31CD01BB5C584D6995872F2F705624667F94C.
Previous AXF backed up. No flashing, live commands, ARM, RELEASE or reset.

## 2026-09-09 Screenshot PID 1/0/5, .5/.0001/0; big rate guard 360 dps

User requested persistence of the pictured big-yaw gains and a wider
identification envelope. Defaults are now angle Kp/Ki/Kd 1/0/5, speed
Kp/Ki/Kd .5/.0001/0, effort 30, speed-filter tau .03 s.
Only identification big-yaw rate guard increased: 180 -> 360 deg/s (60 rpm).
Small-yaw rate remains 180 deg/s, travel remains 12 degrees from anchors,
small soft-limit margin 5 degrees, ARM stationary gate 6 deg/s, all operator,
heartbeat, freshness, configuration and timing checks unchanged.
Normal manual big-yaw speed trip stays removed; no controller/axis logic change.
This does not establish stability or qualify previous aborted captures.

Build tag bumped to 0x59490106; wire schema unchanged. Decoder keeps historical
0x59490105 compatibility, CLI requires matching new build before trial ARM.
New remote directory: /home/nuc11--02/yaw-ident-pid1-KhRIhXIg.
Uploaded CLI, identification/capture protocols, serial helper, offline tests
and unchanged setup metadata; no existing scripts/captures overwritten.

Tests: core state machine accepts +/-360 and rejects +/-361 deg/s;
production supervisor accepts +/-60 and rejects +/-61 big rpm at output gate.
Small +/-30/31 rpm boundary unchanged. Production PID test had stale Kp50
expectations; updated to screenshot values and expected first-step 1.5003.
PID suite passes with alg_math.c's two existing parentheses warnings retained.
Core and production supervisor fixtures pass; local Python 51 (50 pass, 1 skip).
NUC offline ProtocolTests/CliTests 18 (16 pass, 2 skip: local C fixtures absent).
An initial NUC invocation named a nonexistent LifecycleTests class; corrected
to CliTests and rerun successfully. All serial activity in tests was mocked.

AC6 rebuild: 0 errors, 69 existing warnings; Code 136928, RO 4156, RW 336,
ZI 108792. Log: NoMachineTemp/ident-pid1-rate360-20260909.log.
AXF SHA256: 27ADDA8FCA55B8BC3845C98067DF0217052260E78DF6F6C090B8E0632649259D.
Pre-build AXF backed up. No flashing, reset, live serial I/O, RAM tune write,
ARM or RELEASE performed. User to flash and manually initiate one 1-degree
big-yaw trial, then return UP and preserve the capture before any repeat.

## 2026-09-09 Repair missing deployed mpc setup directory

User's CLI failed at main line 199 reading mpc/identification_setup.json.
Deployment had flattened this file into the script directory. The failure
occurred after output-directory creation but before run(), Serial or ARM.
Created /home/nuc11--02/yaw-ident-pid1-KhRIhXIg/mpc and uploaded the unchanged
setup JSON there. Existing top-level file and capture directories preserved.
No firmware/CLI behavior changes, serial access, ARM, RELEASE or reset.

Added DeploymentTests exercising main() with the actual deployed setup path,
temporary output, mocked run() and synthetic complete capture. Verifies
setup.json and report.json creation without actual serial access.
Previous tests exercised run() and missed main's config dependency.
Local suite: 52 tests, 51 passed, 1 skipped. NUC deployment/protocol/CLI:
19 tests, 17 passed, 2 skipped for absent local C fixtures. Safe to retry the
documented timestamped command after the user's usual physical preparation;
no extra flash or ownership release is needed solely for this startup error.

## 2026-09-09 First complete big-yaw capture, PID1 ID 1

Copied /home/nuc11--02/yaw-ident-pid1-KhRIhXIg/captures/big_pid1_20260909_154215
to NoMachineTemp/yaw-ident-pid1-20260909-154215. Prior 153750 directory left intact.
Raw replay into sibling -replay directory agrees: 1001 samples, DONE/reason 0,
CRC 0, quality_issues empty. Start tick 340375, duration 4000 ms; 1000 uniform
4-ms intervals, 125 baseline/750 excite/125 settle/1 terminal samples.
Metadata confirms build 0x59490106 and angle 1/0/5, speed .5/.0001/0,
effort30/tau.03. Final recorded flags253: owned, UP fresh, yaw commands zero.
No live status query, parameter write, RELEASE or ARM performed this turn.

Actual big encoder displacement +/-0.1318359375 deg (6 counts peak-to-peak),
target offset +/-0.58 deg, excited tracking-error RMS .233520 deg.
Big rpm -1..0, small rpm 0 throughout. Big command -.236..+.267 software units,
small -.377..+.106; neither sampled output approaches its bound.
Small joint -8.5693359375..-8.3935546875 deg, start -8.4375, end -8.525390625.
IMU heading relative to start 0..+.18826293945 deg, terminal +.1792755127.
Maximum roll/pitch delta .046976/.054878 deg; maximum IMU/big/small ages3/0/2ms.
Excitation centered two-input singular values3263.177/1889.051 raw, but rank
alone does not establish persistent excitation or identifiable plant dynamics.
Low motion (6 encoder counts) and near-zero quantized rates preclude reliable
inertia/damping/lag fitting from this trial alone. Preserve as low-amplitude
baseline, not as proof of broad stability or a completed identification.

Suggested next trial: unchanged PID/load/Pitch/guards, amplitude parameter3
instead of1 on big axis, with independently executed UP-only release of
current ID1 before ARM. Commands documented; not automatically executed.
First analysis interpreter lacked NumPy; reran numerical analysis successfully
in the existing NoMachineTemp/yaw-mpc-venv environment. No fitting or MPC takeover.

## 2026-09-09 Big amplitude3 ID2 complete, response still small

Latest NUC directory captures/big_pid1_amp3_20260909_160121 copied to
NoMachineTemp/yaw-ident-pid1-amp3-20260909-160121, raw replay in sibling
-replay directory agrees. ID2, amplitude3, build0x59490106, selected PID
1/0/5 and .5/.0001/0 confirmed unchanged. DONE/reason0, 1001 records,
1000 intervals4ms; phases125 baseline/750 excite/125 settle/1 done.
Flags29 for1000 active records, flag5 terminal. CRC0, quality_issues empty.
Final recorded status flags253: owned ID2, remote UP fresh, yaw commands zero.
No live serial access, release, arm, parameter edit or firmware changes.

Big displacement from353.49609375 anchor: -.615234375..+.3076171875deg,
span .9228515625deg (21counts), terminal-.087890625deg. Target +/-1.74deg,
excite tracking-error RMS .648909deg. Big rpm -2..2, only56/1001 nonzero;
small rpm -1..1, only5/1001 nonzero. Big software effort -.500..+.806 of30,
small -.514..+.186 of6. No sampled saturation. Small joint
-8.61328125..-8.4814453125deg (3counts span), terminal-8.5693359375deg.
IMU heading relative to trial start -.014503..+.249619deg, final+.067085deg.
Max roll/pitch excursion .087456/.050405deg; max ages IMU3/big0/small2ms.

Motion increased vs amplitude1 but quantized velocities and weak second-axis
response remain insufficient to claim reliable coupled inertia/lag estimates.
Initial IMU headings differ between trials; compare each trial relative to
its own anchor, do not splice raw headings as a continuous stationary record.
No broad stability claim, fit or MPC takeover. Proposed next is big amplitude5,
within existing cap, same PID/setup/guards, after user-executed release ID2.
If still weak, revisit waveform bandwidth/friction/measurement resolution
instead of repeatedly expanding protection bounds. Guide now separates the
current commands from historical commands to prevent copying amplitude1 again.

## 2026-09-09 Big amplitude5 ID3 complete; independent small input next

NUC captures/big_pid1_amp5_20260909_161356 copied into
NoMachineTemp/yaw-ident-pid1-amp5-20260909-161356. Raw replay into sibling
-replay agrees with report. ID3, amplitude5, build0x59490106, unchanged PID.
DONE/reason0, 1001 samples, 1000 intervals4ms, phases125/750/125/1.
Active flags29 (1000), terminal5. CRC0, quality_issues empty.
Final recorded flags253: owned ID3, remote UP fresh, yaw commands zero.
No new serial observation, RELEASE, ARM, parameter write or firmware edit.

Actual big position relative354.111328125 anchor:
-1.3623046875..+.703125deg, span2.0654296875deg (47 counts), terminal-.2197265625.
Waveform target +/-2.91deg for amplitude parameter5; excite error RMS1.072351deg.
Big rpm +/-3, 150/1001 nonzero. Big effort -.413..+1.300 of30, no sampled cap.
Small joint8.26171875..8.8330078125deg (13 counts); rpm -1..0, only1 nonzero
sample; effort-.463..+.679 of6. Heading change-.342499..+.249634deg, end-.260742.
Settle big encoder remains-.2197265625deg despite+.212..+.223 effort;
consistent with a low-speed friction/deadband hypothesis, not unique proof.
Max roll/pitch delta .061533/.075602deg; feedback ages IMU3/big1/small2ms.

Pose differs from amplitude3: initial small joint -8.525390625 -> +8.8330078125,
initial IMU roll -2.89840698 -> -6.87186289deg. Cannot treat the amplitude series
as same-pose experiments or concatenate absolute headings. Preserve per-trial
anchors/configuration. No broad stability claim, coupled model fit or MPC takeover.
Next recommended trial is small-heading amplitude3 at current fixed pose/load
and unchanged gains/guards; big joint remains closed-loop at its anchor.
Guide gives manual release ID3 then separate small trial command, never executed
by agent. If independent data remains weak, revisit waveform bandwidth/duration
and low-speed nonlinear effects instead of endlessly raising amplitudes.

## 2026-09-09 Small amplitude3 ID4 complete; first paired offline audit

Copied NUC captures/small_pid1_amp3_20260909_162615 to
NoMachineTemp/yaw-ident-small-amp3-20260909-162615. Independent raw replay
in sibling -replay directory agrees. ID4/axis2/amplitude3/build0x59490106,
all gains and limits unchanged. 1001 samples, 1000 intervals4ms, phases
125 baseline/750 excite/125 settle/1 terminal; flags29 active,5 terminal.
DONE/reason0, CRC0, format quality_issues empty. Final recorded flags253:
owned ID4, fresh UP, yaw commands zero. No live serial query or actuation.

Small joint7.3388671875..9.580078125deg, span2.2412109375 (51counts).
Heading relative anchor -1.415634..+1.125778deg; target +/-1.74deg.
Excite heading-error RMS .555056deg, settle RMS .014243deg;
terminal heading error+.010361deg. Big fixed-target displacement
-.6591796875..+.0439453125deg (16counts), terminal-.439453125deg.
Big rpm-2..0 (43 nonzero samples), small rpm-4..3 (154 nonzero).
Effort big-.032..+.723/30, small-.670..+1.246/6, no sampled saturation.
Big settle offset constant-.439453125 despite+.307..+.328 effort.
Max age IMU3/big1/small2ms; roll/pitch max delta .045811/.087704deg.

ID3/ID4 paired: all control configuration equal; initial small joint differs
.395508deg, IMU roll .011716deg, pitch .071118deg. Treat as nearby-pose
candidate records, not proof of identical base/load. Use separate anchors.
Stack each trial's 750 EXCITE two-input rows after per-trial demeaning:
rank2, singular values14.25713/11.15062, ratio1.278595, correlation.026921.
Reference matrix rank2 as well. This is not dynamic persistent-excitation,
closed-loop unbiased estimation or held-out validation. No fitted model or
MPC takeover yet. Pause further field trials; next work is offline measurement
mapping, local model candidates and predictive validation, retaining quantization
and low-speed friction caveats. Dedicated Chinese pair review created and
current guide changed to stop recommending repeated collection/release.
No firmware changes, PID writes, RELEASE, ARM, reset or flash this turn.

## 2026-09-09: first offline MIMO ARX fit, predictive screening failed

Added mpc/identify_offline.py and test_identify_offline.py. Pure local file
analysis only; no serial imports/operations or hardware permission changes.
Replayed ID3 big5 and ID4 small3 raw captures and retained trial boundaries.
Per trial: first 2s train, next 1s select, last 1s held out; drop output-off
terminal sample. Inputs are recorded software voltage commands /1000,
outputs are separately unwrapped big/small joint and IMU heading deltas.

27 regularized ARX structures screened using tuning 200ms rollout only.
Selected output order10, input order5, delay1 sample, ridge1e-6;
autonomous spectral radius .9812992055. Neither mechanical system order
nor actuator delay nor physical stability is established by these values.
No future measured output enters recursive prediction; recorded future
drive inputs are used. Closed-loop bias is not resolved. No test-based
reselection or refit; overlap and limited quiet test coverage documented.

Formal output: NoMachineTemp/yaw-ident-offline-pair-20260909-02.
Report contains raw hashes, full configuration, candidates and baselines;
candidate.json and per-trial final-second trajectory CSVs saved locally.
200ms heading RMSE ID3 .500307deg vs hold .025243deg;
ID4 .130248deg vs hold .044246deg. Full-second heading RMSE
1.175465/.287044deg. Candidate fails held-out rollout screening.
Geometry regression is diagnostic only, not an IMU mounting calibration.
No independent repeated-trial validation, low-speed friction/quantization
unresolved; ARX not compatible with the existing synthetic six-state MPC.
Artifacts remain experimental_unvalidated, hardware_takeover_allowed=false.

22 tests passed: new offline regression/data-leakage/raw-replay tests plus
existing MPC synthetic tests. Chinese result report and current guide updated.
Next recommendation: design slower bidirectional excitation and dwell segments,
preserve PID/pose/load, collect independent repeats; no new waveform deployed.
No remote connection, RELEASE, ARM, serial write, firmware edit, reset or flash.
ID4 ownership/UP/zero yaw outputs refer to last recorded state, not live query.

## 2026-09-09: stronger slow excitation, offline review draft only

User supports substantially stronger slow excitation and will review.
Created slow_excitation_review.json, preview_slow_excitation.py and tests,
plus SLOW_EXCITATION_REVIEW.zh-CN.md. No firmware/live CLI edits.
Proposed actual reference peaks big +/-15deg, small heading +/-10deg,
separate axes, 20s: 2s baseline, 2s ramp, 3s positive dwell, 4s reversal,
3s negative dwell, 2s return, 4s settle. Quintic C2 references;
max reference rates14.0625/9.375dps are not actuator speed limits.
Unchanged PID and effort limits; actual motion cannot be predicted yet.

Proposed small start within +/-2deg of calibrated center; previous +8.4375deg
start fails the proposed full relative-travel corridor check. Original soft
limits/5deg inward guard retained. Draft identification travel aborts big25,
small20, heading20deg require runtime review, not a dynamic safety proof.
All current estop/feedback/heartbeat/config checks retained in proposal.

4ms recording needs5001*48=240048bytes, existing capture48048bytes.
Existing 6*16byte wire format at250Hz requires24000bytes/s, exceeding
115200 8N1. Proposed460800 streaming has52.1% sample-wire utilization,
excluding overhead. Requires zero-output bench validation, bounded DMA
queue, ring overflow shutdown, new protocol/build/profile metadata and
host/decoder support; no baud or firmware settings actually changed.
Offline artifacts: NoMachineTemp/yaw-slow-excitation-review-20260909-01.
30 tests passed including8 new reference-review tests and existing22 tests.
No serial/SSH, ARM, RELEASE, download, reset, flash or physical movement.
Awaiting user review; current historical capture commands do not run this plan.

## 2026-09-09: user accepted profile; firmware-first implementation

User explicitly requested firmware changes after accepting the slow profile.
Gimbal build0x59490201 adds separate slow ARM/reverse and zero-output BENCH,
leaving legacy short excitation and normal PID/remote control behavior intact.
20s quintic profile, max big15/small10deg, 4ms samples,5001 records.
New-profile center2deg and relative travel25/20/20deg; original soft limits,
5deg inward guard, motor-rate trips, estop/freshness/heartbeat/config checks kept.

1001-record ring reuses existing capture union; cumulative host ACK controls
slot reclamation, reserved terminal slot, 300ms ACK-progress timeout.
Six original16-byte frames per record copied into96-byte SRAM DMA buffer.
USART1 and CubeMX source setting now460800 8N1; USART3/6 and chassis unchanged.
New STREAM0x3d and PROFILE0x3e, profile/stream metadata v2; initial/terminal
metadata use different assembly sequences. Wrong/future ACKs do not advance.
BENCH holds remote UP, all yaw/Pitch outputs disabled, never publishes motor
references. Only complete regular5001-point acknowledged bench grants slow ARM
eligibility for10minutes. Ownership remains latched after every terminal state.
No operator approval was treated as a measured link benchmark.

Native GCC -Wall -Wextra -Werror core, slow-core/wire, production supervisor,
production PC DMA transport tests passed. Covers full20s/ring/tick wrap, zero
outputs, bench gate/expiry, stalls, overflow, CRC, DMA busy/failure and IRQ state.
30 Python offline reference/identification/MPC tests passed unchanged.
Keil AC6 6.22 full rebuild:0 errors69 existing warnings; Code140272 RO4156
RW336 ZI108976. DMA buffer at0x20000538, capture union49164 bytes unchanged.
AXF SHA256:685755BE108A2C3F79F05DA6032A41E2B62BF21FE687E7F482FDD89F26DB8437.
Log NoMachineTemp/yaw-slow-stream-rebuild.log; firmware guide YAW_SLOW_STREAM.md.
Host streaming tool and physical throughput/stop validation still pending.
No SSH, serial, ARM, RELEASE, reset, flash or hardware movement this turn.

## 2026-09-09: flashed firmware reported; host streaming tools deployed

Implemented yaw_slow_protocol.py / yaw_slow_cli.py for build0x59490201,
460800baud. Legacy Serial default115200 preserved, optional460800 added;
old captures/decoder retain original builds and reject the new firmware.
New status parses version byte separately from qualification/profile byte.
Separate initial/terminal metadata assemblies, exact profile/config checks,
contiguous CRC-validated records, flush raw file before cumulative receipts,
20ms receipts/100ms heartbeat, no automatic release or following trial.
Probe only by default; BENCH requires BENCH_REMOTE_UP and cannot emit motion ARM.
Disconnect/invalid stream while responsible sends CANCEL where possible;
firmware's independent leases remain the fallback. Release queries current
owned terminal id and needs REMOTE_UP; already-unowned release sends no command.

39 new+legacy Python tests passed locally,11 new tests on NUC Python3.10.
Extended native supervisor test to export a real production-generated BENCH
frame stream. Local and NUC offline replay:5001 records,crc0,quality_issues[].
These are software-only fixtures, not a physical bench pass.
No firmware changes or reflash in this turn; native fixture change is test-only.

Deployment directory:/home/nuc11--02/yaw-slow-59490201-0iRzrp.
Includes mpc/identification_setup.json; old pid1 directory/captures untouched.
Real read-only probe executed via NUC with exclusive serial ownership and then
closed. checks/live_probe_01/report.json: build1497956865 (0x59490201),
id0,count0,phase0,reason0,flags252,axis0,version1,extra0,error null.
At query time: unowned,fresh UP,zero yaw outputs,neutral,centered,valid feedback.
No real BENCH, ARM, RELEASE, motor-target write, reset or flash was sent.
Updated guide's first command to new-directory zero-output BENCH; awaiting
operator execution/result. Physical throughput and motion remain unverified.
