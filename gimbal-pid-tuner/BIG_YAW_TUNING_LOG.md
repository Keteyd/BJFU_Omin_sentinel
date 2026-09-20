# Big-yaw commissioning log

## 2026-09-06: speed Kp 0.60 to 0.70

The user reported no high-frequency oscillation with the rollback defaults,
then confirmed upper SAFE for a single speed-loop Kp change.

- Before: angle Kp 2.6, speed Kp 0.6, effort limit 4.0, filter tau 0.030 s.
- After: angle Kp 2.6, speed Kp 0.7, effort limit 4.0, filter tau 0.030 s.
- Write request ID 33544, firmware status 1 (accepted). A separate read-only
  invocation after closing and reopening the port confirmed all four values.
- Both readbacks reported remote SAFE, outputs off, big inactive and feedback
  online. No axis enable, reference step or motion command was sent.
- This is a RAM-only change. Firmware boot defaults remain 2.6/0.6/4.0/0.030.
- Physical response with Kp 0.70 has not yet been tested or reported.

The first read attempt returned EBUSY. Unprivileged fuser found no owner;
administrator inspection required a password, so the cause was not proven.
The CLI lacked explicit exclusive-mode cleanup. It now issues TIOCNXCL before
closing its owned descriptor, including failure paths, and still closes when
cleanup fails. See the [Linux ioctl documentation](https://man7.org/linux/man-pages/man2/TIOCEXCL.2const.html).
All 10 offline tests passed, including real PTY reopening and mocked cleanup
failure/ownership cases. The fix was deployed to the vehicle PC. Subsequent
query, write and independent query all opened and closed the physical port
successfully. No MCU firmware change or flash was needed for this fix.

## 2026-09-06: angle Kp 2.60 to 3.20

The user subsequently reported that big yaw was still too soft and confirmed
the proposed single angle-loop adjustment. No instrumented vibration or
disturbance trace was captured in this turn.

- Before: angle Kp 2.6, speed Kp 0.7, effort limit 4.0, filter tau 0.030 s.
- After: angle Kp 3.2, speed Kp 0.7, effort limit 4.0, filter tau 0.030 s.
- Before writing, readback confirmed fresh motor feedback, receiver SAFE,
  yaw outputs off and big yaw inactive.
- Write request ID 55195, status 1 (accepted). An independent read-only query
  confirmed the complete parameter set and unchanged SAFE/output-off state.
- No motion, axis-enable or step command was sent. The settings are RAM-only;
  boot defaults remain 2.6/0.6/4.0/0.030. Response at angle Kp 3.2 awaits
  physical testing; no stability or stiffness improvement is claimed yet.

## 2026-09-06: angle Kp 3.20 to 4.00

The user reported some improvement at 3.2 but still insufficient stiffness.
The proposed 4.0 setting had not yet been applied: the initial query in this
turn confirmed 3.2/0.7/4.0/0.030, receiver SAFE, yaw outputs off and feedback
online. The angle gain alone was then updated using the previously proposed
single-parameter trial.

- After: angle Kp 4.0, speed Kp 0.7, effort limit 4.0, filter tau 0.030 s.
- Request ID 58792, status 1 (accepted). A separate read-only invocation
  confirmed all four values and the unchanged SAFE/output-off state.
- No enable, motion or reference-step command was issued. This is RAM-only;
  firmware boot defaults remain 2.6/0.6/4.0/0.030.
- Physical response with angle Kp 4.0 remains untested. No active disturbance
  trace was recorded; saturation=false in SAFE does not establish output
  headroom during vehicle motion. If still soft, capture active error and
  effort before attributing the remaining displacement to insufficient gains.

## 2026-09-06: passive observation after another softness report

- Readback confirmed 4.0/0.7/4.0/0.030, active big yaw, feedback online and
  receiver not in SAFE. No parameter write was attempted.
- Captured 10 seconds using nonlocking observation; capture UTC end was
  2026-09-06T08:59:41.969086+00:00. File:
  `NoMachineTemp/big-yaw-kp4-soft-58792-observe-01.json` in the workspace,
  with the same basename under the vehicle PC's gimbal-pid-tuner directory.
- 200 encoder samples over 9.951 seconds; big joint span 0.04395 degrees,
  net displacement zero. All 100 unique coordinator frames reported active
  HOLD; reported position error, speed reference, speed feedback and effort
  were zero throughout those sampled frames.
- This did not capture an appreciable relative-joint displacement. Whether
  the user disturbed the vehicle during this interval is unknown. It does
  not establish loaded output headroom, torque insufficiency or satisfactory
  disturbance rejection. The roughly 10 Hz coordinator telemetry cannot
  rule out higher-frequency activity between samples.
- No motor command, enable command, firmware edit or parameter change was
  made. Clarify joint-relative displacement versus world-heading changes,
  then capture the reported event before selecting the next adjustment.

## 2026-09-06: reported disturbance and SAFE two-pose feedback check

The user confirmed visible big-yaw motion relative to the chassis during the
requested 30-second observation. The record
`NoMachineTemp/big-yaw-kp4-disturbance-58792-01.json` contains 600 encoder
samples and 300 distinct active HOLD frames. Big raw counts were only 7911
and 7912; maximum reported error was 0.04 deg, speed reference 0.17 rpm,
filtered speed 0.02 rpm and effort 0.123. Maximum sampled feedback age was
1 ms. This discrepancy remains unresolved; telemetry sampling does not rule
out intersample motion and does not justify increasing effort on its own.

The user then selected upper SAFE. Readback confirmed SAFE, outputs off,
big inactive and motor feedback online before each stationary pose capture.
The user manually rotated the big-yaw platform and held the new pose.

- Pose A: big count 7903, small count 6133; 60 samples, accepted.
- Pose B: big count 727, small count 6149; 60 samples, accepted.
- Shortest wrapped big delta: +1016 counts = +44.6484375 encoder degrees.
- Small delta: +16 counts = +0.703125 encoder degrees.
- Both captures have big stationary span 0.04395 deg, small span zero.
- Files: `NoMachineTemp/big-yaw-mapping-pose-a-58792-01.json` and
  `NoMachineTemp/big-yaw-mapping-pose-b-58792-01.json`, also on the vehicle PC
  under `/home/nuc11--02/gimbal-pid-tuner/` with the same basenames.

This supports that the configured big feedback responds to platform rotation
and is not a permanently frozen encoder value. The operator's actual platform
rotation angle is still needed before inferring encoder-to-joint ratio.
This is not a complete verification of wiring, mechanical coupling or dynamic
response. No gains, motor commands, encoder zero, soft limits or firmware were
changed. Running parameters remain 4.0/0.7/4.0/0.030; stay in SAFE pending
the next check.

## 2026-09-06: static mapping confirmed; task-rate peak diagnostics

The user confirmed actual manual rotation approximately matched the measured
44.6484375 degrees. This supports the static encoder-to-platform mapping and
an approximately 1:1 scale, but does not resolve the dynamic wobble discrepancy
or rule out compliance/slip under load. No further PID increase was made.

Added gimbal-only 0x2F peak windows sampled after each yaw control iteration:
unwrapped angle span, absolute error/raw-speed/command-effort peaks, saturation,
enabled sample count and elapsed time. Invalid/stale feedback, task gaps and
overflow are flagged. SAFE movement is excluded from enabled angle spans.
The monitor CLI now supports --require-peaks, retains window records, checks
coverage and rejects old firmware lacking this packet. See README for units
and acquisition limits; this is task-rate monitoring, not an oscilloscope.

- Pure C diagnostic tests passed (wrap, out-and-back, send boundary, inactive
  transitions, timestamp rollover, stale/NaN/bounds/overflow).
- Python: 11 observation tests and 10 big-yaw tuning tests passed offline on
  the vehicle PC. No physical serial port was opened by these tests.
- AC6 rebuild: 0 errors, 85 warnings; Code 102540, RO 4092, RW 336, ZI 58704.
- Build log: NoMachineTemp/yaw_big_peaks_rebuild.log.
- AXF SHA256: B37157F8B2025856416E63B301B79642C7DB70A0C9571D816E79787B0096EEBB.
- No firmware download/reset, serial command, gain write or motion was issued.
  Firmware end-to-end telemetry remains untested until manual flash.
- Defaults remain 2.6/0.6/4.0/0.030. The prior RAM trial 4.0/0.7/4.0/0.030
  does not survive reset; query SAFE readback before the next disturbance run.

## 2026-09-06: independent big-yaw browser console

The user deferred the diagnostic firmware flash and requested a big-yaw web
console on the existing interface firmware. Added `big-yaw.html` with four
readback-backed parameter fields, a SAFE-only review/confirmation workflow,
request-ID/ACK/value verification, independent telemetry plots and CSV export.
There are no step, reset, enable, small-yaw, Pitch or chassis commands.

- 40 in-browser mock-serial assertions passed in headless Chrome at 1440x1000.
  The prior 39-assertion suite also passed at 390x900; the additional assertion
  checks a fixed wire packet against the existing Python codec.
- 11 offline Python tests passed (wire fixture and existing CLI tests).
- Desktop/mobile screenshots inspected; simulated telemetry trace pixels are
  nonblank and the browser tests found no horizontal page overflow.
- Deployed page/assets to `/home/nuc11--02/gimbal-pid-tuner/`, preserving the
  original small-yaw `index.html`. Added trusted desktop shortcut
  `Big Yaw PID Tuner` and opened the page in NoMachine DISPLAY=:1001.
- The page starts disconnected. No physical serial device was opened, PID
  parameter written, firmware flashed or motor commanded in this task.
  Hardware browser readback/write verification awaits the user's connection.
- The deployed interface does not require the newer 0x2F firmware. Its 0x2C
  plots remain approximately 10 Hz and cannot rule out high-frequency motion.

## 2026-09-06: browser DEVICE_LOST / CLI terminal-mode compatibility

The user reported the page's read loop failed with "The device has been lost".
CP2102 was present as ttyUSB1 (stable by-id link), permission group dialout was
present in both SSH and the Chrome process, and no owner was visible to fuser.
Opening/closing the device succeeded. A prior read-only CLI query confirmed
4.0/0.7/4.0/0.030, request 58792, SAFE, outputs off, big inactive and online.
Kernel logs showed the earlier 16:42 disconnect/re-enumeration, not a new USB
disconnect contemporaneous with the browser failure.

The current terminal had VMIN=0 / VTIME=0, matching both Python CLI setup
functions. Linux PTY tests reproduced an empty read on a still-connected
terminal. Chromium POSIX code maps this to DEVICE_LOST and preserves VMIN.
Changed both workspace setup functions to VMIN=1 / VTIME=0; O_NONBLOCK still
prevents idle blocking. 34 offline tests passed including three real PTY cases.

Deployed the compatible helpers. The remote small-yaw CLI was an older version;
its prior rate-cap/default behavior was retained, with only VMIN changed
(staging copy: NoMachineTemp/pid_tune_cli_browser_compat.py).
Corrected the live terminal's VMIN from 0 to 1 while the port was unowned and
exclusively held, preserving baud and all other attributes. No data bytes were
written during this repair; exclusivity and the descriptor were released.
No PID or firmware changes. Browser reconnection is pending user verification;
the PTY reproduction supports this cause but does not itself prove the real
browser connection is now successful.

## 2026-09-06: connection recovered; clearer write-button gating

The user confirmed the browser connection recovered after the VMIN fix, then
reported a persistently disabled write button and unavailable I/D fields.
Code inspection found that unchanged encoded values and invalid drafts disabled
the button without explaining those conditions in the original status message.
The actual live condition in the user's window was not independently observed.

Unified write-button gating and its displayed reason, including empty/range
errors, below-resolution edits, pending reads, SAFE, output-off and freshness.
47 simulated-serial browser assertions passed. Deployed only big-yaw.html and
big-yaw.js without refreshing the user's active session or opening the UART.
No parameter write or firmware change was made.

Confirmed BigYaw_ApplyTune in module_big_yaw_tune.h sets both loops' Ki/Kd and
integral limits to zero. The current four-field 0x2D/0x2E interface cannot tune
I/D; exposing functional controls requires a firmware/protocol extension and
manual flash, which the user previously deferred. No nonfunctional I/D inputs
were added to the existing-firmware page.

## 2026-09-06: user requests full big-yaw effort range

The user reported a large stiffness improvement from raising only the effort
limit, without changing PID gains. The exact newly selected live limit was
not queried because the browser owns the serial session. This is evidence
that output saturation deserves attention, not a measured torque or stability
validation. No online gain or effort write was issued by the agent.

Raised the gimbal firmware's big-yaw configurable effort maximum from 8 to 30,
matching the GM6020 voltage-command full scale of 30000 with effort*1000.
Defaults remain angle Kp 2.6, speed Kp 0.6, effort 4 and filter 0.030 s; both
I/D terms remain zero. Small yaw, Pitch, receiver SAFE and freshness guards
are unchanged. Firmware 0x2E flag bit 5 advertises the extended range, so the
updated browser/CLI refuse above-8 settings on the old running firmware.

Real-PID C boundary tests passed, including +/-30 conversion to +/-30000 and
rejection above 30. AC6: 0 errors, 85 warnings. Browser: 56 mock-serial checks;
Python: 15 offline tests passed. Build log and artifact hash are recorded in
YAW_COORDINATION.md. Firmware was compiled, not flashed; its on-hardware
capability bit and extended output are not yet tested. Opening the updated
page does not apply a higher output. The maximum is a protocol boundary, not
a safe continuous operating recommendation.

## 2026-09-06: complete PID and removal of arbitrary gain ceilings

The user requested all P/I/D terms with no parameter upper limits. Implemented
all six big-yaw gains in the real controller, firmware protocol, browser and
CLI, using four staged float32 packets per atomic write and coherent readback.
Gain/tau limits are now numeric representation only; non-finite/negative
values remain invalid. Effort still cannot exceed the motor protocol's 30.
Older firmware retains four-field compatibility and disables unsupported I/D.

Conditional anti-windup, big-yaw-only full state resets, unsafe/expired/partial
transaction rejection and non-finite arithmetic checks are covered offline.
AC6 build: 0 errors, 85 warnings. C PID/staging tests, 68 browser assertions
and 17 Python tests passed. Build identity is in YAW_COORDINATION.md.
No physical UART session, tuning write, motion command or flash was performed.
The current live settings are unknown; boot defaults are unchanged, I/D=0.
The new I/D behavior still needs on-hardware verification after manual flash.

## 2026-09-06: full-PID timeout repair and immediate recentering

User screenshot: SAFE, online, outputs off, configuration age 185 ms, angle
Kp draft 3 vs readback 2.6, with confirmation timeout. Static investigation
found the first full-PID interface exposed an existing RX boundary bug: four
frames fill the entire 64-byte DMA bank, which was serviced only at IDLE.
The automatically switched current bank could look empty; partial frames
were also discarded. No physical UART was opened to reproduce the live write.

Replaced the USART1 RX path with circular HT/TC/IDLE cursor draining plus a
persistent CRC-validated frame mux. Browser and CLI now pace parameter frames
by 20 ms without retries. The exact-64 loss and corrected receipt of four
atomic PID parts are reproduced in `Tests/test_uart_mux.c`.

At the user's request, big yaw now immediately follows the small-yaw joint
offset from visual-center count 6816 in both enabled operating modes. The
former edge thresholds, dwell, trajectory and lead clipping are removed.
Small IMU targets, limits, PID defaults, Pitch and chassis are unchanged.
This supersedes the previous preference for mostly stationary big yaw.

AC6: 0 errors / 85 warnings. UART and coordinator C tests passed, plus existing
PID/full-staging/soft-limit binaries; 69 browser assertions and 18 Python tests
passed. Artifact hash is recorded at the top of YAW_COORDINATION.md. No flash,
online parameter write or motion was performed. Manual gimbal flash, reset/run
in SAFE, a verified readback/write and physical following test remain pending.

Deployment note: the updated browser/CLI passed tests in the NUC temporary
directory, but SSH to 192.168.1.113 timed out during production deployment
and on retry. Production page update is not confirmed and remains pending.
The new firmware preserves the existing 0x30/0x31 wire format and accepts
unpaced/coalesced frames, so the current full-PID page remains compatible.

## 2026-09-07: recurrent browser read failure

After the vehicle PC reboot, the CP2102 stable by-id link points to ttyUSB0
(previously ttyUSB1). SSH works. Opening the device for read-only termios
inspection returns EBUSY. Nonprivileged fuser shows no accessible owner;
sudo requires a password, so the exact exclusive owner is not confirmed.
No serial settings or bytes have been changed during this investigation.

Found an independent cleanup defect: a rejected reader cancellation or writer
abort skipped port.close(). Cleanup now isolates each operation and attempts
port closure even after a stream failure. Transport and protocol errors get
distinct labels with error name/details in the event log. 70 mock-browser
assertions pass, including injected NetworkError followed by failed abort.
This does not prove the original live read error's cause; the port must be
released and the complete message/termios inspected before claiming recovery.

After the user closed Chrome, no Chrome processes remained but opening ttyUSB0
still returned EBUSY. ModemManager logs did not identify a current owner. On
requested physical USB reconnect, the CP2102 appeared as ttyUSB1 and opened
normally; min=1/time=0. Deployed and hash-verified the browser cleanup and the
previously pending write pacing fix. A read-only nonlocking monitor query then
returned full PID: angle P 2.6, speed P 0.6, effort 4, tau .030, I/D all zero,
request/status 0, SAFE/offline-output/inactive, motor online, effort max 30.
No parameter write or motion command was sent. Actual UART readback is now
verified; reconnection in the updated browser still requires user confirmation.

## 2026-09-07: reboot recurrence traced to autoaim.service

The user's privileged fuser query identifies root PID 684, MiracleVision, as
the ttyUSB0 owner. Read-only SSH inspection confirms /proc/684/cgroup belongs
to autoaim.service, enabled and running since boot. ExecStart is
/home/nuc11--02/Desktop/MiracleVision-ppm-dev/start.sh, which launches
sudo -E ./bin/MiracleVision from its build directory. This is the confirmed
cause of the current reboot-time port conflict; browser cleanup changes did
not remove this startup owner. Replugging had only restored temporary access.

For tuning, stop autoaim.service and disable its boot activation; restore it
only after disconnecting the browser when vision is needed. Do not disable
ModemManager speculatively or kill a transient PID instead of managing the
identified service. SSH sudo -n stop was refused because a password is needed;
no service state was changed by the agent. User terminal authorization is
required before claiming the port is released or reboot recurrence is fixed.

## 2026-09-07: vision-to-browser handoff leaves VMIN=0

Verified autoaim.service is disabled/inactive after the user's command. The
next error is now NetworkError/device lost, not open/EBUSY. stty readback
shows 115200, VMIN=0, VTIME=0; kernel logs contain no new USB disconnect.
Read-only source inspection finds uart_serial.cpp line 68 in the deployed
MiracleVision-ppm-dev explicitly sets VMIN=0. Stopping it therefore removed
the owner but left the terminal setting behind.

Added prepare_browser_serial.py and wired the desktop shortcut through
open_big_yaw.py. Preflight refuses a busy device, takes exclusive access,
changes only VMIN/VTIME without flushing or data IO, verifies and releases.
21 offline Python tests passed, including preservation of pending input and
other termios fields, busy refusal, lock cleanup and prior PID wire tests.

Applied preflight on the unowned real ttyUSB0: previous VMIN=0, now VMIN=1,
VTIME=0. A subsequent read-only monitor query successfully returned full PID
2.6/.6/4/.030, all I/D zero, request/status 0, SAFE, outputs off, inactive,
motor online. No PID, firmware or movement command changed. Browser-side
reconnection is still to be confirmed by the user, not inferred from CLI IO.

## 2026-09-07: user-tuned PID saved, HOLD/relief restored

User reports PID tuning complete and supplies a browser screenshot with matching
edited/readback values: angle P=50, I=.001, D=4; speed P=.6, I=0, D=0;
effort=30, speed-filter tau=.03. The screenshot is the source for this save;
no competing UART query was made while the user was using the browser.

Updated big-yaw boot constants, constant PID arrays and runtime gain initialization
so all six gains survive a firmware restart. In particular, angle Ki/Kd are
initialized from constants rather than unconditionally zeroed. Runtime Web
Serial writes remain RAM-only; saving a different preset still needs a source
update/reflash. No other axis or communication settings changed.

Restored the prior demand-based coordinator: captured encoder HOLD, asymmetric
entry/exit thresholds, dwell, prediction, emergency entry, smoothed relief and
lead-limited moving target. No continuous recentering, no fixed PID RPM ceiling.
Tests cover small-joint center activity without big motion, fixed HOLD under
disturbance, both relief directions, exit braking, multi-turn wrap and SAFE.
Actual PID tests verify all saved defaults, I/D arithmetic and output boundaries.

AC6: 0 errors, 85 warnings. Build log yaw_saved_pid_hold_rebuild.log and artifact
hash are recorded in YAW_COORDINATION.md. Firmware compiled only. The restored
policy with these new gains has not yet been tested physically; check boundary
entry and stopping after manual gimbal flash, Reset/Run in SAFE.

## 2026-09-07: roll back HOLD/relief after reported oscillation

User reports high-frequency oscillation after the strategy change and requests
the previous logic. Restored the immediate small-joint-centre following used
when the user tuned the PID. All saved gains, output 30 and tau .030 are
unchanged. Removed the demand-based HOLD/relief planner only, with no rollback
of UART fixes, complete PID, other-axis tuning, soft limits or SAFE guards.

The effective feedback and target generation differ between the two policies;
the code supports this as a possible explanation, not a verified diagnosis of
the physical oscillation. No serial session, online write, motion or flash was
performed. AC6: 0 errors/85 warnings. Immediate-follow C tests and the saved-PID,
UART mux and soft-limit regression binaries pass. Artifact identity and manual
SAFE Reset/Run guidance are at the top of YAW_COORDINATION.md. Retest pending.

## 2026-09-07: restore less-moving policy for manual retuning

User confirmed immediate-follow rollback eliminated the oscillation, then
explicitly requested HOLD/relief again to retune big yaw themselves. Restored
the same planner and its regression tests. No startup gain, effort, filter,
other-axis control, communication or online parameter was changed.

Known starting state: angle 50/.001/4, speed .6/0/0, effort 30, tau .030 was
stable under immediate following but oscillated under HOLD/relief. Warned
to remain UP/SAFE after flashing and Reset/Run, and adjust through the browser
before enabling. New RAM parameters are not persisted by this operation.

AC6: 0 errors / 85 warnings. Artifact SHA256 equals the earlier HOLD build
C05F293080C808456C35A1A6A61C9938DFACBE3B807DDD3436957BBCC15E7F5A.
Log: NoMachineTemp/yaw_hold_retune_rebuild.log. Coordinator, saved-PID, UART
mux and soft-limit regression tests pass. No automatic flash or live test.
