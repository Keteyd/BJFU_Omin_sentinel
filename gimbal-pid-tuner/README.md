# Gimbal PID Tuner

Current identification update (2026-09-10): [CAN history/status/current capture](mpc/CAN_TRACE_20260910.zh-CN.md).
Firmware `0x59490301` uses `yaw_can_trace_cli.py` for version 3 streaming.
Compilation and offline tests pass. The user flashed this build and reported a passing zero-output BENCH;
raw files were retrieved over SSH/SCP and passed independent replay. Duplicate rpm CSV columns were fixed locally
and re-exported from raw data; update the NUC host scripts before the next capture. Firmware is unchanged.
Older capture tools remain for their matching historical firmware/data. Follow the linked current status before the historical steps below.

## Dual-yaw MPC preparation

Stage 1 observation capture is available in `yaw_capture_cli.py` with a matching
gimbal firmware build. It records coherent 40 Hz snapshots, sensor ages, IMU
update counter, both motor states/commands and receiver/target data to CSV,
raw binary and a quality report. See [YAW_MPC_WORKFLOW.md](YAW_MPC_WORKFLOW.md)
for installation, protocol and the first SAFE recordings. This stage does not
enable big yaw, implement MPC or perform automatic motor excitation.

## Big yaw: SAFE-only parameter updates

**Current isolation build:** `BIG_YAW_OUTPUT_INHIBIT_TEST=1` forces big-yaw
drive off at controller and actuator levels so the small-yaw IMU loop can be
checked independently. Big-yaw gains remain saved below, but changing them
cannot enable its output in this build. Big yaw is passive, not locked.
Small-yaw measured angle and raw gyro auxiliary telemetry continue updating
in SAFE when INS is ready; filtered speed, error and effort are controller
diagnostics and still clear in SAFE. Do not interpret them as a stopped IMU.
Keep UP for flash/Reset/Run and perform the passive feedback check before
enabling small yaw. Physical closed-loop operation is not yet verified.

**Current boot defaults (2026-09-07 saved PID revision):** angle Kp/Ki/Kd
50/0.001/4, speed Kp/Ki/Kd 0.6/0/0, effort 30, filter tau 0.030 s. These
restore the user's earlier immediate-follow readback at their latest request.
Small-yaw IMU angle holding is restored (`SMALL_YAW_ANGLE_OPEN_TEST=0`):
manual/PC rates update the angle target, and the original angle PID and gyro
speed PID run with their saved gains. Yaw vision corrections and angle tuning
steps are restored through their original mode/command gates. Encoder soft
limits, feedback validity checks and SAFE remain active. The rate-only
diagnostic helper is retained but does not drive the restored angle controller.
The saved big-yaw policy (currently output-inhibited) follows small
yaw immediately toward the calibrated joint center, with no boundary wait,
dwell or velocity feedforward (`YAW_COORD_IMMEDIATE_FOLLOW=1`). Setting that
macro to 0 and rebuilding restores the alternate HOLD/relief policy, not its
PID preset. The current isolation build enables only the small-yaw loop;
physical stability after flashing still needs verification.
Keep the receiver UP after flash/Reset/Run and verify readback before enabling.
Flash the updated gimbal firmware to use the saved boot defaults;
updating the browser does not change the running controller. Online edits
remain RAM-only. Earlier defaults in the historical notes below are superseded.

The original 2026-09-06 PC-tuning firmware added independent big-yaw tuning. It defaulted
to angle Kp 2.6, speed Kp 0.6, effort 4.0 and speed-filter time constant 0.030 s,
after the stronger defaults produced high-frequency oscillation. No fixed
speed ceiling was restored. The original `index.html` sliders still tune SMALL
yaw only. Use the independent `big-yaw.html` page or `big_yaw_tune_cli.py` for
big yaw. The full-effort firmware revision raises the configurable big-yaw
effort range from 0..8 to 0..30, without raising the boot default of 4.0.
The complete-PID revision additionally exposes Kp/Ki/Kd for BOTH big-yaw
loops. All six gains and filter tau accept finite nonnegative float32 values
without a manually chosen upper bound. Effort remains 0..30 because it maps
to the motor's +/-30000 CAN command, not because of a tuning preference.
These capabilities require manually flashing the updated gimbal firmware.
Older firmware retains its advertised limits; no live settings are changed
by updating the page. That initial revision kept all Ki/Kd at zero; the latest
saved-PID defaults above now supersede it.

### Big-yaw browser page

On the vehicle PC, open the **Big Yaw PID Tuner** desktop shortcut, or open
`/home/nuc11--02/gimbal-pid-tuner/big-yaw.html` in Google Chrome. This is a
local static page; no server or Internet connection is required. Existing
0x2D/0x2E firmware supports the original four settings only. Complete PID
requires the 0x30/0x31 firmware revision described below.
Firefox does not provide the Web Serial connection used by this page.

Close/disconnect other serial clients, including the small-yaw page and
vision application, before selecting the CP2102 adapter. Page opening never
automatically connects, writes gains or enables motion. Only nonlocking
`A5 01` monitor heartbeats are sent after connection, until a user explicitly
confirms a parameter write. Disconnect is not an emergency stop.

On this vehicle, `autoaim.service` starts MiracleVision at boot and has been
confirmed by privileged `fuser` to own ttyUSB0. For a tuning-only session,
keep the receiver SAFE and run `sudo systemctl disable --now autoaim.service`
in the vehicle terminal. This stops vision and its boot activation, without
deleting the program. Close other serial clients, then reconnect the browser.
After tuning, disconnect the browser before restoring vision with
`sudo systemctl enable --now autoaim.service`. Do not expect the independent
vision process and Web Serial tuner to share this UART simultaneously.

The eight fields start blank and initialize from actual firmware readback;
periodic telemetry never overwrites an edited draft. They are angle Kp/Ki/Kd,
speed Kp/Ki/Kd, effort limit and speed-filter tau. On older firmware the four
I/D fields are disabled, and the original fixed-point limits remain in effect.
Use **Read Current Parameters** to replace a draft with a fresh readback.
Writing requires SAFE, outputs off, big inactive, motor online, and configuration
age no greater than 400 ms. A review dialog shows all eight before/after values
and requires a safety checkbox. Changed readback invalidates an open review.
Firmware independently enforces its receive/apply SAFE interlocks.

The write button's caption below it and hover title now report the actual
blocking reason, including unchanged encoded values, empty/out-of-range input,
pending readback, non-SAFE receiver, active output and stale/offline feedback.
Sub-resolution edits are compared after wire quantization, not as raw strings.
This does not weaken any safety gate. Disconnect, refresh the page, then
reconnect to load an updated local page without competing serial clients.

Every write has a distinct request ID and is sent only once. Only matching ID,
accepted status and all encoded values matching are reported as success.
Rejection, mismatch or timeout requires explicit fresh readback before another
write. RAM changes do not survive reset. There are no axis-enable, step, reset,
small-yaw, Pitch or chassis-command buttons or packets.

The three independently scaled charts show error, target/filtered motor speed
and commanded effort from 0x2C. Encoder angle is the live 0..360-degree raw
coordinate, not a calibrated world heading. Stale values disappear, chart gaps
are not connected, and recording can be paused visually, cleared or exported
to CSV (up to 6000 samples in memory). These approximately 10 Hz snapshots do
not replace the optional 0x2F peak diagnostic firmware or high-rate waveform
capture. Output units are driver command units, not measured torque/current.

Implementation: `big-yaw.html`, `big-yaw.css`, `big-yaw-core.js`, `big-yaw.js`.
Icons are vendored [Lucide](https://lucide.dev/guide/lucide) 0.468.0; its ISC
license is included in `lucide-LICENSE.txt`. No CDN is needed at runtime.
`open_big_yaw.py` opens the page in the user's unique existing GNOME/NoMachine
session. The desktop shortcut now uses this launcher. Before launching Chrome,
`prepare_browser_serial.py` briefly opens the unowned CP2102 by its stable
by-id path, sets VMIN=1/VTIME=0 and closes it. It preserves baud and other
terminal settings, never reads/writes data bytes and does not flush buffers.
Busy-port checks and exclusive access prevent intentional competing use.
Opening the HTML directly bypasses this preflight. If switching back from
vision, stop its service and use the desktop shortcut rather than a saved tab.

Offline browser regression fixture: `test-big-yaw.html`. Run only in an
isolated Chrome test profile with `--headless --allow-file-access-from-files
--virtual-time-budget=15000 --dump-dom file:///.../test-big-yaw.html`.
The fixture substitutes in-memory serial streams and reports JSON in `#result`;
it cannot reach the physical UART. `test_big_yaw_web_protocol.py` checks the
same golden wire packet against the existing Python transaction codec.

#### Linux: "The device has been lost" after CLI use

The CLI serial setup previously left `VMIN=0, VTIME=0` on the terminal.
An idle nonblocking read can then return zero bytes without a USB disconnect.
Chromium's [POSIX serial reader](https://chromium.googlesource.com/chromium/src/+/refs/tags/142.0.7444.20/services/device/serial/serial_io_handler_posix.cc)
maps a zero-byte read to DEVICE_LOST and preserves these terminal control
characters when configuring the port. Both CLI setup functions now use
`VMIN=1, VTIME=0`; existing O_NONBLOCK/select/BlockingIOError handling remains
in place. Real Linux PTY tests in `test_serial_compat.py` reproduce the old
empty read and verify the corrected EAGAIN behavior, incoming binary data and
reopening. Do not change terminal settings while another client owns the port.
This correction does not diagnose all USB disconnects or replace checking
kernel logs, device permissions and serial ownership.

Confirmed again on 2026-09-07 after autoaim.service was stopped: the physical
port remained at VMIN=0/VTIME=0 with no new kernel USB disconnect event.
MiracleVision-ppm-dev/devices/serial/uart_serial.cpp explicitly sets VMIN=0.
Thus stopping vision releases its ownership but does not restore browser-safe
terminal settings. The startup preflight addresses that handoff without
changing or rebuilding the vision program. Six PTY/mock lifecycle checks
cover this preflight and the earlier CLI compatibility fix.

On the vehicle PC, from `/home/nuc11--02/gimbal-pid-tuner`:

```sh
python3 big_yaw_tune_cli.py
```

This is read-only configuration query via nonlocking monitor heartbeats. Stop
other serial clients first. After confirming right switch UP and supporting
Pitch as necessary, an explicit single-field adjustment can be sent with:

```sh
python3 big_yaw_tune_cli.py --apply --arm I_CONFIRM --speed-kp 0.6
```

Other values come from fresh readback. Writes are acknowledged and compared
against the complete requested set; no automatic retries, movement or enable
commands are sent. A timeout may mean the write happened but the ACK was lost:
query again before deciding what to do. Settings are volatile across restart.

- `0x2D`, payload `<5H2B`: angle Kp *1000, speed Kp *1000, effort *1000,
  filter time constant seconds *10000, request ID, magic `0xB7`, version `1`.
- `0x2E`, payload `<5H2B`: the four current settings in the same units,
  last processed request ID, status, flags. Sent about 10 Hz during nonlocking
  yaw monitoring (`0x24`, payload starts `A5 01`).
- Status: 0 no processed request, 1 accepted, 2 unsafe, 3 invalid, 4 expired.
- Legacy flags: bit 0 receiver SAFE, bit 1 yaw outputs off, bit 2 big active,
  bit 3 big speed-PID saturation, bit 4 fresh big feedback, bit 5 supports
  effort range 0..30, bit 6 complete float PID, bit 7 format V1.
  Without bit 5 the effort range is 0..8. With bit 6, clients wait for a full
  0x31 snapshot instead of using the range-limited 0x2E parameter words.
- Legacy client ranges: angle Kp 0..8, speed Kp 0..5, effort 0..30 on the
  full-effort firmware (0..8 on older firmware), filter 0..0.1 s.

### Complete big-yaw PID protocol

#### Write-timeout RX correction

The first full-PID revision could lose a complete write burst: four 16-byte
frames exactly filled the USART1 64-byte double DMA buffer. Automatic buffer
switching reloaded NDTR before the IDLE-only handler examined the current
buffer, which could therefore report zero received bytes. The old mux also
discarded partial frames across IDLE boundaries. The user's screenshot showed
valid full-PID readback but no acknowledged update, consistent with this bug;
the exact live byte sequence was not captured.

The corrected firmware keeps a circular RX buffer running and drains a shared
cursor on half-transfer, transfer-complete and IDLE interrupts. Repeated IRQs
cannot replay already consumed bytes; the mux retains partial PC/vision frames
and still validates their CRCs. A partial candidate expires after a 100 ms gap.
DMA IRQs must be serviced before the reader is lapped; runtime worst-case IRQ
latency has not been measured on the vehicle. Host tests reproduce the old
exact-64 loss and cover arbitrary splits, 640-byte streams, CRC failures,
interleaved vision, duplicate drains and full-PID SAFE/atomic staging.

Browser and CLI also leave 20 ms gaps before each parameter frame for older
IDLE-only firmware. This is a compatibility measure, not a substitute for
the firmware reception fix. The same request ID, SAFE checks, all-field ACK
verification and no-retry policy remain. Refresh the page after disconnecting;
after any unknown result explicitly read current parameters before editing.

All packets retain the existing 16-byte CRC8 framing. Eight float32 fields
in wire order: angle Kp, speed Kp, effort, filter tau, angle Ki, angle Kd,
speed Ki, speed Kd. This field order differs from the grouped visual layout.

- `0x30`: four ordered payloads `<HBBff>`: nonzero request ID, part 0..3,
  magic `0xC3`, two little-endian floats. The entire transaction must arrive
  within 300 ms, in SAFE. No partial update is applied. The task rechecks
  SAFE, outputs off and request age <=100 ms before copying all eight values.
- `0x31`: four coherent snapshot payloads `<HBBff>`: processed request ID,
  part 0..3, packed status/flags, two floats. Status occupies bits 0..2;
  bits 3..7 mean SAFE, outputs off, big active, online, saturated respectively.
  A snapshot is scheduled approximately every 200 ms during monitoring.
- Clients reject missing, reordered, mixed or expired snapshot parts. A write
  succeeds only after an accepted, matching-ID, matching-interface snapshot
  containing all eight requested float32 values. No automatic retries occur.
- CLI options additionally include `--angle-ki`, `--angle-kd`, `--speed-ki`
  and `--speed-kd`; omitted settings are preserved from fresh readback.

Ki and Kd use the existing discrete PID's per-control-update coefficients:
the integral sums error per iteration, and the derivative uses adjacent error
differences, not division by seconds. Do not copy continuous-time Ki/Kd values
without conversion. Changing the control update period changes their effect.
The controller retains output saturation and conditional anti-windup. SAFE
and accepted full-PID updates clear integral and derivative/filter histories.
Non-finite intermediate PID arithmetic stops big-yaw output for that update;
an accepted very large gain does not imply useful or stable operation.
The effort maximum is a protocol boundary, not a continuous-duty recommendation.
  These are validation bounds, not validated stable tuning ranges.
- Firmware checks SAFE at receive AND apply time, actual software output
  commands off, and a 100 ms request lifetime. Rejected requests are consumed,
  not queued to run when SAFE is entered later. Motion liveness is unaffected.

The page reads the capability bit and displays the actual firmware effort
maximum; the CLI also refuses above-8 writes to legacy firmware. No automatic
write occurs on a capability update. Wire encoding stays unchanged; at 30.0,
the command and signed effort telemetry are 30000 and fit int16 without wrap.
The [DJI GM6020 guide, CAN protocol section](https://www.mouser.com/datasheet/2/744/RoboMaster_GM6020_Brushless_DC_Motor_User_Guide-1551074.pdf)
defines voltage-command values -30000..30000. In this project the effort value
is multiplied by 1000 by the motor serializer. These are voltage-command units,
not amperes, N*m, or a validated continuous-duty setting. Full-scale permission
does not establish thermal or mechanical safety. The default gains, SAFE
interlocks, offline guards and small-yaw/Pitch control are unchanged.

The CLI is deployed on the vehicle PC; hardware readback and acknowledged
RAM trials are recorded in `BIG_YAW_TUNING_LOG.md`. Observation
using `yaw_limits_cli.py --observe` remains compatible, but 0x2C at roughly
10 Hz is insufficient to characterize high-frequency vibration.

## Existing small-yaw and Pitch tools

This browser tool connects to the gimbal controller USART1 vision serial link
at 115200 baud. Use a Chromium-based browser with Web Serial support.

Open `index.html` directly in Microsoft Edge or Google Chrome, choose the
USART1 serial adapter, and keep the remote right switch in SAFE while applying
parameters or preparing a step test.

The tuner and the normal vision program cannot open the same serial device at
the same time. While the tuner is connected it sends a `0x24` heartbeat; the
controller temporarily pauses vision telemetry and enables PID telemetry. PID
telemetry stops and vision transmission resumes within 750 ms after disconnect.

The tool extends the existing 16-byte PC frame format:

```text
[0xFF][command][12-byte payload][CRC-8][0x0D]
```

Commands `0x20` and `0x23` configure and exercise the small-yaw controller.
Commands `0x21` and `0x22` stream controller telemetry. Tuning traffic is not
counted as an online vehicle motion command.

With the 2026-09-06 symmetric-yaw firmware, command `0x20` accepts `0xFFFF`
for either the speed-limit uint16 at payload offset 4 or the command-step
uint16 at offset 8 to disable that fixed cap. All other values keep their
existing scaling; **zero is still zero, not unlimited**. The browser defaults
to checked no-fixed-cap checkboxes. Uncheck one to apply a positive or zero
cap. CLI defaults are `--speed-limit -1 --manual-step -1`. Firmware Watch
variables use -1 for the same sentinel. Mechanical boundary braking and
output limits are never disabled by these options. This requires the new
firmware; old clients sending 24 rpm / 0.30 deg will restore those caps.
These local tool changes have not been deployed to the vehicle PC.

Offline codec tests: `python3 -m unittest test_pid_rate_caps.py`.

`pitch_tune_cli.py` uses commands `0x25` through `0x28` for the Pitch IMU
outer loop. It reports IMU attitude/rate, DM4310 position/rate/torque, and the
signed gravity feedforward. The Pitch action frame also carries DM4310 MIT
`Kp/Kd`, configured with `--motor-kp` and `--motor-kd`. Active enable or step tests require the
`I_CONFIRM` arming phrase and disable the Pitch IMU outer loop when the test
ends unless `--leave-enabled` is supplied.

The 2026-09-05 audited firmware adds a nonlocking monitor session:
`0x24` payload starts with `A5 02` (monitor, Pitch). Run
`python3 pitch_tune_cli.py --monitor-only --duration 30 --csv pitch-response.csv`
to record manual operation without suppressing remote Pitch commands. Normal
tuning sessions still lock the selected axis. Vision TX remains paused during
either session to share USART1 bandwidth. Older firmware cannot acknowledge
this monitor mode; the CLI returns failure when `monitor_verified` is false.

Packet `0x29` reports `<4h4B`: motor minimum/maximum and IMU minimum/maximum
in mrad, limits initialized, DM driver state, monitor-only flag, protocol
version (1). Limits are not valid until `initialized` is true. Motor `online`
also requires enabled state and fresh feedback; it is not a CAN-only status.

Pitch defaults match the accepted tuning: IMU Kp/Kd 14/0.30, target rate limit
2 rad/s, MIT Kp/Kd 50/3.2, gravity effort 0.30. Raw attitude error is reported
separately from the firmware's deadbanded error. Active tests abort on a
0.5-second telemetry gap and attempt to disable the IMU outer loop even with
`--leave-enabled` after an I/O error. This is a best-effort serial command;
it does not disable MIT stiffness or replace the remote emergency stop.

Offline tests: `python3 -m unittest -v test_pitch_tune_cli.py` (Linux).

## Small-yaw mechanical landmarks

`yaw_limits_cli.py` only transmits the nonlocking `0x24 A5 01` monitor
heartbeat. It never changes gains, references or motor enable state.
New gimbal firmware provides packet `0x2A` at up to 20 Hz in this session,
including in SAFE. Existing `0x21` yaw feedback is IMU heading, NOT the
mechanical joint angle, and must not be used for this calibration.

The 12-byte `0x2A` payload is little-endian `<4H4B`:
small/big raw encoder counts (0..8191), small/big feedback age in ms
(saturates at 65535), feedback-valid mask (bit 0 small, bit 1 big),
yaw output active, monitor-only, protocol version (1).
Validity bits mean initialized with in-range counts; freshness must also
be checked using age. Output active includes nonzero motor output requests;
it is software status, not a measurement of physical torque.

Support Pitch before selecting remote right-switch upper SAFE. Keep the
receiver and CAN feedback powered, stop other serial clients, and do not
reboot between captures. Keep big yaw stationary. First hold small yaw at
an interior, approximately forward-facing position; then gently move it
by hand to each mechanical endpoint. Never drive against a stop using
the remote or a position command. Capture only after motion has stopped.
Left/right labels use the vehicle's forward-facing perspective.

```sh
python3 yaw_limits_cli.py --label middle --json yaw-middle-01.json
python3 yaw_limits_cli.py --label left --json yaw-left-01.json
python3 yaw_limits_cli.py --label right --json yaw-right-01.json
```

Each command records 3 seconds and refuses to accept stale (>100 ms),
active-output, unsupported-protocol, incomplete or moving (>0.5 encoder
degrees span) data. Files are never overwritten. The interior landmark
helps distinguish the permitted arc when endpoints straddle encoder zero;
do not blindly subtract endpoints or select the shortest circular arc.
Counts are encoder coordinates, not output-shaft angles if a reducer is
present. Confirm transmission ratio and direction before computing limits.
This utility records evidence only: no mechanical or soft limits are applied.

Offline tests: `python3 -m unittest -v test_yaw_limits_cli.py` (Linux).

Small-yaw soft-limit firmware additionally sends monitor packet `0x2B`
at up to 10 Hz: `<4h4B` joint/minimum/maximum in centidegrees relative to
visual center, limited world speed reference in deci-rpm, limits-valid,
status mask, big-yaw-enable request, protocol version (1). Status bits:
1 reference clipped, 2 speed clipped, 4 outward effort blocked, 8 outside
soft bounds, 16 invalid feedback/configuration. The guard-valid field is
zero in SAFE; joint telemetry in this packet is then not updated. Use
`0x2A` for live SAFE position. `yaw_limits_cli.py` attaches the latest guard
packet and its age to the landmark report; landmark acceptance still
requires SAFE and stationary feedback, not merely reception of this packet.

## Big-yaw coordination observation

Packet `0x2C` is `<4h4B`: big-yaw position error in centidegrees,
speed reference/filtered feedback in centi-rpm, effort in milli driver units,
mode, active, relief-negative flag, protocol version (1). Modes are 0 disabled,
1 hold, 2 left relief, 3 right relief, 4 braking. The active flag is distinct
from the requested enable byte in packet `0x2B`.

`python3 yaw_limits_cli.py --observe --label check --duration 30 --json yaw-run.json`
records moving/enabled axes with the same pure-monitor heartbeat. It sends
no parameter, step or enable commands and is not an emergency-stop tool.
Observation acceptance checks recording quality and fresh guard/state
packets; it is not an assertion that the controller behaved correctly.
Big-yaw travel statistics unwrap successive encoder samples across zero.
The default mode still rejects moving/enabled data as mechanical landmarks.

### MCU peak diagnostics (0x2F)

`python3 yaw_limits_cli.py --observe --require-peaks --label check --duration 30 --json yaw-peaks-run.json`
requires the peak-diagnostic firmware. Older firmware is rejected as missing
telemetry, not interpreted as zero motion. This remains a nonlocking monitor:
no gain, target, enable or motion command is sent.

The MCU samples after each yaw control iteration (nominal 1 ms task delay) and
reports windows approximately every 100 ms, only during a nonlocking yaw
monitor session. Payload is six little-endian uint16 words, version 1:

1. Bits 0..13: valid enabled task samples; bit 14: any output saturation;
   bit 15: invalid window (stale/nonfinite feedback, task gap over 10 ms,
   ambiguous half-turn delta, or numeric/count/time overflow).
2. Maximum unwrapped encoder span, in 8192 counts/turn.
3. Peak absolute position-loop error, centidegrees.
4. Peak absolute raw motor CAN speed, centi-rpm, before software filtering.
5. Peak absolute commanded effort, milli driver units (not measured torque).
6. Window duration, milliseconds.

Only active samples contribute to peaks. SAFE breaks angle continuity, so
manual movement while disabled is not included; zero samples means no active
coverage, not proof of quiet closed-loop control. A window spanning multiple
active segments reports their largest separate span. Boundary deltas between
successive windows are preserved. Peaks reset only after UART accepts a frame;
this is not a delivery ACK. Reports check freshness, invalid flags and window
coverage, and retain all peak frames preceding an encoder record in each read.
The last partial window is not flushed when capture ends; the first window
can begin just before capture if another monitor session was already active.

Task sample count is not a distinct-CAN-frame count. Feedback can be repeated
between CAN arrivals; shorter-than-task/CAN transients may still be missed.
Window peaks are not a waveform, and fields need not peak simultaneously.
Use this to diagnose the discrepancy before further PID changes, not as proof
of mechanical rigidity or stability. Small yaw, Pitch and control limits are
unchanged. Restart still restores big-yaw defaults 2.6/0.6/4.0/0.030, not the
last RAM-only trial 4.0/0.7/4.0/0.030.
