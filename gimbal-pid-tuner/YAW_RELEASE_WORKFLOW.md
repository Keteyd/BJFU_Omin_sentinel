# Host-triggered small-yaw release capture

Prepared 2026-09-08. No new flash is needed after the validated byte-wise IMU
read revision. This recorder sends only v1/v2 monitor heartbeats, not motion,
PID writes or stop commands. It is not MPC or an actuator interlock.

Deployment: `/home/nuc11--02/yaw-release-zXa3AHXz` on `192.168.1.113`.
Original tools/captures are unchanged. No real serial access or powered
experiment was performed while preparing this tool.

## Operator prerequisites

- Obtain new readiness confirmation for this trial. Start right switch UP,
  yaw stick centered, mechanism/cables clear, with an accessible stop.
- Big yaw remains inhibited; small yaw remains IMU closed-loop. DOWN also
  enables other normal vehicle functions: secure the chassis and account
  for Pitch motion. Current wire flags cannot distinguish DOWN/manual from
  MIDDLE/auto. The operator must select DOWN, not MIDDLE.
- Do not force a powered joint. Last small encoder was about 316.8 degrees,
  joint +17.3, not true center 299.53125 degrees. About 11.8 degrees remained
  to the positive soft boundary. Confirm current pose and choose a brief
  center-side excursion. RC sign alone does not prove motor motion polarity.
- Close serial clients; do not restart auto-aim. Occupied ports cause an
  abort, never a forced process termination.

## Procedure

Only after explicit readiness confirmation, run on the NUC:

```sh
python3 /home/nuc11--02/yaw-release-zXa3AHXz/yaw_release_capture_cli.py \
  --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
  --output /home/nuc11--02/yaw-release-zXa3AHXz/captures/release_01
```

Use a new output directory each time. Remain UP/centered for the 500 ms SAFE
baseline. When ready, enter DOWN/manual with stick centered. After the
enabled-center prompt, make ONE brief small excursion and release. Detection
requires channel magnitude at least 40 and a same-sign excursion spanning
100 ms; this is not a commanded movement amplitude.

Return UP within about one second after release. If motion is unexpected,
go UP immediately, without waiting to complete the trace. The host's delayed
1.3-second reminder is not verified MCU acquisition completion. Wait for the
download only after returning UP. Program/serial closure or an error does
NOT stop normal motor control; the operator must stop it.

## Evidence and limits

- `live.csv`: 40 Hz history through release. `burst.csv`: 512 buffered 500 Hz
  samples after v2 reaches the MCU. `raw.bin`: all RX bytes. `report.json`:
  event, timing, diagnostics, errors. Partial evidence survives failure;
  no automatic retry or overwrite.
- No high-rate pretrigger exists. Detection uses 25 ms observations, plus
  serial/scheduling delay. Reported MCU delay is from the detected zero-stick
  sample to first burst sample, NOT from exact physical release. More than
  150 ms fails alignment. Acquisition spans 1.022 s; download takes ~13 s.
- There is NO live 40 Hz state stream during burst acquisition/download.
  Buffered flags are historical. Auxiliary diagnostics during download
  describe later live counters, not the buffered sample's instant.
- Invalid observations, gaps, isolation/mode mismatch or travel within
  3 degrees of a soft boundary abort pretrigger recording. These observational
  checks cannot stop a motor; existing firmware soft limits are unchanged.
  SAFE transitions cancel rather than trigger the capture.
- Trailing v1 fragments can increment v2 unsupported-version counts at the
  handoff. Missing/corrupt v2 samples are rejected. Raw data supports replay
  using either version of the existing decoder.
- Compare input, target, INS angle/rate, encoder, speed and output together.
  A constant target after release is expected position hold, not a ramp.
  Output units are not measured torque. This isolated diagnostic alone is
  insufficient to fit a coupled actuator model or prove the runaway fixed.

## Tests

`python -m unittest test_yaw_release test_yaw_burst test_yaw_capture -v`

Windows: 24 tests, 23 pass, Linux pseudo-terminal test skipped.
NUC: 24 tests, 22 pass, two Windows C-fixture tests skipped. The Linux test
exercises the actual CLI with a virtual serial peer, checks a complete
512-point handoff and asserts that every TX packet is a monitor heartbeat.
It does not open real hardware serial ports.
