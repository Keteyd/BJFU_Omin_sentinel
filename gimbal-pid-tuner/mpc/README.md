# Dual-yaw MPC, offline stage

Current deployment: build `0x59490910` retains the field-accepted `0x59490908`
20 ms, 15-step MPC and MIDDLE-mode remote chassis translation in the complete
small-yaw frame. Left-switch DOWN selects the 2026-09-07 cascaded-PID immediate-follow
controller without changing that chassis frame. Both speed PIDs, effort limits,
feedback guards, and small-yaw mechanical limits remain active. See the
[PID fallback change](MPC_PID_FALLBACK_20260913.zh-CN.md),
[small-yaw chassis-frame change](MPC_SMALL_YAW_CHASSIS_FRAME_20260913.zh-CN.md)
and [field steps](MPC_DEPLOYMENT_FIELD_STEPS_20260912.zh-CN.md).

[中文说明与双轴辨识步骤](README.zh-CN.md)

Current status and safety boundaries are indexed in
[the Chinese project status document](MPC_PROJECT_STATUS_AND_SAFETY_20260910.zh-CN.md).
The selected MPC interface is now a pair of speed references: big-motor speed and small-axis inertial
heading rate. Build `0x59490702` implements a dedicated direct-speed S1/S2 identification profile. For
the operator-confirmed cable-free setup it disables big-yaw and heading relative-travel stops while retaining
the small-joint absolute/relative travel guards, speed PIDs, effort limits, feedback, rate, remote-stop, and link guards. S1 is
development data. S1 completed on 2026-09-12 with 8996 records, full-duration timing, no quality/CRC/CAN
faults, and has been released. A stable 2-output-lag/4-reference-lag 20 ms speed predictor and the one-time S2
acceptance gates were frozen before S2. S2 then ran for the full 36 seconds without motion/CAN/feedback faults,
but its 8994 records missed the frozen maximum shortfall by two records, so formal validation fails. The unchanged
model would pass every numerical S2 gate diagnostically (200 ms improvements 48.1%/80.9%), but that cannot
override the data gate. S1/S2 must not be repeated. See the
[implementation review (Chinese)](SPEED_REFERENCE_IMPLEMENTATION_REVIEW_20260911.zh-CN.md) and
[field steps (Chinese)](SPEED_REFERENCE_FIELD_STEPS_20260911.zh-CN.md). No hardware takeover is authorized.
[The S1 capture review (Chinese)](SPEED_S1_59490702_CAPTURE_REVIEW_20260912.zh-CN.md) records the accepted
trace, runtime margins, independent raw replay, and final idle state.
[The S1 model freeze (Chinese)](SPEED_S1_MODEL_FREEZE_20260912.zh-CN.md) records the model, internal checks,
artifact hash, and immutable S2 gates.
The new, uncollected S3 waveform and time-coverage gate are prospectively frozen in
[the S3 design](SPEED_S3_DESIGN_FREEZE_20260912.zh-CN.md). Build `0x59490801`, its S3-only host
tool, and the frozen offline validator are implemented and verified before collection. S3 may now be
collected once by following the [field steps](SPEED_S3_FIELD_STEPS_20260912.zh-CN.md); it may not be
used to change its own model or gate.
[The implementation review](SPEED_S3_IMPLEMENTATION_REVIEW_20260912.zh-CN.md) records the firmware,
host decoder, frozen validator, build/test evidence, and release hashes created before S3 collection.
[The S3 validation review](SPEED_S3_59490801_VALIDATION_REVIEW_20260912.zh-CN.md) records the one-time
capture. A transient `ins_not_ready` condition aborted the run at 34.173 s during the post-excitation settle,
so the frozen formal data gate rejects S3. The complete 3–31 s model window nevertheless passes all four
unchanged numerical gates diagnostically (200 ms improvements 49.6%/84.6%, RMSE 3.920/2.493 deg/s).
This supportive diagnostic does not override the data gate; S3 is released and must not be repeated.
[The S2 validation review (Chinese)](SPEED_S2_59490702_VALIDATION_REVIEW_20260912.zh-CN.md) separates the
formal data-gate failure from the supportive unchanged-model diagnostic.
F4/F5 command-to-joint plant development is complete. A fixed-kinematics IV velocity model improves
the F5 active-window big/small predictions over hold by 30.5%/43.9%, and its 100 ms rolling RMSE is
0.201/0.254 degrees. It nevertheless drifts by 4.836/2.941 degrees during the F5 return-to-zero segment;
baseline adaptation and raw-current input do not repair the failure. The open-loop plant therefore
remains unidentified. See the
[F4/F5 operating-domain plant review (Chinese)](F4_F5_OPERATING_DOMAIN_PLANT_REVIEW_20260911.zh-CN.md).
The cable-free E/R1/F4/F5 comparison is complete. The same-waveform R1 run reduces actual
big/small/heading tracking RMSE by 9.9%/20.2%/9.8% versus cable-present E, while residuals from
the old cable-present model remain strongly slow and position-correlated. Frozen-C normalized
errors are nearly identical at 4x and 5x, supporting local scale transfer, but the small-joint
error remains too large and this is still a PID closed-loop reference map. The operator confirms
that F4/F5 are closer to the competition operating regime, so future model selection treats 4x–5x
as the primary domain and uses R1 only as a cable/low-speed corner diagnostic. See the
[cable-free scaling and model review (Chinese)](CABLE_FREE_SCALING_MODEL_REVIEW_20260911.zh-CN.md).
Build `0x59490503` phase E has completed and passed the predeclared data-quality and unchanged
frozen-C validation gates. See the [E validation review (Chinese)](E_REFERENCE_VALIDATION_REVIEW_20260911.zh-CN.md).
The 8990/9001 records cover the full 36 seconds and are within the predeclared 45-record limit.
Frozen-C improvement over hold is 46.49%/34.11% for the big/small joints; C/E current gain and
delay consistency also pass. A one-byte trailing status-frame prefix initially caused a host-only
`incomplete_stream` label; independent raw replay with the corrected framing boundary has no
quality issues. E is released and must not be repeated. The result qualifies the closed-loop
reference-map candidate, not an open-loop MPC plant, and does not authorize hardware takeover.
The post-capture `yaw-e-trace-59490503-hostfix1.zip` disables all motion actions and preserves the
corrected replay boundary for audit use.
The [post-E C/D/E residual and friction review (Chinese)](CDE_RESIDUAL_FRICTION_REVIEW_20260911.zh-CN.md)
is also complete. Causal leave-one-phase-out diagnostics show that a correction using lagged measured
position, velocity/direction, software command and baseline-centered current reduces the worst held-out
big/small-joint residual RMSE by 59.27%/79.76%. This is an observer-correction candidate selected after
all three phases were consumed, not a validated free-run plant. Simple constant-bias and reference-only
corrections do not generalize. A cable-free amplitude ladder is now frozen: R1 repeats the exact E shape,
F4 is the primary 4x capture, and F5 is an optional 5x capture only after every measured F4 travel, rate,
and command remains below 80% of its runtime bound. A proposed 10x capture was rejected because it exceeds
the big- and small-joint travel budgets and is expected to saturate the small command. Build `0x59490601`
and its host package are verified and deployed but have not yet been flashed or captured. See the
[implementation review (Chinese)](CABLE_FREE_LADDER_IMPLEMENTATION_REVIEW_20260911.zh-CN.md) and
[field steps (Chinese)](CABLE_FREE_LADDER_FIELD_STEPS_20260911.zh-CN.md).
The [E implementation review (Chinese)](E_REFERENCE_IMPLEMENTATION_REVIEW_20260911.zh-CN.md) and
[E field steps (Chinese)](E_REFERENCE_FIELD_STEPS_20260911.zh-CN.md) preserve the pre-capture design.

The preceding field build was `0x59490502`. Its phase-D retry completed the full 36 seconds without
repeating the false feedback timeout. The [D retry review (Chinese)](CD_D_RETRY_REVIEW_20260911.zh-CN.md)
records the result: the original frozen capture gate rejects 8983/9001 records because its
shortfall limit was five, while a separately labelled timing-resampled diagnostic passes the
unchanged frozen-C motion and current gates. D is retained as conditional engineering evidence,
not rewritten as a formal independent pass. Phase E subsequently supplied the clean predeclared
independent validation described above.
The earlier `0x59490405` build removed operator-observable ARM admission checks and the mandatory
BENCH qualification; BENCH remains an optional zero-output diagnostic in compatible tools.
Phase sets A and B have now completed safely and both captures pass their trace-time data gates.
The [independent A/B review](DUAL_AB_59490405_VALIDATION_20260910.zh-CN.md) rejects the fitted
closed-loop motion model: frozen A-to-B improvement is only 27.6%/22.9% for the big/small
joints against the predeclared 30% gate, and the symmetric B-to-A small-joint improvement is
20.7%. The command-to-feedback-current timing and gains reproduce well, but the selected ARX
structure does not. Do not repeat A/B; the next work is offline model and excitation redesign.
No fitted result authorizes MPC hardware takeover.

The next implementation stage is complete: [A/B residual, friction and C/D design review
(Chinese)](NEXT_STAGE_CD_DESIGN_20260911.zh-CN.md). A/B are now development data. A common
12-output-lag/4-reference-lag ridge-regularized structure improves the worst cross-phase joint
case by 36.2%, but its residual remains strongly low-frequency and correlated. The proposed
36-second C/D profile adds smooth reversal dwells and five disjoint multisine components per
axis. Build `0x59490501` and the strict trace-v5 host tool implemented that frozen profile.
Phase C completed and its model is frozen. The first D attempt stopped at 10.590 seconds because
an IRQ could advance the feedback timestamp beyond a previously read millisecond clock, causing
unsigned age underflow and a false `feedback_guard`. The
[D abort review (Chinese)](CD_D_FEEDBACK_RACE_REVIEW_20260911.zh-CN.md) records the evidence.
Build `0x59490502` takes the clock and feedback timestamp atomically and preserves the existing
50 ms feedback limit; C/D signals, controller settings, and all other guards are unchanged.
The operator-approved amplitudes now use the full 3.00/2.00 degree component budgets, with
the remaining budget assigned to the two lowest frequencies on each axis.
The former 100 deg/s^2 value was an offline preview threshold, not a firmware runtime abort;
it is now 200 deg/s^2. The existing feedback, travel, measured-rate, heartbeat, remote-stop,
capture, and finite-output guards remain enabled. C must be reviewed before the one-time D run.

Latest vehicle captures: [small-yaw CAN trace review (Chinese)](SMALL_CAN_TRACE_REVIEW_20260910.zh-CN.md)
and [big-yaw CAN trace review (Chinese)](BIG_CAN_TRACE_REVIEW_20260910.zh-CN.md).
Both 5001-record positive-profile runs are structurally valid and retained; do not repeat them.
The combined [CAN-trace model review (Chinese)](CAN_TRACE_MODEL_REVIEW_20260910.zh-CN.md)
uses interval command integrals and measured microsecond timing. The exploratory candidate is rejected:
big-trial 18-second free-run heading RMSE is 70.06 degrees versus 0.50 degrees for hold.

Latest: [CAN input history, send status and raw current capture (Chinese)](CAN_TRACE_20260910.zh-CN.md).
Build `0x59490301` and `yaw_can_trace_cli.py` are implemented and tested offline.
The user flashed the firmware and reported a passing UP-only zero-output BENCH:
5001 records, no CRC errors or quality issues. Raw files were retrieved over SSH/SCP;
independent replay matches the report. Duplicate rpm CSV columns were fixed locally and re-exported from intact raw data;
the corrected CSV round-trip passes. No firmware reflash or further vehicle trial is currently needed.

Previous: [input timing alignment (Chinese)](TIMING_ALIGNMENT_20260909.zh-CN.md).
Aligned actuator integrals and a six-state predictor pass the controlled
synthetic recovery and frozen new-phase checks. Vehicle free-run checks still
fail. The [closed-loop/current timing review (Chinese)](CLOSED_LOOP_CURRENT_REVIEW_20260910.zh-CN.md)
causally aligns feedback timestamps and uses independent zero-output BENCH bias estimates. A descriptive
reference-to-PID-closed-loop FIR reaches 0.273/0.268 degree held-out heading RMSE, but the instrumental-variable
current-to-joint plant is unstable and fails forced-response validation. All 80 offline MPC tests pass.

Previous stage: [stiction parameter recovery audit (Chinese)](STICTION_RECOVERY_20260909.zh-CN.md)
reproduces severe parameter bias after encoder quantization. A new experimental
angle-weighted fit reduces that bias but still fails the full synthetic recovery
gate and original vehicle free-run checks. At that stage 62 offline tests passed; no vehicle
model is ready for MPC. The integration/build narrative below is historical.

Current identification setup: fixed chassis, usual Pitch pose, camera not
installed. See [load record](identification_setup.json) and
[scheduler preparation status (Chinese)](IDENTIFICATION_CORE.zh-CN.md).
The C scheduler is now integrated through a dedicated USART1 identification
protocol, exclusive control ownership and a 250 Hz compact buffer. The host
tool is `../yaw_identification_cli.py`; its default is query-only. This build
has passed offline tests and AC6 linking, but has not been flashed or exercised
on the vehicle. Follow the Chinese workflow for the initial UP-only check.

The MPC code in this directory implements a working constrained QP and
synthetic closed-loop regressions. The optimizer itself has NO serial, CAN,
PID writes or motor takeover. The separate identification tool can arm a
bounded trial only with explicit confirmation and a physical switch transition.
The default plant is explicitly
`SYNTHETIC_UNIDENTIFIED_NO_HARDWARE_USE`. Synthetic results are not vehicle
performance or a reason to remove the current big-yaw inhibition.

## Controller objective

Use one optimizer for the two axes rather than two independent angle loops
and a switching follower. The six-state local model is:

```
x = [q_big, q_small, dq_big, dq_small, a_big, a_small]
M ddq + D dq = K a
T da/dt = u - a
y_heading = q_big + q_small      (synthetic fixed-base planar case ONLY)
```

Each joint has second-order mechanical dynamics. A full SPD two-by-two
inertia matrix couples both accelerations; actuator lag adds two states.
`a` is a lagged software drive command, NOT measured motor current or torque.
`K`, `M`, damping and time constants are invented simulation parameters.
They must not be interpreted as measured inertias or GM6020 torque constants.
The plant is discretized with SciPy zero-order hold, not forward Euler.

Costs penalize heading error strongly, big relative-joint velocity, small
velocity, input and input changes; a weak big-angle anchor and very weak
small-center cost resolve remaining allocation freedom. The big-angle anchor
must be captured at mode entry, not reset to measurement each step. These
weights are finite tradeoffs, not a mathematical lexicographic priority.
Big-yaw reaction torque may be needed even when its intended movement is zero.
"Move less" does not mean "send less torque" or never use the big motor.
An asymmetric comfort interval spans 70% of each side's soft travel. Positive
slack variables penalize excess beyond that interval, so small yaw can still
use its remaining travel for fast aiming while big yaw gradually restores
reserve. This is a tunable cost, not a second hard limit. Only comfort is
softened; the physical travel constraint never receives a slack variable.

The planar sum is NOT the final estimator for the tilted, parallel-but-offset
physical installation. Actual IMU heading and calibrated encoder frames must
be fused; body gyro Z must not be substituted for Euler heading rate. Base
rotation, pitch/roll, offset-dependent inertia, friction, drive polarity,
voltage/CAN delay and state estimation remain identification/integration work.
The actuator states require an observer or validated input-history estimate.
Changing geometry should change the model/measurement mapping, not the QP
machinery. The present six-state implementation is deliberately a starting
model, not a forced order for later identification.

## Limits and failure behavior

- The small-joint bounds use actual measured encoder landmarks from
  `module_yaw_limits.h`: -56.677734375..+29.080078125 degrees, relative to the
  actual visual center 6816 counts, NOT the midpoint of asymmetric stops.
- Bounds constrain predicted state nodes, including the initial state. A
  target beyond available small-joint travel remains a heading target: the
  optimizer can allocate motion to the unlimited big joint instead of simply
  clipping the heading demand. A terminal zero small-relative-rate equality
  requests braking within the horizon. It is not an invariant terminal set
  or proof of recursive feasibility. There is no slack that relaxes travel.
  Future nodes reserve an additional 0.1 degree numerical margin so finite
  solver tolerance does not plan exactly on a stop. Initial-state validation
  still uses the measured soft interval. This prototype margin is not an
  identified uncertainty bound and does not change firmware limits.
- No hard velocity cap, big-angle bound, or hard input slew cap is added.
  Motion/rate and input changes are penalized, not hard-limited. Finite drive
  effort is essential for realistic prediction: simulation uses the existing
  operating limits big 30 / small 6 software units. These are not calibrated
  torque limits; neither firmware limits nor hardware capacity are increased.
- Out-of-range initial position, invalid/stale input, infeasibility, solver
  iteration/deadline failure, inaccurate status or excessive QP residual
  returns `command=None`. It never reuses a previous command as fallback.
  This is an offline rejection result, NOT an implemented safe vehicle stop.
- Tests check intermediate plant points every 5 ms, between 20 ms MPC nodes.
  This still does not guarantee continuous-time collision avoidance. Model
  error can make a nominally feasible solution unsafe; later work needs
  uncertainty margins, validated braking/viability and a firmware supervisor.
- Continuous unwrapped big angle and heading references support slip-ring
  turns. Do not wrap every target to the nearest +/-180 degrees: an intentional
  multi-turn trajectory and shortest-path aiming are different interfaces.

## Run locally

From the repository root, the prepared independent Windows environment is:

```powershell
NoMachineTemp/yaw-mpc-venv/Scripts/python.exe gimbal-pid-tuner/mpc/simulate.py --output NoMachineTemp/mpc-sim-new
NoMachineTemp/yaw-mpc-venv/Scripts/python.exe gimbal-pid-tuner/mpc/audit_capture.py NoMachineTemp/yaw-release-20260908-01/burst.csv
```

From this directory run tests using that environment's Python:
`python -m unittest test_mpc -v`.
For another offline environment: `python -m pip install -r requirements.txt`.
`requirements-tested.txt` records the exact Windows verification environment;
other Python/platform combinations may need compatible dependency versions.
Simulation writes per-case CSV and `summary.json` to a NEW directory, never
overwrites evidence, and stops on a rejected solution or simulated limit
violation. Cases cover both asymmetric boundaries, small heading steps,
initial velocity disturbance, +25% inertia mismatch and crossing a full turn.
These are software regressions, not new requests for operator movement.

Verified 2026-09-08: 13 MPC tests pass, including six closed-loop scenarios.
Final comfort-zone run: `NoMachineTemp/yaw-mpc-sim-20260908-02/` (1200 solves,
six CSV files plus summary). No simulated soft-limit violation was observed.
The +20-degree heading request near the positive boundary ended at small
joint +22.72 degrees; the negative-boundary case ended at -44.12 degrees,
leaving travel in each case. These are synthetic results only.

In the 10-degree synthetic step, big-motion weight 25 reduced peak big-joint
displacement from 19.89 degrees (weight zero) to 3.14 degrees. This is a cost
function regression, not a forecast of real-vehicle performance. Final run
solve times peaked at 7.61 ms on this Windows desktop, but the prior no-comfort
run had a 48.52 ms outlier. No real-time deadline guarantee follows either.

The captured release audit gives input rank 1, big drive always zero. The
audit is intentionally unable to emit an identified model or authorize
takeover even if a future input matrix has full rank.

The initial 20 ms control interval and 0.6 s horizon are offline prototype
choices. They do not establish the target MCU/PC rate or extreme-performance
capability. Reported solve times are desktop observations, not worst-case AC6
timings. No high-bandwidth loop should be routed over the present 40 Hz
monitor interface; the 500 Hz stream is buffered, not a real-time command bus.

## Mainline next steps

1. Treat the user's manual small-yaw verification as sufficient to leave
   runaway diagnosis for now. No more opposite-direction PID troubleshooting
   is required by this workflow.
2. Build an explicitly authorized dual-input identification procedure. Each
   motor must have independent input variation while BOTH joints and IMU are
   recorded. Existing closed-loop PID data can help initialize a model, but
   feedback bias and actual drive dynamics must be accounted for. A rank
   check alone is not persistent excitation or model validation.
3. Identify the coupled input/state and measurement models, validate with
   held-out data across operating poses, then replace the synthetic plant.
   The existing zero-big-input capture cannot identify the big input column.
4. Compare predicted vs measured trajectories in replay/shadow mode. Establish
   braking margins, infeasible-state handling, sensor freshness and a takeover
   state machine. Pitch/chassis and existing safety supervision remain separate.
5. Select deployment rate/horizon/solver after profiling fixed memory and
   worst-case execution on AC6. MCU-local output supervision is mandatory;
   decide whether the optimizer is MCU-local or PC-assisted from those results.
6. Only after a separate hardware readiness confirmation, perform bounded
   takeover. Do not plug this script's output into the current serial tuner.

## Solver references

The sparse QP/state-equality layout and receding-horizon approach follow the
[official OSQP MPC example](https://osqp.org/docs/examples/mpc.html).
Only `solved` is accepted; see [OSQP status definitions](https://osqp.org/docs/interfaces/status_values.html).
