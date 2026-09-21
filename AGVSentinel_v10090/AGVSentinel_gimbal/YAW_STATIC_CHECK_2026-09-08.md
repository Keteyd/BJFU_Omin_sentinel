# Yaw measurement and reference-path check

## Scope and outcome

Follow-up to the passive `stationary_02` and `manual_small_01` captures.
No serial commands, live tuning, flashing, reset or powered motion were
performed during this code check. The chassis and pitch controller were not
edited. Big-yaw output inhibition and the small-yaw IMU loop remain unchanged.
This is a targeted check, not certification of the whole control system.

## Fixed: undefined IMU calibration input

`Src/Periphal/periph_bmi088.c`, `BMI088_GetOffset` previously converted four
uninitialized stack words to floats: its Flash read was commented out.
The subsequent checks also did not reject NaN or large negative offsets.
Depending on stack contents, a boot could select invalid calibration data.

The function now installs the existing board defaults directly. No Flash
read/write or new calibration storage format is introduced. `BMI088_Init`
now selects either explicit calibration or defaults, rather than overwriting
the result of `BMI088_SetOffset`. Normal startup still calls `BMI088_Init(0)`
in `app_init.c`, so this build does NOT perform automatic zero-bias learning.
Defaults are deterministic, not newly measured values for this sensor.

The source was not valid UTF-8. Before editing it was backed up to
`NoMachineTemp/periph_bmi088-before-offset-fix-20260908.c` in the workspace
root, then losslessly decoded as CP936 and saved as UTF-8. Existing comment
text was not repaired or otherwise rewritten.

## Findings that are not yet root-cause conclusions

- The scheduled input task is `Control_Task`, not the legacy `Remote_Task`.
  `Control_UpdateTargets` starts from a zeroed target each cycle. In manual
  mode it maps current channel 2 to rate; `Control_GimbalStep` integrates that
  rate into the angle target. With a zero channel and no autoaim contribution,
  the reference increment is zero. Holding the last angle after release is
  intended; holding the last nonzero rate was not found in this path.
- `Remote_CancelChannelOffset` only subtracts 1024. There is no center deadband.
  Each remaining channel count requests 0.2 deg/s in magnitude (nominal task
  timing). Existing stationary samples were zero, so a stick-release residual
  must be measured before attributing continuous rotation to this omission.
- Reference integration uses a nominal 2 ms while `Control_Task` uses relative
  `osDelay(2)`. The gimbal task uses relative `osDelay(1)`; the PID I/D and
  small-speed filter operate per call. There is no recorded control-loop dt
  in capture v1. INS dt alone cannot establish the other tasks' timing.
- INS uses measured DWT dt, not a fixed 1 ms. The first dt starts from a zero
  cycle counter; startup elapsed time and debugger pauses merit a separate
  timing guard review before identification. Changing all controller timing
  at once would change existing gain meanings and is not part of this fix.
- Small-yaw angle feedback is `INS.YawTotalAngle`, but speed feedback is
  `INS.Gyro[Z_INS]`, not an attitude-transformed Euler heading rate. The angle
  direction/scale passed the hand-motion check at the tested attitude. That
  does not validate this rate approximation across vehicle tilt/pitch angles.
- `QuaternionEKF_Update` subtracts estimated X/Y gyro bias internally and
  sets estimated Z bias to zero. Published `INS.Gyro` is the driver-corrected
  input, not this EKF-corrected vector. The stationary drift remains a measured
  issue, not proof that this initialization fix will eliminate it.
- Additional sensor validity risk remains: `BMI088_BMI088DecodeData` updates
  its timestamp before reading and marks connected even if the gyro chip ID
  fails. INS publishes an observation on every iteration. Thus update counts
  and ages alone do not prove fresh successful hardware reads. Hand-motion
  correlation provides separate evidence for the captured run. Proper read
  success propagation and fault-injection tests are needed before powered
  identification; they are not implemented by the offset-only patch.

## Verification

- New `Tests/test_bmi088_offsets.c` compiles the actual driver with register
  substitutes, exercises four prior-memory patterns, normal default startup,
  calibrated startup, and a subsequent non-calibrating boot. Passed with GCC
  C11/O2/Wall/Wextra, no host compiler warnings. It is a logic regression test,
  not evidence of hardware calibration quality.
- Eight `gimbal-pid-tuner/test_yaw_capture.py` tests passed, including the
  previously built C wire fixture.
- Existing `yaw-soft-limits-tests.exe` and `gimbal-axis-disable-tests.exe`
  binaries were rerun and passed; their unchanged sources were not rebuilt.
- Keil AC6 full gimbal rebuild: 0 errors, 85 warnings. Code 106064, RO 4092,
  RW 332, ZI 59028 bytes. Log: `NoMachineTemp/imu_offset_fix_rebuild.log`.
- Output: `MDK-ARM/Objects_Gimbal/AGVSentinel_Gimbal.axf`.
- SHA256: `A9A4DFE4D64464DE8A5FB97D6C13417B4AD264815F6D37BB016ECF9DA5E84FAA`.

## Next checkpoint

User-controlled flash of the gimbal build, right switch UP, reset then run,
and a stationary baseline capture. Keep both yaw software commands zero.
Then capture channel return-to-center with UP maintained; do not enable the
motors merely to inspect receiver data. Before powered identification, address
sensor validity propagation and add high-rate buffered samples of reference,
feedback, output and actual loop timing. Passive zero-command recordings
cannot identify actuator dynamics, and no MPC model has been fitted yet.
