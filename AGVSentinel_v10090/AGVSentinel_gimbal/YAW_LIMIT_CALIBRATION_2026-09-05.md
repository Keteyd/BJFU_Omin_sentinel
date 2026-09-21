# Small-yaw mechanical landmarks

Status: measurements below are preserved; the user subsequently reported
the small-yaw soft limits function correctly on the vehicle. The guard uses
the visual-center reference and asymmetric bounds with a 3-degree inward
margin. The later demand-based big-yaw policy is documented separately in
`YAW_COORDINATION.md`; its default enable request is now 1. The historical
soft-limit-only build and tests below do not validate that new policy.

## Hardware and method

- User confirms parallel, non-coaxial yaw axes; big yaw has a slip ring and
  can rotate continuously. Small yaw has two mechanical stops.
- Firmware maps small yaw to a GM6020 encoder on CAN2, feedback ID 0x206.
- Read raw encoder counts through monitor-only USART1 packet 0x2A, version 1.
  Resolution: 8192 counts per encoder revolution.
- User positioned the joint by hand with the remote right switch upper SAFE.
  Each capture contains 60 samples over approximately 3 seconds.
- Every saved sample passed output-inactive, valid-feedback, <=100 ms
  feedback-age and monitor-protocol checks. Output status is a software
  report, not a physical torque measurement.
- Left/right are user labels relative to the vehicle facing forward.
- User subsequently confirmed that the first landmark is the true visual
  center of the assembly. Left and right mechanical travel are asymmetric
  about this reference; it was not an inaccurately positioned midpoint.
- User confirmed direct GM6020 drive without a transmission: encoder
  angular travel can be used directly as small-yaw joint angular travel.

## Measurements

| Landmark | UTC capture time | Small raw median | Encoder degrees | Small span, degrees | Big raw median |
| --- | --- | ---: | ---: | ---: | ---: |
| User-confirmed visual center | 2026-09-05 08:35:09 | 6816 | 299.53125 | 0.00000 | 8165 |
| Left stop | 2026-09-05 08:36:53 | 7546 | 331.61133 | 0.08789 | 8173 |
| Right stop | 2026-09-05 08:38:12 | 5458 | 239.85352 | 0.00000 | 14 |

The interior reference lies inside the increasing right-to-left arc. Thus
the measured admissible encoder interval is [5458, 7546], without a zero
crossing. The complementary circular arc is not the measured motion range.

- Encoder travel: 2088 counts = 91.7578125 degrees.
- Visual-center zero reference: 6816 counts = 299.53125 encoder degrees.
- Relative to visual center: right -1358 counts (-59.677734375 encoder
  degrees), left +730 counts (+32.080078125 encoder degrees).
- Arithmetic travel midpoint: 6502 counts = 285.732421875 degrees. This
  mathematical midpoint is NOT the assembly's visual center or zero.
- Future coordination should retain the user-confirmed visual-center
  reference and manage left/right available travel separately. Any choice
  to bias the operating position toward equal travel is a separate control
  policy, not a correction to the measured zero reference.
- Big yaw moved approximately +1.80176 encoder degrees between the first
  and last captures and crossed its encoder zero. These small-yaw landmarks
  use the small motor's own encoder, not IMU/world heading or big-yaw angle.

## Evidence

Full per-sample feedback and metadata are saved in the workspace:

- `NoMachineTemp/yaw-middle-20260905-01.json`
- `NoMachineTemp/yaw-left-20260905-01.json`
- `NoMachineTemp/yaw-right-20260905-01.json`

Remote copies are in `/home/nuc11--02/gimbal-pid-tuner/` on the vehicle PC.
The recording utility is `gimbal-pid-tuner/yaw_limits_cli.py`; all four
offline unit tests passed before capture. The monitor firmware AC6 build
had 0 errors and 14 warnings (incremental build).

## Before applying limits

These are single-session contact measurements, not repeatability or reboot
validation. Verify the existing IMU-to-encoder sign convention during the
first low-speed soft-limit test. User confirmation establishes the
visual-center reference, not a precision optical boresight calibration.
Repeat endpoint readings if necessary
to establish contact repeatability/backlash, then choose inward soft-limit
margins and predictive braking based on real speed and stopping behavior.
Do not command the measured hard stops as ordinary tracking targets.

## Initial soft-limit implementation

`Inc/Modules/module_yaw_limits.h` owns the calibrated zero/limits and the
pure joint-space guard functions. Soft bounds are -56.677734375 and
+29.080078125 degrees relative to visual center. Target error is bounded
by remaining joint travel and outward joint-speed requests taper over
the last 8 degrees before each soft boundary. At/beyond that boundary,
outward effort requests are suppressed but inward/braking effort is allowed.
Outside soft bounds, enabling captures the current pose without an automatic
move back into the interval. Inward operator input remains possible.

The module uses the accepted `GimbalYaw_DiagSmallDirection` (+1 currently)
as the IMU-to-encoder sign; GM6020 positive effort is assumed to increase
its encoder angle. Encoder feedback older than 50 ms or a position outside
the measured hard interval by over 1 degree inhibits yaw output. These
guards do not establish a certified stopping distance or prevent a person
or moving base from forcing the joint into a hard stop. First validate
direction and each boundary at low speed, with big yaw still disabled.
Do not use high-speed vision tracking as the first validation test.

Validation of the initial soft-limit-only revision (before coordination):

- GCC C11 `-Wall -Wextra -Werror`: `Tests/test_yaw_soft_limits.c` passed.
- Existing Pitch/CRC host regression executable also passed.
- Read-only CLI: 5 offline unit tests passed, including packet 0x2B decoding.
- AC6 V6.22 full gimbal rebuild: 0 errors, 85 warnings; log in
  `NoMachineTemp/yaw_soft_limits_rebuild.log`.
- Gimbal AXF SHA256:
  `1E4DD4213A641A176E3C36E2AFFCA584915EDD43F701C5B67DF814ED654304B7`.
- No firmware download, reset, active serial command or physical motion test
  was performed during implementation. Low-speed hardware validation remains.
