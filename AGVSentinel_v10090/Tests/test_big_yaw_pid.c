#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "module_big_yaw_tune.h"

static void near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.0001f);
}

static float run(PID_PIDTypeDef *angle, PID_PIDTypeDef *speed,
    PID_PIDParamTypeDef *ap, PID_PIDParamTypeDef *sp,
    float error, float feedback_rpm, float feedforward_rpm)
{
    PID_SetPIDRef(angle, error);
    PID_SetPIDFdb(angle, 0);
    PID_CalcPID(angle, ap);
    PID_SetPIDRef(speed, PID_GetPIDOutput(angle) + feedforward_rpm);
    PID_SetPIDFdb(speed, feedback_rpm);
    PID_CalcPID(speed, sp);
    return PID_GetPIDOutput(speed);
}

int main(void)
{
    PID_PIDTypeDef angle = {0}, speed = {0};
    PID_PIDParamTypeDef ap = {0}, sp = {0}, saved_ap, saved_sp;
    const float defaults[8] = {BIG_YAW_ANGLE_KP, BIG_YAW_SPEED_KP,
        BIG_YAW_EFFORT_DEFAULT, BIG_YAW_SPEED_FILTER_TAU_S,
        BIG_YAW_ANGLE_KI, BIG_YAW_ANGLE_KD, BIG_YAW_SPEED_KI, BIG_YAW_SPEED_KD};
    assert(BIG_YAW_PASSIVE_SMALL_TEST == 0U);
    assert(BIG_YAW_OUTPUT_INHIBIT_TEST == 0U);
    assert(BigYaw_ApplyFullTune(&ap, &sp, defaults));
    near(ap.kp, 50.0f);
    near(ap.ki, 0.001f);
    near(ap.kd, 4.0f);
    near(sp.kp, 0.6f);
    near(sp.ki, 0.0f);
    near(sp.kd, 0.0f);
    near(sp.output_max, 30.0f);
    near(BIG_YAW_SPEED_FILTER_TAU_S, 0.03f);
    near(run(&angle, &speed, &ap, &sp, 0.5f, 0, 0), 16.2003f);
    assert(BigYaw_ApplyTune(&ap, &sp, 4, 1.2f, 8, 0.008f));
    assert(ap.output_max == FLT_MAX);
    near(run(&angle, &speed, &ap, &sp, 0, 0, 0), 0);
    assert(BigYaw_TuneRequestStatus(1, 1, 1, 1, 100) == 1);
    assert(BigYaw_TuneRequestStatus(1, 1, 1, 1, 101) == 4);
    assert(BigYaw_TuneRequestStatus(0, 1, 1, 1, 0) == 3);
    assert(BigYaw_TuneRequestStatus(1, 0, 1, 1, 0) == 2);
    assert(BigYaw_TuneRequestStatus(1, 1, 0, 1, 0) == 2);
    assert(BigYaw_TuneRequestStatus(1, 1, 1, 0, 0) == 2);
    near(run(&angle, &speed, &ap, &sp, 0.5f, 0, 0), 2.4f);
    near(run(&angle, &speed, &ap, &sp, -0.5f, 0, 0), -2.4f);
    near(run(&angle, &speed, &ap, &sp, 0, 1, 0), -1.2f);
    near(run(&angle, &speed, &ap, &sp, 0, -1, 0), 1.2f);
    /* Error beyond the former 4-degree clip remains visible to the angle loop. */
    near(run(&angle, &speed, &ap, &sp, 6, 0, 0), 8);
    near(angle.output, 24);
    assert(speed.err_lim < 0);
    near(run(&angle, &speed, &ap, &sp, -6, 0, 0), -8);
    assert(speed.err_lim > 0);
    near(run(&angle, &speed, &ap, &sp, 0, 0, 5), 6);
    near(speed.err_lim, 0);
    for (int i = 0; i < 10000; ++i)
        near(run(&angle, &speed, &ap, &sp, 90, 0, 0), 8);
    near(angle.sum, 0);
    near(speed.sum, 0);
    near(run(&angle, &speed, &ap, &sp, -90, 0, 0), -8);
    near(run(&angle, &speed, &ap, &sp, 0, 0, 0), 0);

    /* Rejected Watch values leave both parameter objects unchanged. */
    saved_ap = ap;
    saved_sp = sp;
    const float invalid[][4] = {
        {NAN, 1.2f, 8, 0.008f}, {4, INFINITY, 8, 0.008f},
        {4, 1.2f, NAN, 0.008f}, {4, 1.2f, 8, NAN},
        {-1, 1.2f, 8, 0.008f}, {INFINITY, 1.2f, 8, 0.008f},
        {4, -1, 8, 0.008f}, {4, INFINITY, 8, 0.008f},
        {4, 1.2f, -1, 0.008f}, {4, 1.2f, 30.001f, 0.008f},
        {4, 1.2f, 8, -1}, {4, 1.2f, 8, INFINITY}
    };
    for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        assert(!BigYaw_ApplyTune(&ap, &sp, invalid[i][0], invalid[i][1],
                                invalid[i][2], invalid[i][3]));
        assert(memcmp(&ap, &saved_ap, sizeof(ap)) == 0);
        assert(memcmp(&sp, &saved_sp, sizeof(sp)) == 0);
    }
    assert(BigYaw_ApplyTune(&ap, &sp, 4, 1.2f, 0, 0));
    near(run(&angle, &speed, &ap, &sp, 90, 0, 0), 0);
    assert(BigYaw_ApplyTune(&ap, &sp, 4, 0.7f, 12, 0.03f));
    near(run(&angle, &speed, &ap, &sp, 90, 0, 0), 12);
    near(run(&angle, &speed, &ap, &sp, -90, 0, 0), -12);
    assert(BigYaw_ApplyTune(&ap, &sp, 4, 0.7f, 30, 0.03f));
    near(run(&angle, &speed, &ap, &sp, 90, 0, 0), 30);
    assert((int16_t)(speed.output * 1000.0f) == 30000);
    near(run(&angle, &speed, &ap, &sp, -90, 0, 0), -30);
    assert((int16_t)(speed.output * 1000.0f) == -30000);
    assert(BigYaw_ApplyTune(&ap, &sp, 4, 1.2f, 8, 0.008f));
    PID_ClearPID(&angle);
    PID_ClearPID(&speed);
    near(run(&angle, &speed, &ap, &sp, 0, 0, 0), 0);
    {
        float full[8] = {100000.0f, 100000.0f, 30, 1000, 100, 100, 100, 100};
        assert(BigYaw_ApplyFullTune(&ap, &sp, full));
        near(ap.kp, 100000); near(ap.ki, 100); near(ap.kd, 100);
        near(sp.kp, 100000); near(sp.ki, 100); near(sp.kd, 100);
        full[4] = NAN; assert(!BigYaw_ApplyFullTune(&ap, &sp, full));
        full[4] = -1; assert(!BigYaw_ApplyFullTune(&ap, &sp, full));
        full[4] = 0; full[0] = 0; full[1] = 1; full[5] = 0.3f; full[6] = full[7] = 0;
        assert(BigYaw_ApplyFullTune(&ap, &sp, full));
        PID_ClearPID(&angle); PID_ClearPID(&speed);
        near(run(&angle, &speed, &ap, &sp, 1, 0, 0), 0.3f);
        near(run(&angle, &speed, &ap, &sp, 1, 0, 0), 0);
        full[4] = 0.25f; full[5] = 0;
        assert(BigYaw_ApplyFullTune(&ap, &sp, full));
        PID_ClearPID(&angle); PID_ClearPID(&speed);
        near(run(&angle, &speed, &ap, &sp, 1, 0, 0), 0.25f);
        near(run(&angle, &speed, &ap, &sp, 1, 0, 0), 0.5f);
        full[0] = 1; full[1] = 0; full[4] = 0; full[6] = 0.2f;
        assert(BigYaw_ApplyFullTune(&ap, &sp, full));
        PID_ClearPID(&angle); PID_ClearPID(&speed);
        near(run(&angle, &speed, &ap, &sp, 1, 0, 0), 0.2f);
        near(run(&angle, &speed, &ap, &sp, 1, 0, 0), 0.4f);
        full[6] = 0; full[7] = 0.2f;
        assert(BigYaw_ApplyFullTune(&ap, &sp, full));
        PID_ClearPID(&angle); PID_ClearPID(&speed);
        near(run(&angle, &speed, &ap, &sp, 1, 0, 0), 0.2f);
        near(run(&angle, &speed, &ap, &sp, 1, 0, 0), 0);
        full[0] = 4; full[1] = .7f; full[2] = 4; full[4] = full[6] = .1f; full[7] = 0;
        assert(BigYaw_ApplyFullTune(&ap, &sp, full));
        PID_ClearPID(&angle); PID_ClearPID(&speed);
        for (int i = 0; i < 1000; ++i) {
            float a = angle.sum, s = speed.sum;
            PID_SetPIDRef(&angle, 90);
            assert(BigYaw_CalcPid(&angle, &ap));
            PID_SetPIDRef(&speed, angle.output);
            assert(BigYaw_CalcPid(&speed, &sp));
            BigYaw_Unwind(&angle, a, angle.err[0], speed.err_lim);
            BigYaw_Unwind(&speed, s, speed.err[0], speed.err_lim);
            near(angle.sum, 0); near(speed.sum, 0);
        }
        PID_SetPIDRef(&angle, 0); assert(BigYaw_CalcPid(&angle, &ap));
        PID_SetPIDRef(&speed, angle.output); assert(BigYaw_CalcPid(&speed, &sp));
        near(speed.output, 0);
        ap.kp = FLT_MAX; PID_SetPIDRef(&angle, 90);
        assert(!BigYaw_CalcPid(&angle, &ap));
        angle.d_fil.filted_last_val = NAN;
        angle.delta_fil.filted_last_val = 7;
        angle.kf1_fil.filted_val = 8;
        angle.kf2_fil.filted_last_val = 9;
        BigYaw_ResetPid(&angle);
        assert(BigYaw_PidFinite(&angle));
        near(angle.d_fil.filted_last_val, 0);
        near(angle.delta_fil.filted_last_val, 0);
        near(angle.kf1_fil.filted_val, 0);
        near(angle.kf2_fil.filted_last_val, 0);
        near(angle.err[0], 0);
        near(ap.kp, FLT_MAX);
    }
    puts("PASS: actual PID arithmetic, signs, uncapped angle output, saturation, reversal and tune validation");
    return 0;
}
