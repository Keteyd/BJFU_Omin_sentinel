#ifndef MODULE_BIG_YAW_TUNE_H
#define MODULE_BIG_YAW_TUNE_H

#include <float.h>
#include <math.h>
#include "alg_pid.h"
#include "module_big_yaw_full.h"

/* Normal operation: small-yaw stabilization and its interlock are restored.
 * Setting this to 1 creates a passive-small-yaw isolation test build. */
#define BIG_YAW_PASSIVE_SMALL_TEST 0U

/* Normal operation: both MPC and the operator-selected PID fallback may drive big yaw. */
#define BIG_YAW_OUTPUT_INHIBIT_TEST 0U

/* Field-tuned immediate-follow preset saved on 2026-09-07. */
#define BIG_YAW_ANGLE_KP 50.0f
#define BIG_YAW_ANGLE_KI 0.001f
#define BIG_YAW_ANGLE_KD 4.0f
#define BIG_YAW_SPEED_KP 0.6f
#define BIG_YAW_SPEED_KI 0.0f
#define BIG_YAW_SPEED_KD 0.0f
/* GM6020 voltage-command full scale: +/-30000 CAN units, effort * 1000. */
#define BIG_YAW_EFFORT_MAX 30.0f
#define BIG_YAW_EFFORT_DEFAULT 30.0f
#define BIG_YAW_SPEED_FILTER_TAU_S 0.030f

static inline uint8_t BigYaw_TuneValid(float angle_kp, float speed_kp,
    float effort_max, float filter_tau_s)
{
    return (uint8_t)(isfinite(angle_kp) && angle_kp >= 0.0f &&
        isfinite(speed_kp) && speed_kp >= 0.0f &&
        isfinite(effort_max) && effort_max >= 0.0f && effort_max <= BIG_YAW_EFFORT_MAX &&
        isfinite(filter_tau_s) && filter_tau_s >= 0.0f);
}

/* Wire status: idle=0, accepted=1, unsafe=2, invalid=3, expired=4. */
static inline uint8_t BigYaw_TuneRequestStatus(uint8_t valid, uint8_t rx_safe,
    uint8_t now_safe, uint8_t outputs_off, uint32_t age_ms)
{
    if (!valid) return 3U;
    if (!rx_safe || !now_safe || !outputs_off) return 2U;
    if (age_ms > 100U) return 4U;
    return 1U;
}

static inline uint8_t BigYaw_ApplyTune(PID_PIDParamTypeDef *angle,
    PID_PIDParamTypeDef *speed, float angle_kp, float speed_kp,
    float effort_max, float filter_tau_s)
{
    if (!BigYaw_TuneValid(angle_kp, speed_kp, effort_max, filter_tau_s))
        return 0U;
    angle->kp = angle_kp;
    angle->ki = angle->kd = angle->sum_max = 0.0f;
    angle->output_max = FLT_MAX;
    speed->kp = speed_kp;
    speed->ki = speed->kd = speed->sum_max = 0.0f;
    speed->output_max = effort_max;
    return 1U;
}

static inline float BigYaw_IntegralBound(float output, float ki)
{
    double bound;
    if (ki == 0.0f) return 0.0f;
    bound = (double)output / ki;
    return bound > FLT_MAX ? FLT_MAX : (float)bound;
}

static inline uint8_t BigYaw_ApplyFullTune(PID_PIDParamTypeDef *angle,
    PID_PIDParamTypeDef *speed, const float values[BIG_YAW_FULL_COUNT])
{
    if (!BigYaw_FullValid(values)) return 0U;
    angle->kp = values[0]; angle->ki = values[4]; angle->kd = values[5];
    speed->kp = values[1]; speed->ki = values[6]; speed->kd = values[7];
    angle->output_max = FLT_MAX;
    speed->output_max = values[2];
    angle->sum_max = BigYaw_IntegralBound(FLT_MAX, angle->ki);
    speed->sum_max = BigYaw_IntegralBound(values[2], speed->ki);
    return 1U;
}

static inline uint8_t BigYaw_PidFinite(const PID_PIDTypeDef *pid)
{
    return (uint8_t)(isfinite(pid->output) && isfinite(pid->sum) &&
        isfinite(pid->err_lim) && isfinite(pid->out_fdf));
}

static inline void BigYaw_ResetPid(PID_PIDTypeDef *pid)
{
    /* Filter histories are controller state too, especially with D enabled. */
    memset(pid, 0, sizeof(*pid));
}

static inline uint8_t BigYaw_CalcPid(PID_PIDTypeDef *pid, PID_PIDParamTypeDef *param)
{
    /* Conditional integration below replaces the shared PID's back-calculation. */
    pid->err_lim = 0.0f;
    PID_CalcPID(pid, param);
    return BigYaw_PidFinite(pid);
}

/* Freeze integration that drives further into the downstream output clamp. */
static inline void BigYaw_Unwind(PID_PIDTypeDef *pid, float previous_sum,
    float error, float saturation_error)
{
    if ((error > 0.0f && saturation_error < 0.0f) ||
        (error < 0.0f && saturation_error > 0.0f)) pid->sum = previous_sum;
}

#endif
