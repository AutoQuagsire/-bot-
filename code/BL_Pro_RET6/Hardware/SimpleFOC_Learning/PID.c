#include "PID.h"
#include "FOC.h"
#include "BLDCMotor.h"
#include <math.h>

#ifndef Uq_max
#define Uq_max 6.0f
#endif

void PID_Reset(PID_t *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->error_integral = 0.0f;
    pid->last_error = 0.0f;
    pid->output = 0.0f;
}

__attribute__((optimize("O2,fast-math")))
void PID_Calculate(PID_t *pid, float target, float measure, uint8_t freeze_external)
{
    float error = target - measure;
    const float UNWIND_GAIN = 1.0f;
    float i_band = pid->I_SEP_RATIO * fabsf(target);
    float derivative;
    float p_term;
    float d_term;
    float i_term_pre;
    float u_raw_pre;
    uint8_t freeze_by_sat;
    uint8_t freeze_integral;
    uint8_t allow_integrate;
    uint8_t allow_unwind;
    float i_term;

    if (i_band < pid->I_ERR_MIN) {
        i_band = pid->I_ERR_MIN;
    }

    derivative = error - pid->last_error;
    p_term = pid->Kp * error;
    d_term = pid->Kd * derivative;

    i_term_pre = pid->Ki * pid->error_integral;
    u_raw_pre = p_term + i_term_pre + d_term;

    freeze_by_sat =
        ((u_raw_pre > pid->output_limit) && (error > 0.0f)) ||
        ((u_raw_pre < -pid->output_limit) && (error < 0.0f));

    freeze_integral = (freeze_external || freeze_by_sat);
    allow_integrate = (fabsf(error) <= i_band);
    allow_unwind = (pid->error_integral * error < 0.0f);

    if (!freeze_integral && allow_integrate) {
        pid->error_integral += error;
    } else if (allow_unwind) {
        pid->error_integral += UNWIND_GAIN * error;
    }

    i_term = pid->Ki * pid->error_integral;

    if (i_term > pid->integral_limit) {
        if (fabsf(pid->Ki) > 1e-6f) {
            pid->error_integral = pid->integral_limit / pid->Ki;
        }
    } else if (i_term < -pid->integral_limit) {
        if (fabsf(pid->Ki) > 1e-6f) {
            pid->error_integral = -pid->integral_limit / pid->Ki;
        }
    }

    i_term = pid->Ki * pid->error_integral;
    pid->output = p_term + i_term + d_term;

    if (pid->output > pid->output_limit) {
        pid->output = pid->output_limit;
    } else if (pid->output < -pid->output_limit) {
        pid->output = -pid->output_limit;
    }

    pid->last_error = error;
}

static float PID_LimitCurrentIntegralUnload(PID_t *pid, float delta_integral, float i_term_pre)
{
    float i_delta;

    if ((pid == NULL) ||
        (CURRENT_LOOP_I_UNLOAD_STEP_MAX <= 0.0f) ||
        (fabsf(pid->Ki) <= 1e-6f) ||
        (delta_integral == 0.0f) ||
        (i_term_pre == 0.0f)) {
        return delta_integral;
    }

    i_delta = pid->Ki * delta_integral;
    if ((i_term_pre * i_delta) >= 0.0f) {
        return delta_integral;
    }

    /* CurrentLoop_FFPI_V1: limit only I-term unloading.  During a target step
     * the large transient error can otherwise dump the residual steady-state
     * integral too quickly and leave Uq low when Iq reaches the new target. */
    if (i_delta > CURRENT_LOOP_I_UNLOAD_STEP_MAX) {
        i_delta = CURRENT_LOOP_I_UNLOAD_STEP_MAX;
    } else if (i_delta < -CURRENT_LOOP_I_UNLOAD_STEP_MAX) {
        i_delta = -CURRENT_LOOP_I_UNLOAD_STEP_MAX;
    }

    return i_delta / pid->Ki;
}

__attribute__((optimize("O2,fast-math")))
void PID_CalCurrent(PID_t *pid, float target, float measure, uint8_t freeze_external)
{
    float error;
    const float UNWIND_GAIN = 1.0f;
    float i_band;
    float p_term;
    float i_term_pre;
    float u_raw_pre;
    uint8_t freeze_requested;
    uint8_t limit_unload;
    uint8_t freeze_by_sat;
    uint8_t freeze_integral;
    uint8_t allow_integrate;
    uint8_t allow_unwind;
    float i_term;
    float i_delta = 0.0f;

    if (pid == NULL) {
        return;
    }

    error = target - measure;
    i_band = pid->I_SEP_RATIO * fabsf(target);
    if (i_band < pid->I_ERR_MIN) {
        i_band = pid->I_ERR_MIN;
    }

    p_term = pid->Kp * error;
    i_term_pre = pid->Ki * pid->error_integral;
    u_raw_pre = p_term + i_term_pre;
    freeze_requested = (uint8_t)((freeze_external & PID_CURRENT_FREEZE_INTEGRAL) != 0U);
    limit_unload = (uint8_t)((freeze_external & PID_CURRENT_LIMIT_I_UNLOAD) != 0U);

    freeze_by_sat =
        ((u_raw_pre > pid->output_limit) && (error > 0.0f)) ||
        ((u_raw_pre < -pid->output_limit) && (error < 0.0f));

    freeze_integral = (uint8_t)(freeze_requested || freeze_by_sat);
    allow_integrate = (fabsf(error) <= i_band);
    allow_unwind = (pid->error_integral * error < 0.0f);

    if (!freeze_integral && allow_integrate) {
        i_delta = error;
    } else if (allow_unwind) {
        i_delta = UNWIND_GAIN * error;
    }

    if ((i_delta != 0.0f) && limit_unload) {
        i_delta = PID_LimitCurrentIntegralUnload(pid, i_delta, i_term_pre);
    }
    if (i_delta != 0.0f) {
        pid->error_integral += i_delta;
    }

    i_term = pid->Ki * pid->error_integral;
    if (i_term > pid->integral_limit) {
        if (fabsf(pid->Ki) > 1e-6f) {
            pid->error_integral = pid->integral_limit / pid->Ki;
        }
    } else if (i_term < -pid->integral_limit) {
        if (fabsf(pid->Ki) > 1e-6f) {
            pid->error_integral = -pid->integral_limit / pid->Ki;
        }
    }

    i_term = pid->Ki * pid->error_integral;
    pid->output = p_term + i_term;

    if (pid->output > pid->output_limit) {
        pid->output = pid->output_limit;
    } else if (pid->output < -pid->output_limit) {
        pid->output = -pid->output_limit;
    }

    pid->last_error = error;
}

void PID_CalculateTest(PID_t *pid, float target, float measure)
{
    float error = target - measure;
    float i_output;
    float derivative;

    pid->error_integral += error;

    i_output = pid->Ki * pid->error_integral;
    if (i_output > pid->integral_limit) {
        if (fabsf(pid->Ki) > 1e-6f) {
            pid->error_integral = pid->integral_limit / pid->Ki;
        }
    } else if (i_output < -pid->integral_limit) {
        if (fabsf(pid->Ki) > 1e-6f) {
            pid->error_integral = -pid->integral_limit / pid->Ki;
        }
    }

    derivative = error - pid->last_error;
    pid->last_error = error;

    pid->output = (pid->Kp * error) +
                  (pid->Ki * pid->error_integral) +
                  (pid->Kd * derivative);
}

void PID_ParameterInitEx(PID_t *pid, float kp, float ki, float kd, float integral_limit,
                         float output_limit, float i_err_min, float i_sep_ratio)
{
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->I_ERR_MIN = i_err_min;
    pid->I_SEP_RATIO = i_sep_ratio;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    PID_Reset(pid);
}

void PID_ParameterInit(PID_t *pid, float kp, float ki, float kd, float integral_limit)
{
    float output_limit = (pid->output_limit > 0.0f) ? pid->output_limit : Uq_max;
    float i_err_min = (pid->I_ERR_MIN > 0.0f) ? pid->I_ERR_MIN : 0.05f;
    float i_sep_ratio = (pid->I_SEP_RATIO > 0.0f) ? pid->I_SEP_RATIO : 0.5f;

    PID_ParameterInitEx(pid, kp, ki, kd, integral_limit,
                        output_limit, i_err_min, i_sep_ratio);
}
