#include "pid.h"

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    PID_Reset(pid);
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_meas = 0.0f;
    pid->primed = 0;
}

float PID_Update(PID_t *pid, float setpoint, float measurement, float dt)
{
    if (dt <= 0.0f) {
        return 0.0f;
    }
    float err = setpoint - measurement;
    float d = pid->primed ? -(measurement - pid->prev_meas) / dt : 0.0f;
    float raw = pid->kp * err + pid->ki * pid->integral + pid->kd * d;
    float out = clampf(raw, pid->out_min, pid->out_max);

    /* anti-windup: integrate only if not saturated, or if the error pulls back out of saturation */
    if (out == raw || (raw > pid->out_max && err < 0.0f) || (raw < pid->out_min && err > 0.0f)) {
        pid->integral += err * dt;
    }
    pid->prev_meas = measurement;
    pid->primed = 1;
    return out;
}
