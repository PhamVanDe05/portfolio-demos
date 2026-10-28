#ifndef PID_H
#define PID_H

/* Discrete PID controller: clamped output + conditional-integration anti-windup.
 * Hardware independent (no HAL includes) so it can be unit-tested on a host. */
typedef struct {
    float kp, ki, kd;
    float out_min, out_max;   /* output limits */
    float integral;
    float prev_meas;          /* derivative on measurement (no setpoint kick) */
    int   primed;             /* 0 until first update */
} PID_t;

void  PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max);
void  PID_Reset(PID_t *pid);
/* dt in seconds, must be > 0; returns the limited control output */
float PID_Update(PID_t *pid, float setpoint, float measurement, float dt);

#endif
