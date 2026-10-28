#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../pid.h"

/* First-order plant: y' = (u*gain - y)/tau, Euler. */
static float plant(float y, float u, float dt) { return y + dt * (u * 10.0f - y) / 0.2f; }

static void test_p_only(void)
{
    PID_t p; PID_Init(&p, 2.0f, 0, 0, -100, 100);
    assert(fabsf(PID_Update(&p, 5.0f, 3.0f, 0.01f) - 4.0f) < 1e-5f);
}

static void test_output_clamped(void)
{
    PID_t p; PID_Init(&p, 100.0f, 0, 0, -1.0f, 1.0f);
    assert(PID_Update(&p, 10.0f, 0.0f, 0.01f) == 1.0f);
    assert(PID_Update(&p, -10.0f, 0.0f, 0.01f) == -1.0f);
}

static void test_antiwindup(void)
{
    PID_t p; PID_Init(&p, 1.0f, 10.0f, 0, -1.0f, 1.0f);
    for (int i = 0; i < 10000; i++) PID_Update(&p, 100.0f, 0.0f, 0.001f);  /* long saturation */
    assert(p.integral < 1.0f);                       /* did not wind up */
    int steps = 0;                                   /* recovers right after setpoint flips */
    while (PID_Update(&p, -1.0f, 0.0f, 0.001f) > 0.0f && steps < 1000) steps++;
    assert(steps < 50);
}

static void test_closed_loop_converges(void)
{
    PID_t p; PID_Init(&p, 0.3f, 3.0f, 0, 0.0f, 1.0f);
    float y = 0.0f;
    for (int i = 0; i < 5000; i++) y = plant(y, PID_Update(&p, 5.0f, y, 0.001f), 0.001f);
    assert(fabsf(y - 5.0f) < 0.05f);
}

static void test_bad_dt_and_reset(void)
{
    PID_t p; PID_Init(&p, 1.0f, 1.0f, 1.0f, -1, 1);
    assert(PID_Update(&p, 1.0f, 0.0f, 0.0f) == 0.0f);
    PID_Update(&p, 1.0f, 0.0f, 0.01f);
    PID_Reset(&p);
    assert(p.integral == 0.0f && p.primed == 0);
}

int main(void)
{
    test_p_only(); test_output_clamped(); test_antiwindup();
    test_closed_loop_converges(); test_bad_dt_and_reset();
    puts("all PID tests passed");
    return 0;
}
