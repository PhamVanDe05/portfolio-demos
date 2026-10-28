/* Sample STM32 HAL application: closed-loop motor speed control.
 * TIM2 = quadrature encoder, TIM3 CH1 = PWM, TIM6 = 1 kHz control tick.
 * Peripheral init (CubeMX-generated MX_*_Init) is omitted; this file is not built on the host. */
#include "stm32f4xx_hal.h"
#include "pid.h"

#define CTRL_DT_S      0.001f
#define ENCODER_CPR    1320.0f      /* counts per output-shaft revolution (x4 decoding) */
#define PWM_PERIOD     999U         /* TIM3 ARR */
#define TARGET_RPM     120.0f

extern TIM_HandleTypeDef htim2, htim3, htim6;

static PID_t speed_pid;
static volatile uint32_t last_count;

/* Called every 1 ms from the TIM6 update interrupt. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6) {
        return;
    }
    uint32_t count = __HAL_TIM_GET_COUNTER(&htim2);
    int16_t delta = (int16_t)(count - last_count);   /* 16-bit wrap-safe */
    last_count = count;

    float rpm = (float)delta / ENCODER_CPR / CTRL_DT_S * 60.0f;
    float duty = PID_Update(&speed_pid, TARGET_RPM, rpm, CTRL_DT_S);   /* 0..1 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(duty * (float)PWM_PERIOD));
}

int main(void)
{
    HAL_Init();
    /* SystemClock_Config(); MX_GPIO_Init(); MX_TIM2_Init(); MX_TIM3_Init(); MX_TIM6_Init(); */

    PID_Init(&speed_pid, 0.004f, 0.02f, 0.0f, 0.0f, 1.0f);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim6);

    for (;;) {
        /* background tasks: UART telemetry, setpoint commands, ... */
    }
}
