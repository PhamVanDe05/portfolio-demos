# STM32 PID Speed Controller (C, HAL style)

A hardware-independent PID module (`pid.c` / `pid.h`) with output limiting and conditional-integration
anti-windup, derivative on measurement (no setpoint kick), and a guard against `dt <= 0`.
`main.c` is a sample STM32 HAL application: TIM2 reads a quadrature encoder, a 1 kHz TIM6 interrupt
computes RPM, runs the PID, and writes the duty cycle to TIM3 PWM. `main.c` needs the CubeMX-generated
peripheral init and an ARM toolchain, so it is not built here; the PID module is tested on the host.

## Run the host tests
```
make test        # gcc + ./tests/test_pid  ->  "all PID tests passed"
```

Tests cover: proportional response, output clamping, anti-windup recovery, closed-loop convergence on a
first-order plant, invalid `dt`, and reset.

## Files
| File | Purpose |
|---|---|
| `pid.h`, `pid.c` | PID controller (portable C99) |
| `main.c` | Example encoder -> PID -> PWM loop (STM32 HAL) |
| `tests/test_pid.c` | Host unit tests |
