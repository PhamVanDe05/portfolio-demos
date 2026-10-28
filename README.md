# Robotics & Embedded Portfolio Demos

Four small, runnable projects. Each folder has its own README with goal, run command and result image.

| Project | Stack | What it shows |
|---|---|---|
| [pid-dc-motor-sim](pid-dc-motor-sim) | Python, numpy, matplotlib | DC motor model + PID speed control, 3 gain sets compared |
| [follow-me-kalman-sim](follow-me-kalman-sim) | Python | Kalman filter (constant velocity) + drone following a noisy 2D target |
| [stm32-pid-firmware](stm32-pid-firmware) | C, STM32 HAL | PID with anti-windup, encoder/PWM sample, host unit tests |
| [gesture-mlp-demo](gesture-mlp-demo) | Python, scikit-learn | MLP gesture classifier on synthetic 21-point hand landmarks |

## Setup & tests
```
python3 -m venv .venv && .venv/bin/pip install numpy matplotlib scikit-learn pytest
for d in pid-dc-motor-sim follow-me-kalman-sim gesture-mlp-demo; do (cd $d && ../.venv/bin/python -m pytest -q); done
make -C stm32-pid-firmware test
```
