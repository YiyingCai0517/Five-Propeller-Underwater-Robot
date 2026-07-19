# STM32F407 firmware

This directory is the source form of the original STM32CubeMX project. It targets an STM32F407 and uses STM32 HAL, CMSIS-RTOS v2, and FreeRTOS.

## Important files

| Path | Purpose |
| --- | --- |
| `unify_v1_0307.ioc` | STM32CubeMX configuration |
| `Core/Src/app_robot.c` | Control, depth, command, and telemetry tasks |
| `Core/Src/pid.c` | PID implementation |
| `Core/Src/mixer.c` | Five-thruster mixer |
| `Core/Src/motor_pwm.c` | TIM2/TIM3 PWM output |
| `Core/Src/MS5837.c` | Pressure sensor driver |
| `Core/Src/analysis_data.c` | H30 IMU protocol parser |
| `Core/Src/freertos.c` | Task, queue, semaphore, and mutex creation |

## Peripheral map from source

| Interface | Use |
| --- | --- |
| USART3 + DMA/IDLE | E22 LoRa commands and telemetry |
| USART6 + DMA/IDLE | H30 IMU stream |
| I²C1 | MS5837 depth sensor |
| TIM2 CH2/3/4 | Three vertical ESC PWM channels |
| TIM3 CH3/4 | Two horizontal ESC PWM channels |

Always verify this table against the `.ioc`, generated pin definitions, PCB, and actual wiring. Some source comments and pin assignments may not describe the final board revision consistently.

## Build

Install CMake 3.22+, Ninja, and the Arm GNU Toolchain, then make sure `arm-none-eabi-gcc` is available in `PATH`.

```bash
cmake --preset Debug
cmake --build --preset Debug
```

Release build:

```bash
cmake --preset Release
cmake --build --preset Release
```

The generated ELF is placed under `build/<preset>/`. Flashing is not scripted in this repository; use the debugger/programmer that matches your hardware and verify the exact target before writing flash.

## Configuration before use

Review at minimum:

- LoRa destination addresses and channel in `Core/Src/app_robot.c` and `tools/pc/lora_pc.py`;
- IMU output protocol/rate and USART6 baud settings;
- PWM frequency, neutral, minimum, maximum, motor order, and direction;
- depth conversion, water density, atmospheric baseline, and test depth limit;
- PID gains and mixer coefficients;
- command/IMU watchdog behavior.

Do not run physical closed-loop tests until the high-priority findings in `docs/code-review.md` and the checklist in `docs/safety.md` have been addressed.
