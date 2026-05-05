# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Flash

- **IDE**: Keil MDK-ARM 5 (project file: `MDK-ARM/WHEELTEC.uvprojx`)
- **Target MCU**: STM32F103RB (Cortex-M3, 72MHz)
- **HAL**: STM32F1xx HAL Driver
- **Build**: Open `.uvprojx` in Keil uVision, press F7 (Build) or F8 (Build+Download)
- **STM32CubeMX**: `WHEELTEC.ioc` — regenerate `Core/` when peripheral config changes, but preserve user code between `USER CODE BEGIN/END` guards
- No CLI build tooling, no test framework, no linting setup

## Architecture

### Control Pipeline (200Hz)

`TIM1` interrupt → `HAL_TIM_PeriodElapsedCallback` in [BALANCE/CONTROL/control.c](BALANCE/CONTROL/control.c:73) runs every 5ms:

1. Read sensors: `Read_Encoder()` (position), `Get_Adc_Average()` (pendulum angle), `Get_D_Angle_Balance()` (angular velocity via mean filter)
2. Run control law (one of two modes, toggled by long-press KEY2):
   - **PD mode** (`auto_run == 0`): Cascade PD — inner angle loop (200Hz) + outer position loop (40Hz). Gains: `Balance_KP/KD`, `Position_KP/KD`.
   - **UAS-SMC mode** (`auto_run == 1`): Uncertainty-adaptive sliding mode control with dual sliding surfaces (position + angle), 4-quadrant switching gains, low-pass filtered velocities, and integral position compensation.
3. Apply PWM limit (`Xianfu_Pwm`, max ±6900), set motor via `Set_Pwm()` → BIN1/BIN2 direction + PWMB duty

### Key Files

| File | Role |
|------|------|
| [BALANCE/CONTROL/control.c](BALANCE/CONTROL/control.c) | All control: ISR callback, PD, SMC, auto-swing-up, key handling, PWM output |
| [BALANCE/CONTROL/control.h](BALANCE/CONTROL/control.h) | System constants (`ZHONGZHI=3100`, `TS=0.005`, `K=0.0000982`), state-space matrices, function declarations |
| [BALANCE/show/show.c](BALANCE/show/show.c) | OLED display + DataScope serial output (channels 1–6 sent via USART1 every main loop iteration) |
| [BALANCE/CHECK/check.c](BALANCE/CHECK/check.c) | Multi-page OLED calibration wizard for sensor zeroing |
| [Core/Src/main.c](Core/Src/main.c) | Entry point: `HAL_Init`, clock config (HSE+PLL→72MHz), peripheral init, then infinite loop calling `DataScope()` + `Tips()` every 50ms |
| [Core/Inc/main.h](Core/Inc/main.h) | Project-wide type aliases (`u8/u16/u32/s16/s32`), bit-band GPIO macros (51-style `PAout(n)`/`PAin(n)`), all includes |

### Peripheral Usage

- **TIM1**: 5ms period interrupt (200Hz control loop) + PWM generation on CH4 (PB1)
- **TIM3**: Encoder interface (quadrature encoder on PA6/PA7)
- **TIM4**: Auxiliary timing
- **ADC1**: Pendulum angle potentiometer (PA3) + battery voltage
- **USART1**: DataScope virtual oscilloscope protocol (115200 baud assumed)
- **GPIO**: PB12 (BIN2), PB13 (BIN1) = motor direction; PA4 = LED; PA5/PA7 = user keys; PA11/PA12 = PID adjust keys

### DataScope Protocol

`DataScope_DP` sends float arrays over USART1 in frame format: `$` header + 4-byte float per channel + position markers. Used for real-time debugging with the DataScope PC上位机. Channels assigned in [show.c:122-131](BALANCE/show/show.c#L122-L131): 1=Angle, 2=Encoder, 3=theta_d, 4=pos_d, 5=u_d, 6=N.

### SMC Control Law Details

The UAS-SMC in `UAS_SMC_Control()` ([control.c:152-248](BALANCE/CONTROL/control.c#L152-L248)):
- **State matrices**: `A[4]` = {A22, A23, A42, A43}, `B[2]` = {B21, B41} — model parameters from system identification
- **Sliding surfaces**: s1 (position) = `e_pos + 1.5*e_pos_dot` alternating with `e_pos + (1/6)*e_pos_dot`; s2 (angle) = same structure
- **4-quadrant switching**: `k11/k12/k21/k22` arrays — gains change based on s1/s2 sign quadrants
- **Robustness term**: `N = k1*|pos_error| + k2*|theta_error|` (k1=100, k2=1200 in current tuning)
- **Output**: `u_real = U2PWM*u_d + 1200*pos_integral`, negated before return

## Code Conventions

- Comments are in **Chinese** (GB2312/GBK encoding). File headers reference WHEELTEC brand.
- Type aliases from `main.h`: `u8/u16/u32` (unsigned), `s16/s32` (signed), `vu8/vu16/vu32` (volatile)
- GPIO uses bit-band macros for 8051-style access: `PBout(1)=1` instead of `HAL_GPIO_WritePin`
- STM32CubeMX user code lives between `/* USER CODE BEGIN/END */` markers — these blocks are preserved on CubeMX regeneration
- Motor PWM range: ±7200 (hardware), software-limited to ±6900
