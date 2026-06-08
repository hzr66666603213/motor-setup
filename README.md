# ODrive v3.6 Axis0 FOC 学习固件框架

本项目面向你的真实硬件：

- ODrive v3.6 双轴三相 FOC 控制板。
- STM32F405RG。
- TI DRV8301 类三相栅极驱动。
- Axis0 接 2804 外转子无刷电机。
- MT6701 磁编码器第一阶段使用 ABZ 增量输出，接 ODrive Encoder0 A/B/Z。
- STLINK-V3MINIE、USB 数据线、直流限流电源。

第一阶段只启用 Axis0，Axis1 资源保留但默认不参与控制。2804 的 `pole_pairs` 默认先填 7，但不同厂家可能不同，必须实测确认。

## 项目目标

这是一套学习和二次开发用的 FOC 固件框架，不直接复制 ODrive 官方固件代码。目标是让 2804 小电机在低压、低电流、可观察、可逐步排故的条件下跑通：

1. TIM1 互补 PWM。
2. DRV8301 初始化和 nFAULT 保护。
3. 两相电流采样，第三相用 `ic=-ia-ib` 推算。
4. VBUS ADC 采样。
5. MT6701 ABZ / TIM3 Encoder Mode 计数。
6. 开环 SVPWM。
7. 编码器方向判断。
8. 电角度零位校准。
9. Id/Iq 电流环。
10. 速度环。
11. 位置环。

## 目录结构

- `include/`：公共头文件。
- `src/app/`：Axis0 应用层、状态机、校准、console。
- `src/board/`：ODrive v3.6 板级安全抽象。
- `src/control/`：电流环、速度环、位置环。
- `src/drivers/`：DRV8301、MT6701 ABZ、电流采样转换。
- `src/foc/`：Clarke、Park、反 Park、SVPWM、限幅、滤波。
- `src/hal/mock/`：PC/Simulink/mock 后端。
- `src/hal/stm32f405/`：STM32F405 真实 HAL 后端。
- `src/sim/`：Simulink / PC 仿真入口。
- `tests/`：PC 单元测试。

## HAL 后端选择

不要把 mock 和真实 STM32 后端同时编译。

PC 测试和 Simulink 使用：

- `src/hal/mock/*.c`

ODrive v3.6 上板使用：

- `src/hal/stm32f405/hal_pwm_stm32f405.c`
- `src/hal/stm32f405/hal_adc_stm32f405.c`
- `src/hal/stm32f405/hal_gpio_stm32f405.c`
- `src/hal/stm32f405/hal_spi_stm32f405.c`
- `src/hal/stm32f405/hal_encoder_stm32f405.c`

早期兼容文件 `src/hal/hal_pwm.c`、`hal_adc.c`、`hal_gpio.c`、`hal_spi.c` 仍是 mock。真实固件工程不要编译这些文件，否则会和 `src/hal/stm32f405/*.c` 重复定义。

## STM32CubeMX / CubeIDE 外设要求

CubeMX 工程需要提供这些句柄：

- `TIM_HandleTypeDef htim1`：Axis0 三相互补 PWM。
- `TIM_HandleTypeDef htim3`：Encoder0 ABZ 的 A/B 计数。
- `ADC_HandleTypeDef hadc1`：VBUS、温度等采样。
- `ADC_HandleTypeDef hadc2`：Axis0 两相电流采样。
- `SPI_HandleTypeDef hspi3`：DRV8301 SPI。

当前真实后端默认资源：

- TIM1 PWM：PA8/PA9/PA10 + PB13/PB14/PB15。
- TIM3 Encoder0 AB：PB4/PB5。
- EN_GATE：PB12，M0/M1 共用。
- nFAULT：PD2，M0/M1 共用，低有效。
- SPI3：PC10/PC11/PC12。
- DRV0 CS：PC13。
- DRV1 CS：PC14。

VBUS 比例、电流采样比例、ADC rank、采样触发点必须按你的 ODrive v3.6 实物和 CubeMX 配置核对。24V/56V 版本、分压电阻批次、DRV8301 gain、shunt 阻值都会影响实际比例。

## 控制频率

- PWM / 电流环：20 kHz。
- 速度环：1 kHz。
- 位置环：500 Hz 或 1 kHz。
- 状态机、保护、通信：100 Hz 到 1 kHz，或后台轮询。

PWM ISR 中禁止 `malloc`、`printf`、`delay`、阻塞式 SPI/CAN/USB/UART。

## 状态机

主要运行状态：

- `boot`：板级安全初始化，PWM off，EN_GATE off。
- `idle`：等待命令，功率级关闭。
- `current_offset_calibration`：电流采样零偏校准。
- `motor_calibration`：低压相电阻/相电感估算。
- `encoder_calibration`：编码器方向和电角度 offset 校准。
- `ready`：三类校准都完成，允许进入闭环。
- `closed_loop`：运行 FOC。
- `fault`：立即关闭 PWM 和 EN_GATE。

Bring-up 测试状态：

- `pwm_test`：EN_GATE 保持低，只输出 TIM1 PWM 波形。
- `encoder_test`：功率级关闭，只观察 TIM3 ABZ count。
- `adc_offset_test`：功率级关闭，观察电流 ADC 零偏。
- `open_loop_voltage_test`：功率级打开，但电压限制在 0.5V 以下。

没有完成 `current_offset_valid`、`motor_calibrated`、`encoder_calibrated` 三个条件时，不允许进入 `closed_loop`。

## 文本命令示例

```text
get vbus
get ia
get ib
get ic
get id
get iq
get angle
get velocity
get fault
get state
set current_limit 1.0
set voltage_limit 3.0
set pole_pairs 7
set encoder_cpr 4096
set control_mode torque
set control_mode velocity
set control_mode position
set input_torque 0.02
set input_velocity 5.0
set input_position 1.57
request_state idle
request_state pwm_test
request_state encoder_test
request_state adc_offset_test
request_state open_loop_voltage_test
request_state current_offset_calibration
request_state motor_calibration
request_state encoder_calibration
request_state closed_loop
clear_faults
save_config
reboot
```

## 编译、烧录、连接

1. 安装 STM32CubeIDE、STM32CubeProgrammer、ST-LINK Driver。
2. 用 CubeMX 建 STM32F405RG 工程，配置 TIM1、TIM3、ADC1、ADC2、SPI3、GPIO、USB CDC 或 UART。
3. 把本工程 `include/` 加入 include path。
4. 把 `src/foc`、`src/control`、`src/drivers`、`src/board`、`src/app` 和 `src/hal/stm32f405` 加入工程。
5. 不要加入 `src/hal/mock` 和早期兼容 mock 文件。
6. 用 STLINK-V3MINIE 连接 SWDIO、SWCLK、GND、NRST、VTref。ODrive 由直流限流电源单独供电，VTref 只作为电平参考。
7. CubeIDE 编译后用 Debug 或 STM32CubeProgrammer 烧录。
8. 用 USB CDC 或 UART 打开串口调试终端。

## 第一次上电顺序

1. 不接电机，不接主电源，只接 USB，确认设备能枚举。
2. 接 STLINK，确认能识别 STM32F405。
3. 接低压限流电源，建议 12V，电流限制 0.3A 到 0.5A。
4. 上电后确认默认状态为 `idle`，PWM 关闭，EN_GATE 关闭。
5. `get vbus`，确认母线电压读数接近电源电压。
6. `request_state pwm_test`，EN_GATE 仍为低，用示波器看 TIM1 六路 PWM。
7. 接 MT6701 ABZ，`request_state encoder_test`，手转电机确认 angle/count/velocity 变化。
8. `request_state adc_offset_test`，确认 ia/ib/ic 接近 0A 且稳定。
9. 初始化/读取 DRV8301，确认 nFAULT 未触发。
10. 接 2804 三相线到 M0，电机空载、能自由转动。
11. `request_state current_offset_calibration`。
12. `request_state open_loop_voltage_test`，只用 0.5V 以下，确认电机有轻微响应且不过流。
13. `request_state motor_calibration`，从 0.05V 到 0.10V 小电压估算 R/L。
14. `request_state encoder_calibration`，判断方向并计算电角度 offset。
15. `set control_mode torque`，`set input_torque 0.01`，`request_state closed_loop`，先用很小力矩测试。
16. torque 模式稳定后，再测速度环和位置环。

## 安全警告

- 不要一上来用 48V。
- 不要一上来大电流。
- 不要带负载校准。
- 电机必须能自由转动。
- 出现抖动、尖叫、过流、nFAULT 或电源限流，立即断电。
- 没有确认 pole_pairs、相序、电流采样方向、编码器方向和 offset 前，不要提高电流/电压限制。

## PC 测试

```bash
make test
```

当前 PC 测试覆盖：

- Clarke/Park/反 Park/SVPWM。
- 电流 PI 和电压限幅。
- `foc_sim_step_wrapper()` 边界条件。
- 速度环 Simulink wrapper。
- 一阶电流对象闭环响应。

## 后续 TODO

- Axis1 支持。
- MT6701 I2C 配置。
- MT6701 SSI 读取。
- CAN 控制。
- 参数 Flash 保存。
- 制动电阻保护。
- 温度降额。
- 弱磁控制。
- MTPA。
