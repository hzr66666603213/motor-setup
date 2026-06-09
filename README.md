# ODrive v3.6 Axis0 FOC 学习固件

本项目面向真实硬件：

- 主控/驱动板：ODrive v3.6
- MCU：STM32F405RG
- 栅极驱动：DRV8301 类三相预驱
- 电机：2804 外转子无刷电机，默认 `pole_pairs = 7`，必须实测确认
- 编码器：MT6701，第一阶段使用 ABZ 增量输出，接 Encoder0 A/B/Z
- 调试工具：STLINK-V3MINIE、USART2 串口、USB 数据线、直流限流电源

第一阶段只使用 Axis0，Axis1 保留但不启用。所有高危输出默认关闭：上电 `EN_GATE=0`、`TIM1 MOE=0`、不自动闭环、不自动转电机。

## 当前上板状态

仓库已经接入 CubeMX 工程：

- CubeIDE 工程目录：`firmware/odrive_v36_cube`
- CubeMX 配置文件：`firmware/odrive_v36_cube/odrive_v36_cube.ioc`
- 当前 `firmware/odrive_v36_cube/Core/Src/main.c` 是安全 bring-up 程序

当前 `main.c` 做的事情：

1. 初始化 GPIO、ADC1、ADC2、SPI3、TIM1、TIM3、USART2。
2. 调用 `hal_adc_init()`，使用 injected ADC interrupt 等待 TIM1 触发。
3. 调用 `board_init_power_safe()`，保持 `EN_GATE=0`、`TIM1 MOE=0`，只启动 TIM1/CC4 作为 ADC 触发源。
4. 启动 TIM3 Encoder Mode，方便手转电机观察 ABZ 计数。
5. 每 1s 通过 USART2 打印 bring-up 状态。

当前 `main.c` 不会使能 DRV8301，不会打开三相功率 PWM，不会让电机转起来。这是故意的，用于第一轮上板验收。

## 本轮板级审核结论

已经确认的安全点：

- `EN_GATE` 默认由 GPIO 拉低。
- `nFAULT` 为 PD2，低有效，板级读取逻辑按低有效处理。
- TIM1 三相 PWM 使用 PA8/PA9/PA10 和 PB13/PB14/PB15。
- ADC injected trigger 配置为 `TIM1_CC4`。
- `hal_pwm_start_adc_trigger_only()` 只启动 TIM1 base + OC4，不启动 CH1/2/3 和 CH1N/2N/3N。
- ADC 后端使用 `HAL_ADCEx_InjectedStart_IT()`。
- `HAL_ADCEx_InjectedConvCpltCallback(hadc)` 已在 `main.c` 中转发到 `hal_adc_stm32f405_on_injected_complete((void *)hadc)`。
- `hal_adc_stm32f405_on_injected_complete()` 只有在 `hadc1` 和 `hadc2` 本周期都完成后才递增 `seq`。
- CubeIDE 工程已改为相对路径链接仓库源码，不再依赖旧电脑绝对路径。

仍需实物确认的点：

- ODrive v3.6 具体版本的 VBUS 分压比例，24V/56V 版本可能不同。
- DRV8301 shunt amp gain 与 `amp_per_count` 是否一致。
- TIM1 deadtime 是否适合当前功率级。
- MT6701 ABZ 的实际 PPR/CPR，默认按 `ppr=1024`、`cpr=4096`。
- 2804 电机极对数，默认 7 只是初始值。
- USART2 的实际引脚 PA2/PA3 是否已接出到你的调试转串口。

## CubeMX 外设配置摘要

当前 `.ioc` 关键配置：

- TIM1
  - Center aligned mode
  - Period = 4199，目标 20 kHz
  - CH1/CH2/CH3：三相 PWM
  - CH1N/CH2N/CH3N：三相互补 PWM
  - CH4：Output Compare no output，用于 ADC injected trigger
  - DeadTime = 50
- ADC1
  - Injected rank 1：VBUS，PA6 / ADC1_IN6
  - Injected rank 2：MOS/板温，PC5 / ADC1_IN15
  - External trigger：TIM1_CC4 rising edge
- ADC2
  - Injected rank 1：M0_SO1，PC0 / ADC2_IN10
  - Injected rank 2：M0_SO2，PC1 / ADC2_IN11
  - External trigger：TIM1_CC4 rising edge
- TIM3
  - Encoder Mode TI12
  - PB4 / TIM3_CH1：Encoder A
  - PB5 / TIM3_CH2：Encoder B
- SPI3
  - PC10 SCK、PC11 MISO、PC12 MOSI
  - 16-bit，Mode 1 风格：CPOL low、CPHA 2 edge
  - PC13 DRV0_CS，PC14 DRV1_CS
- GPIO
  - PB12：EN_GATE，默认 low
  - PD2：DRV_NFAULT，输入，低有效
- USART2
  - PA2 TX、PA3 RX，115200 8N1

## CubeIDE 工程说明

导入工程：

1. 打开 STM32CubeIDE。
2. `File -> Import -> Existing Projects into Workspace`。
3. 选择 `firmware/odrive_v36_cube`。
4. 导入后执行 `Project -> Clean`，让 CubeIDE 重新生成 Debug makefile。

注意：

- 工程通过 linked resources 引用仓库根目录的 `src/app`、`src/board`、`src/control`、`src/drivers`、`src/foc`、`src/hal/stm32f405`、`src/protection`。
- 不要把 `src/hal/mock` 加入真实固件工程。
- 如果 CubeMX 重新生成工程后覆盖了 `.project` / `.cproject`，要确认 linked resources 和 include path 仍指向仓库根目录。
- `Debug/` 是构建产物目录，不应作为源码目录参与扫描。

## STLINK 烧录连接

STLINK-V3MINIE 连接 ODrive：

- SWDIO -> ODrive SWDIO
- SWCLK -> ODrive SWCLK
- GND -> ODrive GND
- NRST -> ODrive NRST
- VTref -> ODrive 3.3V reference

ODrive 必须由自己的电源供电，STLINK 的 VTref 只作为电平参考，不给 ODrive 主电源供电。

烧录方式：

1. CubeIDE 中选择 Debug。
2. 第一次建议只接 USB/STLINK，不接电机三相线，不接大电流负载。
3. 烧录后程序会进入安全 bring-up 循环。
4. USART2 每 1s 输出一次状态。

串口参数：

- 115200 baud
- 8 data bits
- no parity
- 1 stop bit
- no flow control

## 第一次上板流程

严格按顺序执行。

### 1. 不接电机，不接主电源

目标：确认 MCU、STLINK、串口基本可用。

1. 只接 STLINK 和必要的 3.3V reference。
2. CubeIDE 能识别 STM32F405RG。
3. 烧录 `firmware/odrive_v36_cube`。
4. 打开 USART2 串口。
5. 应看到类似输出：

```text
odrive_v36_cube bringup start
SAFE MODE: EN_GATE should stay LOW, MOE should stay OFF, motor must be disconnected.
bringup: adc_init=... board_init=... cb=... valid=... seq=...
```

验收：

- 板子不发热。
- `EN_GATE` 测量为低。
- DRV `nFAULT` 没有被拉低。
- 程序持续打印，没有 HardFault。

### 2. 接低压限流电源，不接电机

目标：确认 VBUS ADC 和 ADC seq。

建议：

- 电源 12V。
- 限流 0.3A 到 0.5A。
- 电机三相线仍不要接。

观察：

- `cb` 应持续增加，说明 ADC 中断在触发。
- `valid=1` 后，`seq` 应持续增加。
- `raw_vbus` 应随母线电压变化。
- `gate=0`，`pwm_disabled=1`。

如果 `cb` 不增加：

- 检查 ADC_IRQn 是否使能。
- 检查 `HAL_ADCEx_InjectedConvCpltCallback()` 是否存在。
- 检查 ADC injected external trigger 是否为 `TIM1_CC4`。
- 检查 `hal_pwm_start_adc_trigger_only()` 是否启动 TIM1 OC4。

### 3. 检查 TIM1 ADC trigger-only 安全态

目标：确认 ADC 触发在跑，但功率级没有输出。

用示波器或调试器确认：

- `EN_GATE=0`。
- TIM1 `MOE=0`。
- CH1/CH2/CH3 和 CH1N/CH2N/CH3N 不应输出功率 PWM。
- TIM1 CH4/OC4 或内部 CC4 事件用于 ADC 触发。

这一步不能接电机。

### 4. 检查 MT6701 ABZ

目标：确认 TIM3 Encoder Mode 可计数。

1. 接 MT6701 的 A/B/GND/VCC。
2. 如果接 Z/index，当前第一版可暂不使用。
3. 手转电机轴。
4. 用调试器观察 TIM3 CNT，或后续接入 `encoder_mt6701_abz_update()` 后观察 raw_count。

验收：

- 手转正反方向，TIM3 CNT 有变化。
- 变化连续，没有明显跳变。
- 如果方向反，后续通过 encoder direction 配置修正。

### 5. 检查 DRV8301 SPI，不使能 EN_GATE

目标：确认 SPI3 和两个 DRV CS 可读写。

顺序：

1. 保持 `EN_GATE=0`。
2. 通过低频后台或临时测试代码读取 DRV0/DRV1 status。
3. 确认 SPI 不超时。
4. 确认 `nFAULT` 未触发。

注意：

- DRV8301 SPI 不能在 20kHz ISR 中阻塞读取。
- EN_GATE/nFAULT 是 M0/M1 共享资源，Axis0-only 也必须确认 DRV1 不异常。

### 6. 检查电流 ADC 零偏

目标：确认 two-shunt 电流采样可用。

条件：

- 不接电机或电机三相线悬空。
- `EN_GATE=0`。
- `TIM1 MOE=0`。
- ADC seq 持续增加。

验收：

- M0_SO1/M0_SO2 原始 count 稳定。
- 计算出的 ia/ib 接近 0A。
- `ic` 不来自伪造 ADC，必须由 `ic=-ia-ib` 推算。

### 7. 只测 PWM 波形，仍不使能 DRV

目标：确认 TIM1 六路互补 PWM 逻辑。

当前 `main.c` 还没有进入 `pwm_test` 状态；要做这一步，需要后续接入状态机/console，或临时测试调用 `hal_pwm_enable()`，但必须保持 `EN_GATE=0`。

验收：

- CH1/2/3 和 CH1N/2N/3N 频率约 20kHz。
- duty 初始 50%。
- 互补和 deadtime 正常。
- 任何异常先断电，不要接电机。

### 8. 接电机前检查

接 2804 电机前必须满足：

- VBUS 读数可信。
- ADC seq 稳定更新。
- 电流零偏稳定。
- MT6701 ABZ 计数正常。
- DRV SPI 可读。
- nFAULT 未触发。
- PWM 波形在 EN_GATE=0 时确认过。
- 电源仍设为 12V，限流 0.5A 左右。

### 9. 低压开环测试

目标：确认相序、电流采样方向和编码器方向。

建议初始限制：

- `voltage_limit <= 0.5V`
- `current_limit <= 0.5A`
- 电机空载、能自由转动

流程：

1. 进入 open-loop voltage test。
2. 缓慢旋转电角度。
3. 观察电机是否轻微、平滑跟随。
4. 一旦电源限流、抖动、尖叫、nFAULT，立即断电。

### 10. 校准和闭环

闭环前必须依次完成：

1. current offset calibration。
2. motor resistance calibration，从 0.05V 到 0.1V 小电压开始。
3. motor inductance calibration，短脉冲、小电流。
4. encoder direction calibration。
5. encoder offset calibration。

三个标志都为 true 后才允许进入 closed-loop：

- `current_offset_valid`
- `motor_calibrated`
- `encoder_calibrated`

第一次闭环建议：

- torque mode。
- `input_torque` 很小，例如 0.005 到 0.01 Nm。
- 电源 12V，限流 0.5A 到 1A。
- 确认电流环稳定后再测试速度环。
- 速度环稳定后再测试位置环。

## 当前 main.c 的下一步建议

当前 `main.c` 适合第一轮硬件 bring-up。要让电机真正用这套代码转起来，下一步建议按这个顺序接入：

1. USART2 简单命令层：`get_status`、`get_adc`、`get_encoder`、`get_drv`。
2. DRV8301 status 读取和初始化打印。
3. TIM3 encoder 状态打印。
4. `pwm_test`，EN_GATE 保持低，只测六路 PWM。
5. `current_offset_calibration`。
6. `open_loop_voltage_test`，低压低流。
7. `motor_calibration` 和 `encoder_calibration`。
8. `closed_loop_control`，先 torque，再 velocity，再 position。

## 安全警告

- 不要一上来使用 48V。
- 不要一上来接负载。
- 不要跳过 ADC/VBUS/ABZ/DRV/PWM 单项验证。
- 不要在电机不能自由转动时做方向和 offset 校准。
- 出现抖动、尖叫、过流、nFAULT 或电源限流，立即断电。
- 2804 不同厂家极对数可能不同，`pole_pairs=7` 只是默认值。
- DRV8301 gain、shunt 阻值、VBUS 分压比例必须按实物校准。

## PC 测试

PC 侧算法测试：

```bash
make test
```

当前覆盖：

- Clarke / Park / inverse Park / SVPWM。
- 电流 PI 和电压限幅。
- `foc_sim_step_wrapper()` 边界条件。
- 速度环 Simulink wrapper。
- 一阶电流对象闭环响应。
- ADC trigger-only mock 测试。

## 后续 TODO

- CubeIDE 工程加入更完整的 console 命令。
- Flash 参数保存。
- Axis1 支持。
- MT6701 I2C/SSI 支持。
- CAN 控制。
- 制动电阻保护。
- 温度降额。
- 弱磁控制。
- MTPA。
