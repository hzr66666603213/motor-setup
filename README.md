# ODrive v3.6 Axis0 学习用 FOC 固件框架

本项目是在 **ODrive v3.6** 硬件上搭建的一套学习和二次开发用 FOC 电机控制代码框架。第一阶段只启用 **Axis0**，用于安全地驱动一颗 **2804 外转子无刷电机**，编码器使用 **MT6701 的 ABZ 增量输出** 接入 ODrive Encoder0。

本框架不复制 ODrive 官方固件代码，只参考 ODrive v3.x 的硬件架构和模块划分。所有硬件访问都通过 `board/hal/drivers` 抽象层完成，FOC 数学模块不直接调用 STM32 HAL。

## 硬件清单

- ODrive v3.6
- STM32F405RG，板载于 ODrive v3.6
- DRV8301 类三相栅极驱动
- 2804 外转子无刷电机
- MT6701 磁编码器，第一阶段使用 ABZ
- STLINK-V3MINIE
- USB 数据线
- 直流限流电源，建议先用 12 V 或 24 V，不要直接上 48 V

## 为什么第一版使用 ABZ 而不是 SSI

- ODrive v3.6 原生硬件接口支持 quadrature encoder。
- MT6701 的 ABZ 输出更容易先接入 Encoder0 A/B/Z。
- ABZ 计数可以直接用 STM32 定时器 Encoder Mode 读取，适合第一阶段学习。
- SSI/绝对角读取后续再做，第一版先把 PWM、电流采样、ABZ、开环和闭环链路跑通。

注意：ABZ 是增量反馈，上电后绝对角度未知。如果不使用 Z/index，每次上电都必须重新做电角度零位校准。

## 目录结构

```text
include/
  app/
    axis0_types.h              Axis0 公共配置、实时状态、命令和状态枚举。
    axis0_current_loop_isr.h   Axis0 20kHz FOC 电流环 ISR 入口。
    axis_state_machine.h       Axis0 状态机，1kHz/后台调用。
    calibration.h              Axis0 非阻塞校准流程。
    console.h                  USB CDC/UART 文本命令接口。
    parameter_table.h          Axis0 调试参数读写表。
  board/
    board_odrive_v36.h         ODrive v3.6 板级资源、pin map 和安全接口。
  config/
    axis0_default_config.h     2804 + MT6701 ABZ 的低风险默认参数。
  drivers/
    current_sensor.h           ODrive Axis0 电流/母线电压采样转换。
    drv8301.h                  DRV8301 gate driver 抽象。
    encoder_mt6701_abz.h       MT6701 ABZ 增量编码器模块。
  foc/
    foc_math.h                 Clarke/Park/反 Park/角度归一化/限幅/滤波。
    svpwm.h                    独立 SVPWM 模块。
  control/
    current_controller.h       d/q 电流 PI。
    velocity_controller.h      速度 PI。
    position_controller.h      位置 P。
  protection/
    fault.h                    故障 bitmask 和安全关断。
    protection.h               快速/慢速保护。
  hal/
    hal_*.h                    可替换为 STM32 HAL/LL 的硬件抽象接口。
src/
  app/
    axis0_current_loop_isr.c
    axis_state_machine.c
    calibration.c
    console.c
    parameter_table.c
    axis0_default_config.c
  board/
    board_odrive_v36.c
  drivers/
    current_sensor.c
    drv8301.c
    encoder_mt6701_abz.c
  foc/
    foc_math.c
    svpwm.c
  control/
    current_controller.c
    velocity_controller.c
    position_controller.c
  protection/
    fault.c
    protection.c
  hal/
    hal_*.c                    当前是 mock/stub，后续替换为 STM32 实现。
tests/
  foc_math_test.c
```

## 控制频率

| 任务 | 频率 | 主要文件 |
| --- | --- | --- |
| PWM/电流环 | 20 kHz | `axis0_current_loop_isr.c`、`current_controller.c`、`foc_math.c`、`svpwm.c` |
| 速度环 | 1 kHz | `axis_state_machine.c`、`velocity_controller.c` |
| 位置环 | 500 Hz 或 1 kHz | `position_controller.c` |
| 状态机/保护慢速检查 | 1 kHz / 100 Hz | `axis_state_machine.c`、`protection.c` |
| USB CDC/UART 命令 | 后台 | `console.c`、`parameter_table.c` |

20 kHz ISR 中禁止 `malloc`、`printf`、`delay` 和阻塞式 SPI/I2C/CAN/USB 通信。

## ODrive v3.6 Pin Map 说明

已按公开 ODrive v3.6 pinout 填入的资源：

- Axis0 PWM：TIM1
  - AH `PA8`
  - BH `PA9`
  - CH `PA10`
  - AL `PB13`
  - BL `PB14`
  - CL `PB15`
- Axis0 电流采样：
  - `M0_SO1 = PC0 / ADC2_IN10`
  - `M0_SO2 = PC1 / ADC2_IN11`
  - 第三相第一版按两相采样由 `ic = -ia - ib` 推算
- VBUS：`PA6 / ADC1_IN6`
- Axis0 温度：`PC5 / ADC1_IN15`
- DRV8301 SPI3：
  - SCK `PC10`
  - MISO `PC11`
  - MOSI `PC12`
  - Axis0 CS `PC13`
  - Axis1 CS `PC14`
  - EN_GATE `PB12`
  - nFAULT `PD2`
- Encoder0：
  - A `PB4 / TIM3_CH1`
  - B `PB5 / TIM3_CH2`
  - Z `PC9`
- USB：
  - DM `PA11`
  - DP `PA12`
- UART2 可选：
  - TX `PA2`
  - RX `PA3`
- CAN 可选：
  - RX `PB8`
  - TX `PB9`

仍需人工确认：

- `ODRV36_BRAKE_RES_PORT_PIN`：制动电阻控制引脚和驱动逻辑。
- 电流采样比例：采样电阻、运放增益、ADC 参考电压。
- VBUS 分压比例。
- Axis0 两路电流采样与实际相名的对应关系。

参考资料：[Apache NuttX ODrive v3.6 board 文档](https://nuttx.apache.org/docs/12.11.0/platforms/arm/stm32f4/boards/odrive36/index.html)。

## 默认低风险参数

见 `include/config/axis0_default_config.h`：

- 默认测试电压：12 V 或 24 V
- `current_limit = 1.0 A`
- `calibration_current = 0.5 A`
- `voltage_limit = 3.0 V`
- `velocity_limit = 20 rad/s`
- `pwm_frequency = 20000 Hz`
- `current_loop = 20000 Hz`
- `velocity_loop = 1000 Hz`
- `position_loop = 500 Hz`
- MT6701 ABZ 默认 `ppr = 1024`，`cpr = 4096`
- `pole_pairs = 7`

重要：2804 不同厂家极对数可能不同，默认 7 只是起点，必须实测确认。

## 接线说明

- 2804 三相线接 ODrive 的 M0 三相输出。
- MT6701：
  - A 接 Encoder0 A
  - B 接 Encoder0 B
  - Z 可选接 Encoder0 Z
  - VCC/GND 接 ODrive 编码器供电和地
- 直流限流电源接 ODrive DC input。
- STLINK-V3MINIE 接 ODrive SWD。
- USB 数据线接 ODrive USB。

## STLINK-V3MINIE 烧录说明

STLINK-V3MINIE 到 ODrive SWD 至少连接：

- SWDIO
- SWCLK
- GND
- NRST
- VTref / 3.3 V reference

ODrive 板必须由自己的直流电源供电，STLINK 的 VTref 只用于电平参考，不建议用 STLINK 给功率板供电。

## 当前代码到电机转起来的前提

当前仓库已经给出了 FOC、状态机、校准、保护、DRV8301、MT6701 ABZ、控制台和 ODrive v3.6 板级抽象的代码框架，但 `src/hal/hal_*.c` 仍然是 mock/stub。也就是说：

- 现在的代码结构是可以移植的固件骨架。
- 只有把 HAL 层真正绑定到 STM32F405RG 外设后，ODrive v3.6 板上的电机才会实际转动。
- 在 HAL 未替换前，`get vbus`、PWM、ADC、TIM3 编码器、DRV8301 SPI 都只是占位行为。

详细实现步骤见 [ODrive v3.6 真实硬件移植实现指南](docs/ODRIVE_V36_REAL_PORTING.md)。

让电机真正转起来前，必须完成以下硬件绑定：

1. `hal_pwm.c`：绑定 TIM1 CH1/2/3 和 CH1N/2N/3N，中心对齐 20 kHz，互补 6PWM，配置死区。
2. `hal_adc.c`：绑定 Axis0 电流采样 ADC、VBUS ADC、温度 ADC，并由 TIM1 在正确采样窗口触发。
3. `hal_gpio.c`：绑定 `PB12 EN_GATE`、`PD2 nFAULT`、调试 LED 或测试点。
4. `hal_spi.c`：绑定 SPI3，用于 DRV8301 寄存器读写，Axis0 CS 为 `PC13`。
5. `encoder_mt6701_abz.c`：把 `encoder_mt6701_abz_read_raw_count()` 改为读取 TIM3 Encoder Mode 的展开计数。
6. `hal_uart.c` 或 USB CDC 后端：把 `console_poll()` 接到 USB CDC 或 UART 接收环形缓冲。
7. 在 ADC 转换完成中断或 PWM 周期中断中调用 `axis0_current_loop_isr()`。
8. 在 1 kHz 定时任务中调用 `axis_update_1khz()`。
9. 在后台循环中调用 `axis_update_background()` 和 `console_poll()`。

## STM32CubeIDE 工程接入步骤

推荐先用 STM32CubeIDE 建一个最小工程，再把本仓库模块逐步加入。

1. 新建 MCU 工程，芯片选择 `STM32F405RGT6` 或与 ODrive v3.6 实物一致的 STM32F405RG 封装配置。
2. 时钟建议配置到 168 MHz，打开 SWD 调试接口。
3. 配置 USB FS Device，启用 CDC，或先使用 UART2 做文本控制台。
4. 配置 TIM1：
   - center-aligned PWM；
   - 20 kHz；
   - CH1/2/3 + CH1N/2N/3N；
   - dead time 必须按 ODrive v3.6 MOS/DRV8301 实际需求设置；
   - 上电默认不启动 PWM 输出。
5. 配置 ADC：
   - Axis0 电流采样：`PC0 ADC2_IN10`、`PC1 ADC2_IN11`；
   - VBUS：`PA6 ADC1_IN6`；
   - 温度：`PC5 ADC1_IN15`；
   - ADC 触发应与 TIM1 PWM 同步，采样点应落在低边电流有效窗口。
6. 配置 TIM3 Encoder Mode：
   - Encoder0 A：`PB4 TIM3_CH1`；
   - Encoder0 B：`PB5 TIM3_CH2`；
   - Z/index：`PC9` 可先不启用。
7. 配置 SPI3：
   - SCK `PC10`；
   - MISO `PC11`；
   - MOSI `PC12`；
   - Axis0 DRV8301 CS `PC13`。
8. 配置 GPIO：
   - EN_GATE `PB12` 输出，默认低；
   - nFAULT `PD2` 输入；
   - 可选调试 LED/测试点。
9. 把 `include/` 加入编译器 include path。
10. 把 `src/app`、`src/board`、`src/control`、`src/drivers`、`src/foc`、`src/protection` 加入工程。
11. 用真实 STM32 HAL/LL 代码替换 `src/hal/hal_*.c` 的 stub。
12. 在 `main.c` 中静态创建 Axis0 对象、控制器对象、DRV8301 对象、编码器对象和状态机对象。

最小主循环结构示意：

```c
static Axis0Context axis0;
static Drv8301 drv0;
static EncoderMt6701AbzState enc0;
static CurrentSensorConfig current_sensor0;
static CurrentController current_ctrl0;
static VelocityController velocity_ctrl0;
static PositionController position_ctrl0;
static Axis0StateMachineContext sm0;
static Axis0Console console0;

int main(void)
{
    hal_init_all_stm32_peripherals();

    axis0.config = axis0_default_config_make();
    axis0.state = AXIS0_STATE_BOOT;
    axis0.requested_state = AXIS0_STATE_IDLE;

    current_sensor_set_default_config(&current_sensor0);
    drv8301_init(&drv0);
    encoder_mt6701_abz_init(&enc0, &encoder_config);
    current_controller_init(&current_ctrl0,
                            axis0.config.control.current_kp,
                            axis0.config.control.current_ki,
                            axis0.config.motor.voltage_limit_v);
    velocity_controller_init(&velocity_ctrl0,
                             axis0.config.control.velocity_kp,
                             axis0.config.control.velocity_ki,
                             axis0.config.motor.current_limit_a,
                             axis0.config.motor.velocity_limit_rad_s);
    position_controller_init(&position_ctrl0,
                             axis0.config.control.position_kp,
                             axis0.config.motor.velocity_limit_rad_s,
                             -3.14159f,
                             3.14159f);

    sm0.current_sensor = &current_sensor0;
    sm0.encoder = &enc0;
    sm0.drv = &drv0;
    sm0.velocity_controller = &velocity_ctrl0;
    sm0.position_controller = &position_ctrl0;

    console_init(&console0, &axis0, &enc0);

    while (1) {
        console_poll(&console0);
        axis_update_background(&axis0, &sm0);
    }
}
```

20 kHz 中断中只做高速电流环：

```c
void adc_or_pwm_20khz_callback(void)
{
    static Axis0IsrContext isr_ctx = {
        .axis = &axis0,
        .current_controller = &current_ctrl0,
        .current_sensor = &current_sensor0,
        .encoder = &enc0,
        .drv = &drv0,
    };

    axis0_current_loop_isr(&isr_ctx, 1.0f / 20000.0f);
}
```

1 kHz 定时器任务中跑状态机和外环：

```c
void timer_1khz_callback(void)
{
    axis_update_1khz(&axis0, &sm0, 0.001f);
}
```

## 编译与烧录流程

使用 STM32CubeIDE：

1. `Project -> Build Project`，确认无编译错误。
2. 接好 STLINK-V3MINIE：SWDIO、SWCLK、GND、NRST、VTref。
3. ODrive 用直流限流电源单独供电，先不要接电机三相线。
4. `Run -> Debug Configurations`，选择 ST-LINK。
5. 第一次建议勾选下载后停在 `main()`，不要让程序直接跑起来。
6. 点击 Debug，确认能连接 STM32F405 并成功下载。
7. 单步或运行到 `board_init_power_safe()`，确认 EN_GATE 仍为低、PWM 未输出。
8. 退出 Debug 后复位板子，打开 USB CDC 或 UART 串口，确认控制台有响应。

使用命令行工具时，流程等价：

```text
arm-none-eabi-gcc / CubeIDE build
STM32_Programmer_CLI -c port=SWD -w build/firmware.elf -v -rst
```

具体命令取决于你最终选择 CubeIDE、Makefile 还是 CMake。

## 第一次上电顺序

1. 不接电机。
2. 只接 USB，确认设备能被电脑识别。
3. 接 STLINK，确认 SWD 能连接 STM32F405。
4. 接低压限流电源，建议 12 V，电流限制先设很小。
5. 读取 `vbus`，确认母线电压合理。
6. 确认 PWM 默认关闭。
7. 确认 DRV8301 默认关闭，EN_GATE 不使能。
8. 接 MT6701 ABZ，手动旋转电机，检查 Encoder0 count 是否变化。
9. 断电后接电机三相线。
10. 上电后做电流零偏校准、电机校准、编码器校准。
11. 低电流、低电压进入闭环。

## 调试阶段建议顺序

1. LED/USB 通信测试。
2. PWM 空载测试，gate 仍保持关闭。
3. ADC 零偏测试。
4. MT6701 ABZ 计数测试。
5. 开环 SVPWM，低压低占空比。
6. 电流采样方向确认。
7. 编码器方向判断。
8. 电角度零位校准。
9. Id/Iq 电流环。
10. 速度环。
11. 位置环。

2804 第一次测试必须低压、低电流、空载，转子必须能自由旋转。

## 从烧录到电机转动的实操流程

下面流程按风险从低到高排列。不要跳步。

### 0. 上电前检查

1. 电机三相线先不要接。
2. MT6701 可以先接，也可以等 USB 通信确认后再接。
3. 直流电源设为 12 V，限流建议先设 `0.5 A` 到 `1.0 A`。
4. STLINK 和 USB 都接好。
5. 确认电机轴能自由转动，后续校准时不能带负载。

### 1. 烧录后确认固件活着

打开 USB CDC 或 UART 串口，发送：

```text
get state
get fault
get vbus
```

期望：

- `state idle` 或 `state boot` 后很快进入 `idle`；
- `fault 0`；
- `vbus` 接近你的电源电压，例如 12 V 附近。

如果 `vbus` 明显不对，先不要继续。需要修正 VBUS ADC 分压比例。

### 2. 确认默认安全输出

用万用表或示波器确认：

- `PB12 EN_GATE` 默认低；
- TIM1 PWM 输出默认不驱动功率级；
- DRV8301 `nFAULT` 未报错；
- 直流电源电流接近空载电流。

如果 EN_GATE 上电就是高，立即断电并检查 `hal_gpio_set_gate_enable(false)` 后端。

### 3. 验证 MT6701 ABZ 计数

接好 MT6701 ABZ 到 Encoder0，手动慢慢转动电机，反复发送：

```text
get angle
get velocity
```

期望：

- 手动旋转时 `angle` 变化；
- 停止时 `velocity` 接近 0；
- 如果角度不变，检查 MT6701 供电、A/B 线、TIM3 Encoder Mode、CPR。

### 4. 电流零偏校准

保持电机三相线未接，或者确认功率级关闭，发送：

```text
request_state current_offset_calibration
get state
get fault
```

期望：

- 校准过程中 PWM 和 EN_GATE 仍关闭；
- 完成后进入 `ready` 或回到安全状态；
- `fault 0`。

如果失败，优先检查 ADC 原始值是否稳定、零偏是否接近 ADC 中点。

### 5. 接电机，做低风险电机校准

断电，接 2804 三相线到 M0。重新上电，保持 12 V 限流。发送：

```text
set current_limit 1.0
set voltage_limit 3.0
set pole_pairs 7
request_state motor_calibration
get fault
```

期望：

- 电机可能轻微动作，但不应剧烈抖动；
- 电源不应进入持续限流；
- 无 nFAULT、无过流。

注意：当前 `calibration.c` 的电阻/电感注入仍是 skeleton。如果你还没有实现真实注入函数，这一步只会走状态流程，不会得到真实 R/L。要让电流环可靠工作，必须补上低电压注入和电流采样计算。

### 6. 编码器方向和电角度零位校准

电机必须空载自由旋转，发送：

```text
request_state encoder_calibration
get angle
get fault
```

期望：

- 电机低速、低电流缓慢转动或锁定；
- 方向判断成功；
- offset 写入 `axis0.config.encoder.encoder_offset_rad`；
- 无故障。

注意：当前 `calibration.c` 已预留方向判断和 offset 计算，但“开环旋转电角度”和“小 d 轴锁定电流”的实际输出函数仍需要接入。未接入前，编码器校准不能真正完成实际物理校准。

### 7. 第一次闭环前检查

发送：

```text
get fault
get state
get vbus
get angle
get velocity
set current_limit 0.5
set voltage_limit 2.0
set control_mode torque
set input_torque 0.0
```

确认：

- `fault 0`；
- 已完成电流零偏、电机校准、编码器校准；
- 电机空载；
- 电源限流仍较小；
- 手放在电源开关附近，准备随时断电。

### 8. 进入闭环但不给力矩

```text
request_state closed_loop
get state
get fault
get id
get iq
```

期望：

- 进入 `closed_loop`；
- `id/iq` 接近 0；
- 电机不明显抖动；
- 电源电流没有突然升高。

如果一进闭环就抖动或限流，立即断电。重点检查：

- `pole_pairs`；
- 编码器方向；
- encoder offset；
- 电流采样符号；
- 三相线相序；
- current PI 参数过大。

### 9. 低力矩让电机轻微转动

```text
set input_torque 0.005
get iq
get velocity
set input_torque 0.0
```

如果方向正常、无过流，可以稍微增大到：

```text
set input_torque 0.01
```

不要一开始超过 `0.02 Nm`。如果 torque_constant 不准，实际电流会偏差，以 `get iq` 和电源电流为准。

### 10. 速度环低速测试

```text
set input_torque 0.0
set control_mode velocity
set input_velocity 2.0
get velocity
set input_velocity 0.0
```

确认速度能平滑跟随，再尝试：

```text
set input_velocity 5.0
```

如果振荡，降低 `velocity.kp/ki` 默认值，或先只开很小的 `velocity_kp`。

### 11. 位置环小角度测试

```text
set control_mode position
set input_position 0.2
get angle
set input_position 0.0
```

位置环第一次测试只做小角度，不要直接给大阶跃。

## 当前代码还需要补齐的“真转电机”关键点

为了让 2804 真正稳定转起来，下面这些不是文档项，而是必须完成的代码项：

- `hal_pwm.c` 必须真实写 TIM1 互补 PWM。
- `hal_adc.c` 必须真实读取同步 ADC 样本。
- `hal_spi.c` 必须真实读写 DRV8301。
- `hal_gpio.c` 必须真实控制 EN_GATE 和读取 nFAULT。
- `encoder_mt6701_abz_read_raw_count()` 必须真实读取 TIM3 Encoder Mode 计数。
- `calibration.c` 需要补上开环电压矢量/小 d 轴电流注入函数，否则电阻、电感、方向、offset 只是流程占位。
- DRV8301 寄存器配置和状态解析需要按 datasheet 补齐。
- 电流采样比例、VBUS 比例必须按 ODrive v3.6 实物校准。

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
request_state current_offset_calibration
request_state motor_calibration
request_state encoder_calibration
request_state closed_loop
clear_faults
save_config
reboot
```

## 安全警告

- 不要一上来用 48 V。
- 不要一上来设置大电流。
- 不要带负载校准。
- 校准时电机必须能自由转动。
- 出现抖动、尖叫、过流、nFAULT，立即断电。
- 没有完成电流零偏、电机参数和编码器零位校准，不允许进入闭环。

## 常见问题排查

- STLINK 连接失败：检查 SWDIO/SWCLK/GND/NRST/VTref，确认 ODrive 已单独供电。
- USB 不识别：检查 USB 线是否为数据线，确认固件未卡死在早期 hard fault。
- nFAULT 报错：确认 EN_GATE 默认关闭，检查 DRV8301 电源、SPI 配置和功率级短路。
- 编码器计数不变：检查 MT6701 供电、A/B 接线、TIM3 Encoder Mode 和 cpr 配置。
- 方向反：运行 encoder direction calibration，或临时设置 `encoder_direction = -1`。
- 电机抖动：检查相序、电流采样符号、编码器方向和电角度 offset。
- 电流环发散：降低 `current_kp/current_ki`，确认 R/L 校准和电流采样比例。

## 后续 TODO

- Axis1 支持
- MT6701 I2C 配置
- MT6701 SSI 读取
- CAN 控制
- 参数 Flash 保存
- 制动电阻保护
- 温度降额
- 力矩模式优化
