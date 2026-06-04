# ODrive v3.6 真实硬件移植实现指南

本文回答 README 中列出的“真转电机”关键项如何实现。目标硬件：

- ODrive v3.6
- STM32F405RG
- DRV8301
- Axis0 M0 三相功率级
- MT6701 ABZ 接 Encoder0
- 2804 小无刷电机

## 总体原则

当前工程已经把算法层和硬件层分开。真正让电机转起来时，你要做的是：

1. 用 CubeMX/CubeIDE 配好 STM32F405 外设。
2. 把 `src/hal/hal_*.c` 从 mock 改成真实 STM32 HAL/LL。
3. 把 `encoder_mt6701_abz_read_raw_count()` 改成真实 TIM3 Encoder Mode 计数。
4. 把 `calibration.c` 的占位注入动作改成真实开环电压矢量输出。
5. 校准电流比例、母线电压比例和 DRV8301 配置。

建议顺序：先不要接电机，先验证 GPIO、PWM、ADC、SPI、ABZ，再接电机做低压低流校准。

## 1. `hal_pwm.c`：真实写 TIM1 互补 PWM

### CubeMX 配置

- TIM1
- Center-aligned mode
- PWM frequency：20 kHz
- CH1/CH2/CH3：高边 PWM
- CH1N/CH2N/CH3N：低边互补 PWM
- Dead time：按 ODrive v3.6 MOSFET/DRV8301 实际需要设置
- Break 输入第一版可先关闭，后续建议接入硬件保护
- 上电默认不要自动启动 PWM

Axis0 引脚：

- AH `PA8 / TIM1_CH1`
- BH `PA9 / TIM1_CH2`
- CH `PA10 / TIM1_CH3`
- AL `PB13 / TIM1_CH1N`
- BL `PB14 / TIM1_CH2N`
- CL `PB15 / TIM1_CH3N`

### HAL 实现思路

`hal_pwm_init()`：

- 初始化 TIM1 PWM 和互补输出
- 设置 CCR1/CCR2/CCR3 为 ARR/2
- 不启动 PWM

`hal_pwm_enable()`：

- 先写 50% duty
- 启动 CH1/2/3
- 启动 CH1N/2N/3N
- 最后置 `s_pwm_enabled = true`

`hal_pwm_disable()`：

- 关闭 TIM1 CH1/2/3 和 CH1N/2N/3N
- 必要时强制 MOE=0
- 置 `s_pwm_enabled = false`

`hal_pwm_set_duty()`：

```c
uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
uint32_t ccr1 = (uint32_t)(clamp01(duty_u) * (float)arr);
uint32_t ccr2 = (uint32_t)(clamp01(duty_v) * (float)arr);
uint32_t ccr3 = (uint32_t)(clamp01(duty_w) * (float)arr);
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);
```

### 验证

不接电机、不使能 EN_GATE，用示波器看 TIM1 六路 PWM：

- 频率 20 kHz
- 三路高边 duty 正确
- 三路低边互补
- 有死区
- `hal_pwm_disable()` 后全部关闭

## 2. `hal_adc.c`：真实读取同步 ADC 样本

### CubeMX 配置

ODrive v3.6 Axis0 公开 pinout：

- M0_SO1：`PC0 / ADC2_IN10`
- M0_SO2：`PC1 / ADC2_IN11`
- VBUS：`PA6 / ADC1_IN6`
- M0_TEMP：`PC5 / ADC1_IN15`

建议：

- 电流采样用 TIM1 触发，触发点放在低边采样有效窗口。
- 用 ADC injected conversion 或 DMA regular conversion 都可以，但要保证样本和 PWM 周期同步。
- 第一版只用两相电流，第三相由 `ic = -ia - ib` 推算。

### HAL 实现思路

维护一个 DMA/注入转换结果缓存：

```c
static volatile uint16_t s_raw_phase_a;
static volatile uint16_t s_raw_phase_b;
static volatile uint16_t s_raw_vbus;
static volatile bool s_adc_valid;
```

ADC 转换完成回调：

```c
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc2) {
        s_raw_phase_a = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
        s_raw_phase_b = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
        s_adc_valid = true;
    }
}
```

`hal_adc_get_phase_current_raw()`：

- 从 volatile 缓存复制到局部
- `raw->u = s_raw_phase_a`
- `raw->v = s_raw_phase_b`
- `raw->w = 2048` 或保留占位，真实电流在 `drivers/current_sensor.c` 用两相模式推算

### 电流比例公式

假设：

- ADC 参考电压 `Vref`
- ADC 满量程 `adc_full_scale = 4095`
- 分流电阻 `Rshunt`
- 电流采样放大倍数 `gain`
- ADC 零偏 `offset_count`

则：

```text
amp_per_count = Vref / adc_full_scale / gain / Rshunt
phase_current_A = (raw_count - offset_count) * amp_per_count
```

ODrive v3.6 的 `Rshunt`、放大器增益需要按实物/原理图确认。不要凭默认值直接大电流运行。

### VBUS 比例公式

假设母线分压为：

```text
ADC_voltage = VBUS * R_low / (R_high + R_low)
```

则：

```text
VBUS = raw / 4095 * Vref * (R_high + R_low) / R_low
```

先用万用表测真实母线电压，再调整 `board_read_vbus_v()` 或 `axis0_current_sensor_vbus_from_raw()` 的比例，让 `get vbus` 与万用表一致。

## 3. `hal_spi.c`：真实读写 DRV8301

### CubeMX 配置

- SPI3
- SCK `PC10`
- MISO `PC11`
- MOSI `PC12`
- Axis0 CS `PC13`
- Mode 1 或按 DRV8301 datasheet 配置 CPOL/CPHA
- 16-bit 或 8-bit 都可；当前代码按两个 byte 发送

### HAL 实现思路

`hal_spi_transfer()` 中：

```c
bool hal_spi_transfer(uint8_t bus_id, const uint8_t *tx, uint8_t *rx, size_t length)
{
    if (bus_id != 3u) {
        return false;
    }

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi3,
                                                   (uint8_t *)tx,
                                                   rx,
                                                   length,
                                                   10);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    return st == HAL_OK;
}
```

注意：

- 这个阻塞 SPI 只能在启动/后台使用。
- 不能在 `axis0_current_loop_isr()` 中调用。
- DRV8301 读寄存器结果延迟一帧，`drv8301.c` 已按“两帧读”方式封装。

### DRV8301 当前已实现

`src/drivers/drv8301.c` 已补上：

- SPI 读写帧格式
- STATUS1/STATUS2 读取
- STATUS1 过流、过温、欠压、总故障解析
- STATUS2 GVDD_OV 解析
- Control1 gate reset 清故障
- 6PWM 保守配置入口
- OCP threshold 设置入口
- gate current 设置入口

后续你还需要按最终硬件策略确认：

- OCP 模式
- OC_ADJ_SET 阈值
- shunt amplifier gain
- gate current
- 是否需要读取并打印原始 status 寄存器

## 4. `hal_gpio.c`：真实控制 EN_GATE 和 nFAULT

### CubeMX 配置

- `PB12 EN_GATE`：GPIO Output，默认低
- `PD2 nFAULT`：GPIO Input，上拉/不上拉按原理图确认

### HAL 实现

```c
void hal_gpio_set_gate_enable(bool enabled)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,
                      enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool hal_gpio_read_fault_pin(void)
{
    return HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == GPIO_PIN_RESET;
}
```

注意：`nFAULT` 通常低有效，所以返回 `true` 表示故障。

验证：

- 上电 EN_GATE 必须为低
- `request_state closed_loop` 前 EN_GATE 仍为低
- 故障后 EN_GATE 立即变低

## 5. `encoder_mt6701_abz_read_raw_count()`：真实读取 TIM3 Encoder Mode

### CubeMX 配置

- TIM3 Encoder Mode
- CH1 `PB4`
- CH2 `PB5`
- Z/index `PC9` 第一版可先不用
- Counter period 可设 0xffff

### 实现思路

TIM3 是 16-bit 计数器，需要展开成 int32：

```c
static int32_t s_encoder_count_accum;
static uint16_t s_prev_tim3_cnt;

int32_t encoder_mt6701_abz_read_raw_count(void)
{
    uint16_t now = __HAL_TIM_GET_COUNTER(&htim3);
    int16_t delta = (int16_t)(now - s_prev_tim3_cnt);
    s_prev_tim3_cnt = now;
    s_encoder_count_accum += (int32_t)delta;
    return s_encoder_count_accum;
}
```

初始化时：

```c
s_prev_tim3_cnt = 0;
s_encoder_count_accum = 0;
__HAL_TIM_SET_COUNTER(&htim3, 0);
HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
```

验证：

- 手转电机，`get angle` 变化
- 一圈大约变化 `2*pi`
- 如果一圈不是 `2*pi`，检查 CPR
- 如果方向反，运行方向校准或设置 direction=-1

## 6. `calibration.c`：补开环电压矢量/小 d 轴注入

当前 `calibration.c` 是状态流程，缺少真实“给电机施加电压矢量”的函数。建议增加内部函数：

```c
static void axis0_apply_open_loop_voltage(Axis0Context *axis,
                                          float v_alpha_v,
                                          float v_beta_v)
{
    SvpwmDuty duty = svpwm_generate(v_alpha_v, v_beta_v, axis->rt.vbus_v);
    board_axis0_set_pwm_duty(duty.duty_a, duty.duty_b, duty.duty_c);
}

static void axis0_apply_voltage_in_electrical_frame(Axis0Context *axis,
                                                    float vd_v,
                                                    float vq_v,
                                                    float electrical_angle_rad)
{
    float v_alpha = 0.0f;
    float v_beta = 0.0f;
    foc_limit_voltage(&vd_v, &vq_v, axis->config.motor.voltage_limit_v);
    foc_inv_park(vd_v, vq_v, electrical_angle_rad, &v_alpha, &v_beta);
    axis0_apply_open_loop_voltage(axis, v_alpha, v_beta);
}
```

### 电阻测量

安全版本：

1. 使能 DRV8301 和 PWM
2. 施加很小的固定电压，例如 `0.2V ~ 0.5V`
3. 等待电流稳定
4. 记录电流 `I`
5. `R = V / I`

注意：

- 小电机相电阻很低，电流会上升很快
- 必须检查 `calibration_current_a`
- 超时必须关断

### 电感测量

安全版本：

1. 施加短电压脉冲 `V`
2. 记录脉冲前后电流差 `di`
3. 记录脉冲时间 `dt`
4. `L = V / (di/dt)`

注意：

- `dt` 可以从几个 PWM 周期开始
- 过流立即停止
- 多次测量取平均

### 编码器方向判断

用开环电角度缓慢正向旋转：

```c
electrical_angle += electrical_speed_rad_s * dt_s;
axis0_apply_voltage_in_electrical_frame(axis,
                                        calibration_voltage_v,
                                        0.0f,
                                        electrical_angle);
```

观察 ABZ count：

- count 增加：direction=+1
- count 减少：direction=-1
- 变化太小：编码器没接好、电机没动、相序不对或转子卡住

### 电角度零位

锁定到已知电角度，例如 0 rad：

```c
axis0_apply_voltage_in_electrical_frame(axis,
                                        lock_voltage_v,
                                        0.0f,
                                        0.0f);
```

等待机械角稳定后：

```text
encoder_offset = target_electrical_angle
               - mechanical_angle * pole_pairs * direction
```

然后归一化到 `0..2pi`。

## 7. DRV8301 寄存器配置和状态解析

`drv8301.c` 已按 datasheet 形式补了骨架。你需要根据真实策略决定：

- gate current：学习阶段建议低一点，例如 `DRV8301_GATE_CURRENT_0P25A`
- OCP mode：学习阶段建议 current limit 或 latch shutdown
- OC_ADJ_SET：从保守阈值开始，不要太高
- shunt amp gain：必须和电流换算公式一致

后台建议每 10ms~100ms 读一次：

```c
drv8301_read_status(&drv0);
if (drv8301_has_fault(&drv0)) {
    set_fault(&axis0, AXIS0_FAULT_DRV8301_FAULT);
}
```

不要在 20kHz ISR 中读 SPI。

## 8. 电流采样比例和 VBUS 比例校准

### VBUS

1. 用万用表测 DC input，比如 12.00V
2. 固件 `get vbus`
3. 修改比例：

```text
scale_new = scale_old * multimeter_vbus / firmware_vbus
```

直到误差足够小。

### 电流

安全做法：

1. PWM 关闭，做 offset calibration
2. 用已知小电流或可控负载校准 `amp_per_count`
3. 或按原理图计算初值，再用实验修正

公式：

```text
amp_per_count = Vref / 4095 / gain / Rshunt
```

注意：

- DRV8301 shunt amp gain 配置必须和公式一致
- 两相采样相名和符号必须确认
- 符号错会导致电流环发散

## 推荐落地顺序

1. 实现 GPIO：EN_GATE/nFAULT
2. 实现 VBUS ADC，校准 `get vbus`
3. 实现 TIM3 ABZ，确认 `get angle`
4. 实现 TIM1 PWM，但 EN_GATE 保持低，用示波器看波形
5. 实现 DRV8301 SPI，能读 status
6. 实现电流 ADC 零偏
7. 接电机，低压限流，做开环 SVPWM
8. 确认电流采样符号
9. 做编码器方向和 offset
10. 进电流环，`input_torque` 从 0 开始
11. 再做速度环、位置环

## 关键安全检查

- 上电 EN_GATE 必须低
- PWM 默认关闭
- VBUS 不准，不进入闭环
- nFAULT 有效，立即关 PWM 和 EN_GATE
- ADC 无效，不进入闭环
- 编码器无效，不进入闭环
- 未校准，不进入闭环
- 第一次闭环 `current_limit <= 0.5A`，`voltage_limit <= 2V`
