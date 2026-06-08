#include "drivers/drv8301.h"

#include <string.h>
#include "board/board_odrive_v36.h"
#include "hal/hal_gpio.h"
#include "hal/hal_pwm.h"
#include "hal/hal_spi.h"

/*
 * drv8301.c
 *
 * DRV8301 驱动 skeleton。
 * 当前 SPI 读写为 mock/占位；真实工程需要按 DRV8301 datasheet 实现寄存器读写。
 * 任何 nFAULT 或 SPI 错误都应由保护/状态机转入 FAULT。
 *
 * 与 PWM ISR 的关系：
 * - drv8301_read_status()/configure/clear_faults 都可能走阻塞 SPI，只能在启动阶段、
 *   低频状态机或后台任务中调用。
 * - PWM ISR 只允许读取已经缓存的 drv->status 和 nFAULT GPIO，不允许现场 SPI 读寄存器。
 *
 * EN_GATE 安全原则：
 * - 上电先 disable。
 * - 配置完成、PWM 已处于安全 duty、无 nFAULT 后才 enable。
 * - 一旦故障，先关 PWM，再拉低 EN_GATE。
 */

#define DRV8301_REG_STATUS1        0x00u
#define DRV8301_REG_STATUS2        0x01u
#define DRV8301_REG_CONTROL1       0x02u
#define DRV8301_REG_CONTROL2       0x03u

#define DRV8301_SPI_READ           0x8000u
#define DRV8301_ADDR_SHIFT         11u
#define DRV8301_DATA_MASK          0x07ffu

#define DRV8301_STATUS1_FAULT      (1u << 10)
#define DRV8301_STATUS1_GVDD_UV    (1u << 9)
#define DRV8301_STATUS1_PVDD_UV    (1u << 8)
#define DRV8301_STATUS1_OTSD       (1u << 7)
#define DRV8301_STATUS1_OTW        (1u << 6)
#define DRV8301_STATUS1_OC_MASK    0x003fu
#define DRV8301_STATUS2_GVDD_OV    (1u << 7)

#define DRV8301_CTRL1_GATE_CURRENT_SHIFT 0u
#define DRV8301_CTRL1_GATE_RESET         (1u << 2)
#define DRV8301_CTRL1_OCP_MODE_SHIFT     4u
#define DRV8301_CTRL1_OC_ADJ_SHIFT       6u
#define DRV8301_CTRL2_GAIN_SHIFT         2u

static float drv8301_gain_from_code(uint8_t gain_code)
{
    switch (gain_code & 0x03u) {
    case 0u: return 10.0f;
    case 1u: return 20.0f;
    case 2u: return 40.0f;
    default: return 80.0f;
    }
}

static uint16_t drv8301_make_read(uint8_t addr)
{
    return DRV8301_SPI_READ | ((uint16_t)addr << DRV8301_ADDR_SHIFT);
}

static uint16_t drv8301_make_write(uint8_t addr, uint16_t data)
{
    return ((uint16_t)addr << DRV8301_ADDR_SHIFT) | (data & DRV8301_DATA_MASK);
}

static bool drv8301_spi_transfer16(const Drv8301 *drv, uint16_t tx, uint16_t *rx)
{
    /*
     * DRV8301 SPI 一帧为 16 bit。
     * 当前通过 hal_spi_transfer 做同步传输；真实工程可改为：
     * - 初始化/后台使用阻塞 SPI；
     * - 或使用 DMA + 完成标志，但绝不能在 20kHz ISR 中等待完成。
     */
    uint8_t tx_buf[2] = { (uint8_t)(tx >> 8), (uint8_t)(tx & 0xffu) };
    uint8_t rx_buf[2] = { 0u, 0u };
    if (!hal_spi_transfer_device(3u, drv->spi_device_id, tx_buf, rx_buf, sizeof(tx_buf))) {
        return false;
    }
    *rx = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
    return true;
}

static bool drv8301_write_reg(const Drv8301 *drv, uint8_t addr, uint16_t data)
{
    uint16_t rx = 0u;
    return drv8301_spi_transfer16(drv, drv8301_make_write(addr, data), &rx);
}

static bool drv8301_read_reg(const Drv8301 *drv, uint8_t addr, uint16_t *data)
{
    uint16_t ignored = 0u;
    uint16_t rx = 0u;

    /*
     * DRV8301 的 SPI 读数据延迟一帧返回：
     * 第 1 帧发送 read command，第 2 帧发送任意安全命令并接收上一帧结果。
     */
    if (!drv8301_spi_transfer16(drv, drv8301_make_read(addr), &ignored)) {
        return false;
    }
    if (!drv8301_spi_transfer16(drv, drv8301_make_read(DRV8301_REG_STATUS1), &rx)) {
        return false;
    }
    *data = rx & DRV8301_DATA_MASK;
    return true;
}

static void drv8301_parse_status(Drv8301Status *status)
{
    const uint16_t s1 = status->status1_raw & DRV8301_DATA_MASK;
    const uint16_t s2 = status->status2_raw & DRV8301_DATA_MASK;

    status->over_current = (s1 & DRV8301_STATUS1_OC_MASK) != 0u;
    status->over_temperature = (s1 & (DRV8301_STATUS1_OTSD | DRV8301_STATUS1_OTW)) != 0u;
    status->under_voltage = (s1 & (DRV8301_STATUS1_GVDD_UV | DRV8301_STATUS1_PVDD_UV)) != 0u;
    status->gate_driver_fault = (s1 & DRV8301_STATUS1_FAULT) != 0u;
    status->phase_fault = status->over_current;
    status->unknown_fault = (s2 & DRV8301_STATUS2_GVDD_OV) != 0u;
}

bool drv8301_init(Drv8301 *drv)
{
    return drv8301_init_axis(drv, 0u);
}

bool drv8301_init_axis(Drv8301 *drv, uint8_t axis_index)
{
    /*
     * 初始化顺序：
     * 1. 清软件状态；
     * 2. 明确关闭 gate；
     * 3. 通过 SPI 写入保守配置。
     * 如果第 3 步失败，initialized=false，状态机不应允许闭环。
     */
    memset(drv, 0, sizeof(*drv));
    drv->axis_index = axis_index;
    drv->spi_device_id = axis_index;
    drv->shunt_amp_gain_v_v = 10.0f;
    drv8301_disable(drv);
    drv->initialized = drv8301_configure_for_6pwm(drv);
    return drv->initialized;
}

bool drv8301_enable(Drv8301 *drv)
{
    /*
     * 使能前必须确保 PWM 关闭或处于安全 duty，避免 EN_GATE 瞬间产生异常输出。
     * 这里使用 hal_pwm_is_enabled() 做简单门控：
     * - 当前 skeleton 要求 enable 前 PWM 尚未真正打开；
     * - board_enable_axis0_power_stage() 会按 duty=50% -> EN_GATE -> PWM enable 的顺序执行。
     */
    if (!hal_pwm_is_enabled() && !board_read_drv_nfault() && drv->initialized) {
        hal_gpio_set_gate_enable(true);
        drv->enabled = true;
        return true;
    }
    return false;
}

void drv8301_disable(Drv8301 *drv)
{
    /*
     * 故障安全关断路径：
     * 1. 禁止 PWM 更新；
     * 2. 三相置安全低电平；
     * 3. 拉低 EN_GATE。
     */
    hal_pwm_disable();
    hal_pwm_set_all_low();
    hal_gpio_set_gate_enable(false);
    drv->enabled = false;
}

bool drv8301_read_status(Drv8301 *drv)
{
    /*
     * 读取状态寄存器。
     * 注意：此函数不能在 PWM ISR 中调用，因为 SPI 传输可能阻塞。
     * 建议在 100Hz 后台任务中周期读取，并把结果缓存到 drv->status。
     */
    uint16_t status1 = 0u;
    uint16_t status2 = 0u;
    if (!drv8301_read_reg(drv, DRV8301_REG_STATUS1, &status1) ||
        !drv8301_read_reg(drv, DRV8301_REG_STATUS2, &status2)) {
        drv->status.spi_error = true;
        return false;
    }

    drv->status.status1_raw = status1;
    drv->status.status2_raw = status2;
    drv->status.spi_error = false;
    drv8301_parse_status(&drv->status);
    return true;
}

bool drv8301_clear_faults(Drv8301 *drv)
{
    /*
     * 清故障应在功率级关闭状态下执行。
     * 如果 nFAULT 持续存在，清故障不会成功，状态机应保持 FAULT。
     */
    drv->status.unknown_fault = false;
    drv->status.spi_error = false;
    if (!drv8301_write_reg(drv, DRV8301_REG_CONTROL1, DRV8301_CTRL1_GATE_RESET)) {
        drv->status.spi_error = true;
        return false;
    }
    return drv8301_read_status(drv);
}

bool drv8301_configure_for_6pwm(Drv8301 *drv)
{
    /*
     * ODrive v3.6 以 6PWM 方式驱动三相桥。
     * 真实实现需要写 DRV8301 控制寄存器：
     * - PWM mode / gate drive current；
     * - OCP 模式和阈值；
     * - OC latch/报告方式；
     * - 放大器增益等。
     */
    const uint16_t control1 =
        ((uint16_t)DRV8301_GATE_CURRENT_0P25A << DRV8301_CTRL1_GATE_CURRENT_SHIFT) |
        ((uint16_t)DRV8301_OCP_MODE_CURRENT_LIMIT << DRV8301_CTRL1_OCP_MODE_SHIFT) |
        ((uint16_t)8u << DRV8301_CTRL1_OC_ADJ_SHIFT);
    const uint16_t control2 = (0u << DRV8301_CTRL2_GAIN_SHIFT);

    if (!drv8301_write_reg(drv, DRV8301_REG_CONTROL1, control1) ||
        !drv8301_write_reg(drv, DRV8301_REG_CONTROL2, control2)) {
        drv->status.spi_error = true;
        return false;
    }
    drv->status.spi_error = false;
    drv->shunt_amp_gain_v_v = 10.0f;
    return true;
}

bool drv8301_set_ocp_threshold(Drv8301 *drv, uint8_t threshold_code)
{
    /*
     * OCP 阈值越低越安全，但过低可能导致启动/校准误报。
     * 第一阶段建议使用保守低电流，并让 MCU 侧 overcurrent 阈值先发挥保护作用。
     */
    const uint16_t code = (uint16_t)(threshold_code & 0x1fu);
    const uint16_t control1 =
        ((uint16_t)DRV8301_GATE_CURRENT_0P25A << DRV8301_CTRL1_GATE_CURRENT_SHIFT) |
        ((uint16_t)DRV8301_OCP_MODE_CURRENT_LIMIT << DRV8301_CTRL1_OCP_MODE_SHIFT) |
        (code << DRV8301_CTRL1_OC_ADJ_SHIFT);

    if (!drv8301_write_reg(drv, DRV8301_REG_CONTROL1, control1)) {
        drv->status.spi_error = true;
        return false;
    }
    drv->status.spi_error = false;
    return true;
}

bool drv8301_set_gate_current(Drv8301 *drv, uint8_t gate_current_code)
{
    /*
     * gate current 影响 MOSFET 开关速度、损耗和 EMI。
     * 第一阶段不追求性能，应选择保守配置，避免过快开关带来尖峰。
     */
    const uint16_t code = (uint16_t)(gate_current_code & 0x03u);
    const uint16_t control1 =
        (code << DRV8301_CTRL1_GATE_CURRENT_SHIFT) |
        ((uint16_t)DRV8301_OCP_MODE_CURRENT_LIMIT << DRV8301_CTRL1_OCP_MODE_SHIFT) |
        ((uint16_t)8u << DRV8301_CTRL1_OC_ADJ_SHIFT);

    if (!drv8301_write_reg(drv, DRV8301_REG_CONTROL1, control1)) {
        drv->status.spi_error = true;
        return false;
    }
    drv->status.spi_error = false;
    return true;
}

bool drv8301_set_shunt_amp_gain(Drv8301 *drv, uint8_t gain_code)
{
    const uint16_t code = (uint16_t)(gain_code & 0x03u);
    const uint16_t control2 = code << DRV8301_CTRL2_GAIN_SHIFT;

    if (!drv8301_write_reg(drv, DRV8301_REG_CONTROL2, control2)) {
        drv->status.spi_error = true;
        return false;
    }

    drv->shunt_amp_gain_v_v = drv8301_gain_from_code(gain_code);
    drv->status.spi_error = false;
    return true;
}

bool drv8301_has_fault(const Drv8301 *drv)
{
    /*
     * 快速路径使用：
     * - nFAULT GPIO：硬件实时故障；
     * - drv->status：后台缓存的 SPI 状态。
     */
    return board_read_drv_nfault() ||
           drv->status.over_current ||
           drv->status.over_temperature ||
           drv->status.under_voltage ||
           drv->status.gate_driver_fault ||
           drv->status.phase_fault ||
           drv->status.unknown_fault ||
           drv->status.spi_error;
}
