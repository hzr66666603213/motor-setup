#include "hal/hal_spi.h"

/*
 * hal_spi.c
 *
 * SPI mock/stub 实现。
 * 当前将 tx 回环到 rx，便于上层接口测试。
 * 移植时应替换为 STM32 SPI HAL/LL 或 DMA 后端。
 */

bool hal_spi_transfer(uint8_t bus_id, const uint8_t *tx, uint8_t *rx, size_t length)
{
    (void)bus_id;
    for (size_t i = 0u; i < length; ++i) {
        if (rx) {
            rx[i] = tx ? tx[i] : 0u;
        }
    }
    return true;
}

bool hal_spi_transfer_device(uint8_t bus_id, uint8_t device_id, const uint8_t *tx, uint8_t *rx, size_t length)
{
    /*
     * device_id 用于同一 SPI 总线上的不同片选：
     * - ODrive v3.6 DRV8301 M0 CS = PC13
     * - ODrive v3.6 DRV8301 M1 CS = PC14
     * mock 中不真正拉片选，只保留接口形状。
     */
    (void)device_id;
    return hal_spi_transfer(bus_id, tx, rx, length);
}

bool hal_spi_transfer16_device(uint8_t bus_id, uint8_t device_id, const uint16_t *tx, uint16_t *rx, size_t word_count)
{
    (void)bus_id;
    (void)device_id;
    for (size_t i = 0u; i < word_count; ++i) {
        if (rx) {
            rx[i] = tx ? tx[i] : 0u;
        }
    }
    return true;
}

bool hal_spi_transfer_dma(uint8_t bus_id, const uint8_t *tx, uint8_t *rx, size_t length)
{
    /* mock 中 DMA 与普通传输等价；真实工程应异步返回并由回调置位完成标志。 */
    return hal_spi_transfer(bus_id, tx, rx, length);
}
