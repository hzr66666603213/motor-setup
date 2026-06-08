#include "hal/hal_spi.h"

/*
 * hal_spi_mock.c
 *
 * SPI mock 后端。
 * tx 回环到 rx，用于验证上层协议和 DRV8301 寄存器打包逻辑。
 */

bool hal_spi_transfer(uint8_t bus_id, const uint8_t *tx, uint8_t *rx, size_t length)
{
    (void)bus_id;
    for (size_t i = 0u; i < length; ++i) {
        if (rx != 0) {
            rx[i] = (tx != 0) ? tx[i] : 0u;
        }
    }
    return true;
}

bool hal_spi_transfer_device(uint8_t bus_id, uint8_t device_id, const uint8_t *tx, uint8_t *rx, size_t length)
{
    (void)device_id;
    return hal_spi_transfer(bus_id, tx, rx, length);
}

bool hal_spi_transfer_dma(uint8_t bus_id, const uint8_t *tx, uint8_t *rx, size_t length)
{
    return hal_spi_transfer(bus_id, tx, rx, length);
}
