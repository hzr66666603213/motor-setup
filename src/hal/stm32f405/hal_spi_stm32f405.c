#include "hal/hal_spi.h"

/*
 * hal_spi_stm32f405.c
 *
 * ODrive v3.6 DRV8301 SPI3 真实后端。
 *
 * 资源假设：
 * - SPI3_SCK  = PC10
 * - SPI3_MISO = PC11
 * - SPI3_MOSI = PC12
 * - DRV0_CS   = PC13
 * - DRV1_CS   = PC14
 *
 * 约束：
 * - 该阻塞式接口只能在启动阶段、状态机或后台任务调用；
 * - 禁止在 20 kHz PWM ISR 中读写 DRV8301 SPI 寄存器；
 * - Axis0-only 也要能访问 DRV1，因为 EN_GATE/nFAULT 是 M0/M1 共用资源。
 */

#include "stm32f4xx_hal.h"

extern SPI_HandleTypeDef hspi3;

#define ODRV36_DRV0_CS_GPIO_Port GPIOC
#define ODRV36_DRV0_CS_Pin       GPIO_PIN_13
#define ODRV36_DRV1_CS_GPIO_Port GPIOC
#define ODRV36_DRV1_CS_Pin       GPIO_PIN_14

#define HAL_SPI_BUS_DRV8301      3u
#define HAL_SPI_DEVICE_DRV0      0u
#define HAL_SPI_DEVICE_DRV1      1u
#define HAL_SPI_TIMEOUT_MS       10u

static bool select_device(uint8_t device_id, GPIO_TypeDef **port, uint16_t *pin)
{
    if (device_id == HAL_SPI_DEVICE_DRV0) {
        *port = ODRV36_DRV0_CS_GPIO_Port;
        *pin = ODRV36_DRV0_CS_Pin;
        return true;
    }
    if (device_id == HAL_SPI_DEVICE_DRV1) {
        *port = ODRV36_DRV1_CS_GPIO_Port;
        *pin = ODRV36_DRV1_CS_Pin;
        return true;
    }
    return false;
}

bool hal_spi_transfer(uint8_t bus_id, const uint8_t *tx, uint8_t *rx, size_t length)
{
    if (bus_id != HAL_SPI_BUS_DRV8301 || tx == 0 || rx == 0 || length == 0u) {
        return false;
    }

    return HAL_SPI_TransmitReceive(&hspi3,
                                   (uint8_t *)tx,
                                   rx,
                                   (uint16_t)length,
                                   HAL_SPI_TIMEOUT_MS) == HAL_OK;
}

bool hal_spi_transfer_device(uint8_t bus_id, uint8_t device_id, const uint8_t *tx, uint8_t *rx, size_t length)
{
    GPIO_TypeDef *cs_port = 0;
    uint16_t cs_pin = 0u;
    bool ok = false;

    if (!select_device(device_id, &cs_port, &cs_pin)) {
        return false;
    }

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    ok = hal_spi_transfer(bus_id, tx, rx, length);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    return ok;
}

bool hal_spi_transfer16_device(uint8_t bus_id, uint8_t device_id, const uint16_t *tx, uint16_t *rx, size_t word_count)
{
    GPIO_TypeDef *cs_port = 0;
    uint16_t cs_pin = 0u;
    bool ok = false;

    if (bus_id != HAL_SPI_BUS_DRV8301 || tx == 0 || rx == 0 || word_count == 0u) {
        return false;
    }
    if (!select_device(device_id, &cs_port, &cs_pin)) {
        return false;
    }

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    ok = HAL_SPI_TransmitReceive(&hspi3,
                                  (uint8_t *)tx,
                                  (uint8_t *)rx,
                                  (uint16_t)word_count,
                                  HAL_SPI_TIMEOUT_MS) == HAL_OK;
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    return ok;
}

bool hal_spi_transfer_dma(uint8_t bus_id, const uint8_t *tx, uint8_t *rx, size_t length)
{
    if (bus_id != HAL_SPI_BUS_DRV8301 || tx == 0 || rx == 0 || length == 0u) {
        return false;
    }

    return HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t *)tx, rx, (uint16_t)length) == HAL_OK;
}
