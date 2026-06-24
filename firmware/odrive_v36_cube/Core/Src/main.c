/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/axis0_types.h"
#include "board/board_odrive_v36.h"
#include "config/axis0_default_config.h"
#include "drivers/drv8301.h"
#include "hal/hal_adc.h"
#include "hal/hal_gpio.h"
#include "hal/hal_pwm.h"
#include "protection/fault.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint32_t sample_ms;
  bool nfault_active;
  uint32_t nfault_pin_raw;
} NfaultTimedSample;

typedef struct {
  uint32_t sample_ms;
  Drv8301StatusReadback drv0;
  Drv8301StatusReadback drv1;
  bool nfault_active;
  uint32_t nfault_pin_raw;
} DrvStatusPollSample;

typedef struct {
  bool ran;
  bool pass;
  bool spi_ok;
  bool drv0_cfg;
  bool drv1_cfg;
  bool adc_ok;
  bool nfault_no_pull;
  bool nfault_with_pullup;
  uint32_t nfault_no_pull_raw;
  uint32_t nfault_with_pullup_raw;
  bool nfault_active_after_enable;
  uint32_t gate_pin_raw;
  uint32_t nfault_pin_raw;
  uint32_t nfault_release_ms;
  bool nfault_timeout;
  bool drv0_spi_ok;
  bool drv1_spi_ok;
  bool drv0_status_fault;
  bool drv1_status_fault;
  bool drv0_gate_reset_ok;
  bool drv1_gate_reset_ok;
  bool nfault_after_gate_reset;
  uint32_t nfault_after_gate_reset_raw;
  Drv8301StatusReadback drv0_after_gate_reset;
  Drv8301StatusReadback drv1_after_gate_reset;
  NfaultTimedSample nfault_samples[5];
  DrvStatusPollSample status_poll[10];
  uint32_t status_poll_count;
  uint32_t gpio_b_idr;
  uint32_t gpio_b_odr;
  uint32_t gpio_d_idr;
  uint32_t gpio_pb12_raw;
  uint32_t gpio_pd2_raw;
  bool gpio_nfault_active;
  bool gpio_polarity_ok;
  bool adc_seq_growing;
  uint32_t adc_seq_before;
  uint32_t adc_seq_after;
  Drv8301StatusReadback drv0_status_rb;
  Drv8301StatusReadback drv1_status_rb;
  uint32_t fail_step;
  Drv8301Registers drv0_regs;
  Drv8301Registers drv1_regs;
  uint32_t samples;
  uint32_t offset_u;
  uint32_t offset_v;
  uint16_t u_min;
  uint16_t u_max;
  uint16_t v_min;
  uint16_t v_max;
  uint16_t u_noise_pp;
  uint16_t v_noise_pp;
  bool final_gate;
  bool final_nfault;
  uint32_t final_ccer;
  uint32_t final_bdtr;
} DrvBringupTestResult;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_PRINT_TIMEOUT_MS 100u
#define BRINGUP_PRINT_PERIOD_MS 1000u
#define DRV_OFFSET_SAMPLE_COUNT 4096u
#define DRV_OFFSET_TIMEOUT_MS 1000u
#define DRV_NFAULT_RELEASE_TIMEOUT_MS 50u
#define DRV_NFAULT_NOT_RELEASED_MS 0xffffffffu
#define DRV_NFAULT_RESET_TEST_TIMEOUT_MS 200u
#define DRV_STATUS_POLL_COUNT 10u
#define DRV_STATUS_POLL_PERIOD_MS 100u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static Axis0Context g_axis0;
static Drv8301 g_drv0;
static Drv8301 g_drv1;
static DrvBringupTestResult g_drv_test;
static volatile uint32_t g_adc_callback_count = 0u;
static bool g_adc_init_ok = false;
static bool g_board_init_ok = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static void uart2_write(const char *text);
static void uart2_printf_line(const char *text);
static void axis0_context_init_minimal(void);
static void print_bringup_status(void);
static void drv_bringup_test_run(void);
static void print_drv_bringup_test_status(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void uart2_write(const char *text)
{
  if (text == NULL) {
    return;
  }

  const size_t len = strlen(text);
  if (len == 0u) {
    return;
  }

  (void)HAL_UART_Transmit(&huart2,
                          (uint8_t *)text,
                          (uint16_t)len,
                          UART_PRINT_TIMEOUT_MS);
}

static void uart2_printf_line(const char *text)
{
  uart2_write(text);
  uart2_write("\r\n");
}

static void axis0_context_init_minimal(void)
{
  memset(&g_axis0, 0, sizeof(g_axis0));

  g_axis0.config = axis0_default_config_make();
  g_axis0.state = AXIS0_STATE_BOOT;
  g_axis0.requested_state = AXIS0_STATE_IDLE;
  g_axis0.request_pending = false;
  g_axis0.cmd.control_mode = AXIS0_CONTROL_MODE_IDLE;
  g_axis0.cmd.id_target_a = 0.0f;
  g_axis0.cmd.iq_target_a = 0.0f;
}

static void drv_bringup_force_safe_off(void)
{
  hal_pwm_disable();
  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
  hal_gpio_set_gate_enable(false);
}

static bool drv_bringup_power_channels_off(void)
{
  const uint32_t power_ccer_mask = TIM_CCER_CC1E | TIM_CCER_CC1NE |
                                   TIM_CCER_CC2E | TIM_CCER_CC2NE |
                                   TIM_CCER_CC3E | TIM_CCER_CC3NE;
  return ((TIM1->BDTR & TIM_BDTR_MOE) == 0u) &&
         ((TIM1->CCER & power_ccer_mask) == 0u);
}

static void drv_bringup_capture_final_state(void)
{
  g_drv_test.final_gate = HAL_GPIO_ReadPin(EN_GATE_GPIO_Port, EN_GATE_Pin) == GPIO_PIN_SET;
  g_drv_test.final_nfault = board_read_drv_nfault();
  g_drv_test.final_ccer = TIM1->CCER;
  g_drv_test.final_bdtr = TIM1->BDTR;
}

static void drv_bringup_capture_raw_pins(void)
{
  g_drv_test.gate_pin_raw = (uint32_t)HAL_GPIO_ReadPin(EN_GATE_GPIO_Port, EN_GATE_Pin);
  g_drv_test.nfault_pin_raw = (uint32_t)HAL_GPIO_ReadPin(DRV_NFAULT_GPIO_Port, DRV_NFAULT_Pin);
}

static void drv_bringup_configure_nfault_pull(uint32_t pull)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = DRV_NFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = pull;
  HAL_GPIO_Init(DRV_NFAULT_GPIO_Port, &GPIO_InitStruct);
}

static void drv_bringup_capture_gpio_registers(void)
{
  g_drv_test.gpio_b_idr = GPIOB->IDR;
  g_drv_test.gpio_b_odr = GPIOB->ODR;
  g_drv_test.gpio_d_idr = GPIOD->IDR;
  g_drv_test.gpio_pb12_raw = (g_drv_test.gpio_b_idr & GPIO_PIN_12) ? 1u : 0u;
  g_drv_test.gpio_pd2_raw = (g_drv_test.gpio_d_idr & GPIO_PIN_2) ? 1u : 0u;
  g_drv_test.gpio_nfault_active = board_read_drv_nfault();
  g_drv_test.gpio_polarity_ok =
      (g_drv_test.gpio_nfault_active == (g_drv_test.gpio_pd2_raw == 0u));
}

static uint32_t drv_bringup_get_adc_seq(void)
{
  HalAdcSnapshot snap = {0};
  return hal_adc_get_snapshot(&snap) ? snap.seq : 0u;
}

static bool drv_bringup_drv_has_status_fault(const Drv8301StatusReadback *rb)
{
  const uint16_t status1 = rb->status1_data;
  const uint16_t status2 = rb->status2_data;
  return (status1 & 0x07ffu) != 0u || (status2 & 0x0080u) != 0u;
}

static void drv_bringup_mark_fault(Axis0FaultFlags fault)
{
  g_axis0.fault_flags |= (uint32_t)fault;
  g_axis0.state = AXIS0_STATE_FAULT;
}

static void drv_bringup_fail(uint32_t step)
{
  g_drv_test.fail_step = step;
  g_drv_test.pass = false;
  g_drv_test.adc_seq_after = drv_bringup_get_adc_seq();
  g_drv_test.adc_seq_growing = g_drv_test.adc_seq_after > g_drv_test.adc_seq_before;
  drv_bringup_force_safe_off();
  drv_bringup_configure_nfault_pull(GPIO_NOPULL);
  drv_bringup_capture_final_state();
}

static void drv_bringup_test_run(void)
{
  memset(&g_drv_test, 0, sizeof(g_drv_test));
  g_drv_test.ran = true;

  drv8301_prepare_axis(&g_drv0, 0u);
  drv8301_prepare_axis(&g_drv1, 1u);
  g_drv_test.nfault_release_ms = DRV_NFAULT_NOT_RELEASED_MS;

  drv_bringup_force_safe_off();
  drv_bringup_configure_nfault_pull(GPIO_NOPULL);
  HAL_Delay(1u);
  g_drv_test.nfault_no_pull = board_read_drv_nfault();
  g_drv_test.nfault_no_pull_raw = (uint32_t)HAL_GPIO_ReadPin(DRV_NFAULT_GPIO_Port,
                                                             DRV_NFAULT_Pin);

  drv_bringup_configure_nfault_pull(GPIO_PULLUP);
  HAL_Delay(1u);
  g_drv_test.nfault_with_pullup = board_read_drv_nfault();
  g_drv_test.nfault_with_pullup_raw = (uint32_t)HAL_GPIO_ReadPin(DRV_NFAULT_GPIO_Port,
                                                                 DRV_NFAULT_Pin);

  hal_pwm_start_adc_trigger_only();
  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
  g_drv_test.adc_seq_before = drv_bringup_get_adc_seq();

  if (!drv_bringup_power_channels_off()) {
    drv_bringup_mark_fault(AXIS0_FAULT_PWM_NOT_ENABLED);
    drv_bringup_fail(1u);
    return;
  }

  hal_gpio_set_gate_enable(false);
  HAL_Delay(2u);
  hal_gpio_set_gate_enable(true);
  g_drv0.enabled = true;
  g_drv1.enabled = true;

  const uint32_t sample_marks[5] = {1u, 10u, 50u, 100u, 200u};
  uint32_t sample_index = 0u;
  for (uint32_t ms = 1u; ms <= DRV_NFAULT_RESET_TEST_TIMEOUT_MS; ++ms) {
    HAL_Delay(1u);
    drv_bringup_capture_raw_pins();
    const bool nfault_active = board_read_drv_nfault();

    if (sample_index < 5u && ms == sample_marks[sample_index]) {
      g_drv_test.nfault_samples[sample_index].sample_ms = ms;
      g_drv_test.nfault_samples[sample_index].nfault_active = nfault_active;
      g_drv_test.nfault_samples[sample_index].nfault_pin_raw =
          (uint32_t)HAL_GPIO_ReadPin(DRV_NFAULT_GPIO_Port, DRV_NFAULT_Pin);
      sample_index++;
    }

    if (nfault_active) {
      drv_bringup_mark_fault(AXIS0_FAULT_DRV8301_FAULT);
    } else if (g_drv_test.nfault_release_ms == DRV_NFAULT_NOT_RELEASED_MS) {
      g_drv_test.nfault_release_ms = ms;
    }

    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
  }

  g_drv_test.nfault_active_after_enable = board_read_drv_nfault();
  g_drv_test.nfault_timeout = g_drv_test.nfault_active_after_enable;
  if (!drv_bringup_power_channels_off()) {
    drv_bringup_mark_fault(AXIS0_FAULT_PWM_NOT_ENABLED);
    drv_bringup_fail(2u);
    return;
  }

  g_drv_test.drv0_spi_ok = drv8301_read_status_raw(&g_drv0, &g_drv_test.drv0_status_rb);
  g_drv_test.drv1_spi_ok = drv8301_read_status_raw(&g_drv1, &g_drv_test.drv1_status_rb);
  g_drv_test.spi_ok = g_drv_test.drv0_spi_ok && g_drv_test.drv1_spi_ok;
  g_drv_test.drv0_status_fault = g_drv_test.drv0_spi_ok &&
                                 drv_bringup_drv_has_status_fault(&g_drv_test.drv0_status_rb);
  g_drv_test.drv1_status_fault = g_drv_test.drv1_spi_ok &&
                                 drv_bringup_drv_has_status_fault(&g_drv_test.drv1_status_rb);

  if (!g_drv_test.drv0_spi_ok) {
    drv_bringup_mark_fault(AXIS0_FAULT_DRV8301_SPI_ERROR);
    drv_bringup_fail(3u);
    return;
  }
  if (!g_drv_test.drv1_spi_ok) {
    drv_bringup_mark_fault(AXIS0_FAULT_DRV8301_SPI_ERROR);
    drv_bringup_fail(4u);
    return;
  }

  g_drv_test.drv0_gate_reset_ok = drv8301_gate_reset(&g_drv0);
  g_drv_test.drv1_gate_reset_ok = drv8301_gate_reset(&g_drv1);
  HAL_Delay(2u);
  g_drv_test.nfault_after_gate_reset = board_read_drv_nfault();
  g_drv_test.nfault_after_gate_reset_raw =
      (uint32_t)HAL_GPIO_ReadPin(DRV_NFAULT_GPIO_Port, DRV_NFAULT_Pin);
  (void)drv8301_read_status_raw(&g_drv0, &g_drv_test.drv0_after_gate_reset);
  (void)drv8301_read_status_raw(&g_drv1, &g_drv_test.drv1_after_gate_reset);
  if (g_drv_test.nfault_after_gate_reset) {
    drv_bringup_mark_fault(AXIS0_FAULT_DRV8301_FAULT);
  }
  if (!g_drv_test.drv0_gate_reset_ok || !g_drv_test.drv1_gate_reset_ok) {
    drv_bringup_mark_fault(AXIS0_FAULT_DRV8301_SPI_ERROR);
  }
  if (!drv_bringup_power_channels_off()) {
    drv_bringup_mark_fault(AXIS0_FAULT_PWM_NOT_ENABLED);
    drv_bringup_fail(13u);
    return;
  }

  for (uint32_t i = 0u; i < DRV_STATUS_POLL_COUNT; ++i) {
    HAL_Delay(DRV_STATUS_POLL_PERIOD_MS);
    g_drv_test.status_poll[i].sample_ms = (i + 1u) * DRV_STATUS_POLL_PERIOD_MS;
    (void)drv8301_read_status_raw(&g_drv0, &g_drv_test.status_poll[i].drv0);
    (void)drv8301_read_status_raw(&g_drv1, &g_drv_test.status_poll[i].drv1);
    g_drv_test.status_poll[i].nfault_active = board_read_drv_nfault();
    g_drv_test.status_poll[i].nfault_pin_raw =
        (uint32_t)HAL_GPIO_ReadPin(DRV_NFAULT_GPIO_Port, DRV_NFAULT_Pin);
    if (g_drv_test.status_poll[i].nfault_active) {
      drv_bringup_mark_fault(AXIS0_FAULT_DRV8301_FAULT);
    }
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    if (!drv_bringup_power_channels_off()) {
      drv_bringup_mark_fault(AXIS0_FAULT_PWM_NOT_ENABLED);
      drv_bringup_fail(14u);
      return;
    }
  }
  g_drv_test.status_poll_count = DRV_STATUS_POLL_COUNT;

  drv_bringup_capture_gpio_registers();

  if (g_drv_test.drv0_status_fault || g_drv_test.drv1_status_fault) {
    drv_bringup_mark_fault(AXIS0_FAULT_DRV8301_FAULT);
    drv_bringup_fail(5u);
    return;
  }
  if (g_drv_test.nfault_timeout) {
    drv_bringup_mark_fault(AXIS0_FAULT_DRV8301_FAULT);
    drv_bringup_fail(6u);
    return;
  }

  g_drv_test.adc_seq_after = drv_bringup_get_adc_seq();
  g_drv_test.adc_seq_growing = g_drv_test.adc_seq_after > g_drv_test.adc_seq_before;
  if (!g_drv_test.adc_seq_growing) {
    drv_bringup_fail(12u);
    return;
  }

  g_drv_test.pass = g_drv_test.spi_ok &&
                    g_drv_test.drv0_gate_reset_ok &&
                    g_drv_test.drv1_gate_reset_ok &&
                    !g_drv_test.nfault_after_gate_reset &&
                    !board_read_drv_nfault();

  drv_bringup_force_safe_off();
  drv_bringup_configure_nfault_pull(GPIO_NOPULL);
  drv_bringup_capture_final_state();
}

static void print_bringup_status(void)
{
  char line[320];

  HalAdcSnapshot snap = {0};
  HalAdcDiagnostics adc_diag = {0};
  HalPwmDiagnostics pwm_diag = {0};
  const bool snap_ok = hal_adc_get_snapshot(&snap);
  const BoardOdriveV36Status board_status = board_get_status();
  const unsigned int enc_cnt = (unsigned int)__HAL_TIM_GET_COUNTER(&htim3);
  hal_adc_get_diagnostics(&adc_diag);
  hal_pwm_get_diagnostics(&pwm_diag);

  snprintf(line,
           sizeof(line),
           "bringup: adc_init=%u board_init=%u irq=%lu cb=%lu cb1=%lu cb2=%lu snapcnt=%lu snap_ok=%u valid=%u seq=%lu raw_u=%u raw_v=%u raw_vbus=%u enc_cnt=%u nfault=%u gate=%u pwm_disabled=%u tim_base=%lu tim_oc4=%lu adc1_start=%lu adc2_start=%lu fault=0x%08lX",
           (unsigned int)g_adc_init_ok,
           (unsigned int)g_board_init_ok,
           (unsigned long)adc_diag.irq_count,
           (unsigned long)g_adc_callback_count,
           (unsigned long)adc_diag.adc1_callback_count,
           (unsigned long)adc_diag.adc2_callback_count,
           (unsigned long)adc_diag.snapshot_count,
           (unsigned int)snap_ok,
           (unsigned int)snap.valid,
           (unsigned long)snap.seq,
           (unsigned int)snap.raw_u,
           (unsigned int)snap.raw_v,
           (unsigned int)snap.raw_vbus,
           enc_cnt,
           (unsigned int)board_status.drv_nfault_active,
           (unsigned int)board_status.drv_gate_enabled,
           (unsigned int)board_status.pwm_disabled,
           (unsigned long)pwm_diag.base_start_status,
           (unsigned long)pwm_diag.oc4_start_status,
           (unsigned long)adc_diag.injected_start_adc1_status,
           (unsigned long)adc_diag.injected_start_adc2_status,
           (unsigned long)g_axis0.fault_flags);

  uart2_printf_line(line);

  snprintf(line,
           sizeof(line),
           "tim1: cr1=0x%08lX cnt=%lu arr=%lu ccr4=%lu ccer=0x%08lX sr=0x%08lX bdtr=0x%08lX",
           (unsigned long)TIM1->CR1,
           (unsigned long)TIM1->CNT,
           (unsigned long)TIM1->ARR,
           (unsigned long)TIM1->CCR4,
           (unsigned long)TIM1->CCER,
           (unsigned long)TIM1->SR,
           (unsigned long)TIM1->BDTR);
  uart2_printf_line(line);

  snprintf(line,
           sizeof(line),
           "adc1: cr1=0x%08lX cr2=0x%08lX sr=0x%08lX jsqr=0x%08lX",
           (unsigned long)ADC1->CR1,
           (unsigned long)ADC1->CR2,
           (unsigned long)ADC1->SR,
           (unsigned long)ADC1->JSQR);
  uart2_printf_line(line);

  snprintf(line,
           sizeof(line),
           "adc2: cr1=0x%08lX cr2=0x%08lX sr=0x%08lX jsqr=0x%08lX",
           (unsigned long)ADC2->CR1,
           (unsigned long)ADC2->CR2,
           (unsigned long)ADC2->SR,
           (unsigned long)ADC2->JSQR);
  uart2_printf_line(line);
}

static void print_drv_bringup_test_status(void)
{
  char line[320];

  snprintf(line,
           sizeof(line),
           "drvtest: ran=%u drv_test_pass=%u fail_step=%lu spi_ok=%u drv0_cfg=%u drv1_cfg=%u adc_ok=%u nfault_after_en=%u nfault_timeout=%u nfault_release_ms=%lu",
           (unsigned int)g_drv_test.ran,
           (unsigned int)g_drv_test.pass,
           (unsigned long)g_drv_test.fail_step,
           (unsigned int)g_drv_test.spi_ok,
           (unsigned int)g_drv_test.drv0_cfg,
           (unsigned int)g_drv_test.drv1_cfg,
           (unsigned int)g_drv_test.adc_ok,
           (unsigned int)g_drv_test.nfault_active_after_enable,
           (unsigned int)g_drv_test.nfault_timeout,
           (unsigned long)g_drv_test.nfault_release_ms);
  uart2_printf_line(line);

  snprintf(line,
           sizeof(line),
           "nfault_pull: nfault_no_pull=%u nfault_no_pull_raw=%lu nfault_with_pullup=%u nfault_with_pullup_raw=%lu",
           (unsigned int)g_drv_test.nfault_no_pull,
           (unsigned long)g_drv_test.nfault_no_pull_raw,
           (unsigned int)g_drv_test.nfault_with_pullup,
           (unsigned long)g_drv_test.nfault_with_pullup_raw);
  uart2_printf_line(line);

  snprintf(line,
           sizeof(line),
           "nfault_reset: t1=%u raw1=%lu t10=%u raw10=%lu t50=%u raw50=%lu t100=%u raw100=%lu t200=%u raw200=%lu",
           (unsigned int)g_drv_test.nfault_samples[0].nfault_active,
           (unsigned long)g_drv_test.nfault_samples[0].nfault_pin_raw,
           (unsigned int)g_drv_test.nfault_samples[1].nfault_active,
           (unsigned long)g_drv_test.nfault_samples[1].nfault_pin_raw,
           (unsigned int)g_drv_test.nfault_samples[2].nfault_active,
           (unsigned long)g_drv_test.nfault_samples[2].nfault_pin_raw,
           (unsigned int)g_drv_test.nfault_samples[3].nfault_active,
           (unsigned long)g_drv_test.nfault_samples[3].nfault_pin_raw,
           (unsigned int)g_drv_test.nfault_samples[4].nfault_active,
           (unsigned long)g_drv_test.nfault_samples[4].nfault_pin_raw);
  uart2_printf_line(line);

  snprintf(line,
           sizeof(line),
           "drvdiag: gate_pin_raw=%lu nfault_pin_raw=%lu drv0_spi_ok=%u drv1_spi_ok=%u drv0_status_fault=%u drv1_status_fault=%u adc_seq_before=%lu adc_seq_after=%lu adc_seq_growing=%u",
           (unsigned long)g_drv_test.gate_pin_raw,
           (unsigned long)g_drv_test.nfault_pin_raw,
           (unsigned int)g_drv_test.drv0_spi_ok,
           (unsigned int)g_drv_test.drv1_spi_ok,
           (unsigned int)g_drv_test.drv0_status_fault,
           (unsigned int)g_drv_test.drv1_status_fault,
           (unsigned long)g_drv_test.adc_seq_before,
           (unsigned long)g_drv_test.adc_seq_after,
           (unsigned int)g_drv_test.adc_seq_growing);
  uart2_printf_line(line);

  snprintf(line,
           sizeof(line),
           "gatereset: drv0_ok=%u drv1_ok=%u nfault=%u nfault_raw=%lu drv0_s1=0x%04X drv0_s2=0x%04X drv1_s1=0x%04X drv1_s2=0x%04X",
           (unsigned int)g_drv_test.drv0_gate_reset_ok,
           (unsigned int)g_drv_test.drv1_gate_reset_ok,
           (unsigned int)g_drv_test.nfault_after_gate_reset,
           (unsigned long)g_drv_test.nfault_after_gate_reset_raw,
           (unsigned int)g_drv_test.drv0_after_gate_reset.status1_frame,
           (unsigned int)g_drv_test.drv0_after_gate_reset.status2_frame,
           (unsigned int)g_drv_test.drv1_after_gate_reset.status1_frame,
           (unsigned int)g_drv_test.drv1_after_gate_reset.status2_frame);
  uart2_printf_line(line);

  snprintf(line,
           sizeof(line),
           "gpioreg: GPIOB_IDR=0x%08lX GPIOB_ODR=0x%08lX GPIOD_IDR=0x%08lX PB12=%lu PD2=%lu nfault_active=%u polarity_ok=%u",
           (unsigned long)g_drv_test.gpio_b_idr,
           (unsigned long)g_drv_test.gpio_b_odr,
           (unsigned long)g_drv_test.gpio_d_idr,
           (unsigned long)g_drv_test.gpio_pb12_raw,
           (unsigned long)g_drv_test.gpio_pd2_raw,
           (unsigned int)g_drv_test.gpio_nfault_active,
           (unsigned int)g_drv_test.gpio_polarity_ok);
  uart2_printf_line(line);

  snprintf(line,
           sizeof(line),
           "nfault_polarity: PD2=0=>nfault_active=1 PD2=1=>nfault_active=0 observed_PD2=%lu observed_nfault_active=%u ok=%u",
           (unsigned long)g_drv_test.gpio_pd2_raw,
           (unsigned int)g_drv_test.gpio_nfault_active,
           (unsigned int)g_drv_test.gpio_polarity_ok);
  uart2_printf_line(line);

  snprintf(line,
           sizeof(line),
           "drvfinal: gate=%u nfault=%u ccer=0x%08lX bdtr=0x%08lX",
           (unsigned int)g_drv_test.final_gate,
           (unsigned int)g_drv_test.final_nfault,
           (unsigned long)g_drv_test.final_ccer,
           (unsigned long)g_drv_test.final_bdtr);
  uart2_printf_line(line);

  snprintf(line,
           sizeof(line),
           "drv0: status1_ok=%u status1_frame=0x%04X status1_data=0x%04X status2_ok=%u status2_frame=0x%04X status2_data=0x%04X",
           (unsigned int)g_drv_test.drv0_status_rb.status1_ok,
           (unsigned int)g_drv_test.drv0_status_rb.status1_frame,
           (unsigned int)g_drv_test.drv0_status_rb.status1_data,
           (unsigned int)g_drv_test.drv0_status_rb.status2_ok,
           (unsigned int)g_drv_test.drv0_status_rb.status2_frame,
           (unsigned int)g_drv_test.drv0_status_rb.status2_data);
  uart2_printf_line(line);

  snprintf(line,
           sizeof(line),
           "drv1: status1_ok=%u status1_frame=0x%04X status1_data=0x%04X status2_ok=%u status2_frame=0x%04X status2_data=0x%04X",
           (unsigned int)g_drv_test.drv1_status_rb.status1_ok,
           (unsigned int)g_drv_test.drv1_status_rb.status1_frame,
           (unsigned int)g_drv_test.drv1_status_rb.status1_data,
           (unsigned int)g_drv_test.drv1_status_rb.status2_ok,
           (unsigned int)g_drv_test.drv1_status_rb.status2_frame,
           (unsigned int)g_drv_test.drv1_status_rb.status2_data);
  uart2_printf_line(line);

  for (uint32_t i = 0u; i < g_drv_test.status_poll_count; ++i) {
    snprintf(line,
             sizeof(line),
             "drvpoll: t=%lums nfault=%u raw=%lu drv0_s1=0x%04X drv0_s2=0x%04X drv1_s1=0x%04X drv1_s2=0x%04X",
             (unsigned long)g_drv_test.status_poll[i].sample_ms,
             (unsigned int)g_drv_test.status_poll[i].nfault_active,
             (unsigned long)g_drv_test.status_poll[i].nfault_pin_raw,
             (unsigned int)g_drv_test.status_poll[i].drv0.status1_frame,
             (unsigned int)g_drv_test.status_poll[i].drv0.status2_frame,
             (unsigned int)g_drv_test.status_poll[i].drv1.status1_frame,
             (unsigned int)g_drv_test.status_poll[i].drv1.status2_frame);
    uart2_printf_line(line);
  }

  snprintf(line,
           sizeof(line),
           "drvregs: expected_ctrl1=0x%04X expected_ctrl2=0x%04X drv0_control1=0x%04X drv0_control2=0x%04X",
           (unsigned int)drv8301_default_control1(),
           (unsigned int)drv8301_default_control2(),
           (unsigned int)g_drv_test.drv0_regs.control1,
           (unsigned int)g_drv_test.drv0_regs.control2);
  uart2_printf_line(line);

  snprintf(line,
           sizeof(line),
           "drvregs: drv1_control1=0x%04X drv1_control2=0x%04X",
           (unsigned int)g_drv_test.drv1_regs.control1,
           (unsigned int)g_drv_test.drv1_regs.control2);
  uart2_printf_line(line);

  snprintf(line,
           sizeof(line),
           "offset: samples=%lu offset_u=%lu offset_v=%lu u_min=%u u_max=%u v_min=%u v_max=%u u_noise_pp=%u v_noise_pp=%u",
           (unsigned long)g_drv_test.samples,
           (unsigned long)g_drv_test.offset_u,
           (unsigned long)g_drv_test.offset_v,
           (unsigned int)g_drv_test.u_min,
           (unsigned int)g_drv_test.u_max,
           (unsigned int)g_drv_test.v_min,
           (unsigned int)g_drv_test.v_max,
           (unsigned int)g_drv_test.u_noise_pp,
           (unsigned int)g_drv_test.v_noise_pp);
  uart2_printf_line(line);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  axis0_context_init_minimal();

  /*
   * 先启动 ADC injected interrupt，再启动 TIM1 trigger-only。
   * hal_adc_init() 只是让 ADC 等待外部触发；
   * board_init_power_safe() 会保持 EN_GATE=0/MOE=0，并启动 TIM1 CC4 触发 ADC。
   */
  g_adc_init_ok = hal_adc_init();
  g_board_init_ok = board_init_power_safe(&g_axis0);

  /*
   * 编码器计数器先启动，方便后面手转检查 TIM3 AB。
   * 这一步不使能功率级，不会驱动电机。
   */
  (void)HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  uart2_printf_line("");
  uart2_printf_line("odrive_v36_cube bringup start");
  uart2_printf_line("SAFE MODE: motor phases disconnected; DRV wake test uses EN_GATE=1 only with MOE=0 and power PWM off.");
  drv_bringup_test_run();
  print_drv_bringup_test_status();
  print_bringup_status();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_print_ms = HAL_GetTick();

  while (1)
  {
    const uint32_t now_ms = HAL_GetTick();

    if ((now_ms - last_print_ms) >= BRINGUP_PRINT_PERIOD_MS) {
      last_print_ms = now_ms;
      print_bringup_status();
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  hal_adc_stm32f405_on_injected_complete((void *)hadc);
  g_adc_callback_count++;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
