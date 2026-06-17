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
#include "hal/hal_adc.h"
#include "hal/hal_pwm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_PRINT_TIMEOUT_MS 100u
#define BRINGUP_PRINT_PERIOD_MS 1000u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static Axis0Context g_axis0;
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

static void print_bringup_status(void)
{
  char line[320];

  HalAdcSnapshot snap = {0};
  HalAdcDiagnostics adc_diag = {0};
  HalPwmDiagnostics pwm_diag = {0};
  const bool snap_ok = hal_adc_get_snapshot(&snap);
  const BoardOdriveV36Status board_status = board_get_status();
  hal_adc_get_diagnostics(&adc_diag);
  hal_pwm_get_diagnostics(&pwm_diag);

  snprintf(line,
           sizeof(line),
           "bringup: adc_init=%u board_init=%u irq=%lu cb=%lu cb1=%lu cb2=%lu snapcnt=%lu snap_ok=%u valid=%u seq=%lu raw_u=%u raw_v=%u raw_vbus=%u nfault=%u gate=%u pwm_disabled=%u tim_base=%lu tim_oc4=%lu adc1_start=%lu adc2_start=%lu fault=0x%08lX",
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
  uart2_printf_line("SAFE MODE: EN_GATE should stay LOW, MOE should stay OFF, motor must be disconnected.");
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
