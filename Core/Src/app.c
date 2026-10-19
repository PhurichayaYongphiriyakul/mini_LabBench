/**
 * app.c - Glue: startup, TIM6 sample-rate ISR, ADC pair read, debug info
 *
 * Every TIM6 tick (2 kHz):
 *   1. read ADC rank 1 (x = source sine) and rank 2 (y = LM358 output)
 *   2. hand the pair to the lock-in together with the generator phase
 *   3. advance the generator and write the next DAC value
 */
#include "app.h"
#include "app_config.h"
#include "siggen.h"
#include "lockin.h"
#include "console.h"
#include <stdio.h>

static volatile uint16_t last_x, last_y;
static volatile uint32_t isr_count;
static volatile uint32_t adc_errors;

static uint32_t tim6_update_hz(void)
{
  uint32_t pclk1  = HAL_RCC_GetPCLK1Freq();
  uint32_t timclk = (READ_BIT(RCC->CFGR, RCC_CFGR_PPRE1) == RCC_HCLK_DIV1) ? pclk1 : 2u * pclk1;
  return timclk / ((htim6.Init.Prescaler + 1u) * (htim6.Init.Period + 1u));
}

/* Read both ranks of one scan. Returns 1 on success. */
static int adc_read_pair(uint16_t *x, uint16_t *y)
{
  if (HAL_ADC_Start(&hadc1) != HAL_OK)               return 0;
  if (HAL_ADC_PollForConversion(&hadc1, 1) != HAL_OK) return 0;
  *x = (uint16_t)HAL_ADC_GetValue(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 1) != HAL_OK) return 0;
  *y = (uint16_t)HAL_ADC_GetValue(&hadc1);
  return 1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM6) return;
  isr_count++;

  uint16_t x, y;
  if (adc_read_pair(&x, &y)) {
    last_x = x;
    last_y = y;
    lockin_on_sample(x, y, siggen_index());
  } else {
    adc_errors++;
  }

  siggen_tick();
}

void app_init(void)
{
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  siggen_init();
  HAL_TIM_Base_Start_IT(&htim6);

  printf("\r\n\r\nELEC2104 TestBench booted\r\n");
  app_print_info();
  console_menu();
}

void app_poll(void)
{
  console_poll();
}

void app_print_info(void)
{
  uint32_t c0 = isr_count;
  HAL_Delay(1000);
  uint32_t c1 = isr_count;

  printf("\r\n--- info ---\r\n");
  printf("SYSCLK          : %lu Hz\r\n", (unsigned long)HAL_RCC_GetSysClockFreq());
  printf("TIM6 (computed) : %lu Hz   (need 2000)\r\n", (unsigned long)tim6_update_hz());
  printf("TIM6 (measured) : %lu ISR/s (need ~2000; 0 = interrupt not enabled)\r\n",
         (unsigned long)(c1 - c0));
  printf("ADC errors      : %lu (should stay 0)\r\n", (unsigned long)adc_errors);
  printf("Generator       : %.2f Hz, amp %.0f counts, Vin ~ %.2f mVpp at node X\r\n",
         siggen_get_freq(), siggen_get_amp(), siggen_vin_mvpp());
  printf("Attenuator K    : 1/%.1f\r\n", 1.0f / K_ATT);
}

void app_print_raw(int n)
{
  printf("\r\n  x_pin(mV)  y_pin(mV)  y_node(mV)\r\n");
  for (int i = 0; i < n; i++) {
    uint16_t x = last_x, y = last_y;
    float xm = x * VDDA_MV / 4095.0f;
    float ym = y * VDDA_MV / 4095.0f;
    printf("  %9.0f  %9.0f  %10.0f\r\n", xm, ym, ym * OUT_DIV);
    HAL_Delay(50);
  }
}
