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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ----- ELEC2104 Stage II test bench ---------------------------------------
 * PA4 DAC1_OUT1 : sine  --[R_SER]--+-- node X --> C1(+) of Stage II
 *                                  |
 *                               [R_SH]
 *                                  |
 * PA5 DAC1_OUT2 : DC 2.5 V --------+
 * PC0 (A5) ADC1_IN1 <- jumper from PA4 (measures the source sine)
 * PA3 (A0) ADC1_IN8 <- LM358 pin 1 via 10k/10k divider
 * ------------------------------------------------------------------------ */
#define FS_HZ        2000.0f     /* TIM6 update rate = sample rate          */
#define VDDA_MV      3300.0f     /* nominal analog supply                    */
#define DAC_MID      2048.0f     /* DAC1 centre ~1.65 V                      */
#define DAC2_DC_MV   2500.0f     /* DC bias at C1 input (like collector)     */

#define R_SER        20000.0f    /* put MEASURED values here                 */
#define R_SH         100.0f
#define K_ATT        (R_SH / (R_SER + R_SH))

#define OUT_DIV      2.0f        /* (10k + 10k) / 10k on LM358 output         */
#define OUT_CLIP_MV  3300.0f     /* LM358 @5V cannot swing much above ~3.5 V  */
#define OUT_FLOOR_MV 80.0f       /* ...or below ~50 mV                        */

#define N_FREQ 17
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
static const float freqs[N_FREQ] = {
  0.2f, 0.3f, 0.5f, 0.7f, 1.0f, 1.5f, 2.0f, 3.0f, 5.0f,
  7.0f, 10.0f, 15.0f, 20.0f, 30.0f, 50.0f, 70.0f, 100.0f
};

typedef struct {
  float f, gain, gain_db, phase_deg, vin_mvpp, vout_mvpp, vout_dc_mv;
  int   clipped;
} BodePoint;

static BodePoint pts[N_FREQ];
static float     sin_lut[256];

/* Lock-in state shared with the TIM6 ISR */
static volatile uint32_t phase_acc, tuning_word;
static volatile uint32_t n_settle, n_meas, n_count;
static volatile uint8_t  lockin_busy;
static volatile float    dac_amp = 620.0f;   /* +/-620 counts = 1 Vpp -> ~4.5 mVpp at node X */
static volatile uint16_t ymin, ymax;
static double sx, sy, sxs, sxc, sys, syc, ss, sc;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Route printf to the ST-LINK virtual COM port */
int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;
  HAL_UART_Transmit(&hlpuart1, &c, 1, HAL_MAX_DELAY);
  return ch;
}

static inline float cnt_to_mv(double cnt) { return (float)(cnt * VDDA_MV / 4095.0); }

static float vin_mvpp_setting(void)
{
  return 2.0f * dac_amp * (VDDA_MV / 4095.0f) * K_ATT;
}

/* Run the lock-in at one frequency and fill one BodePoint */
static BodePoint measure(float f)
{
  float settle_s = fmaxf(2.0f / f, 2.0f);              /* >= 2 cycles and >= 2 s */
  float cycles   = ceilf(fmaxf(4.0f, f * 1.0f));        /* >= 4 cycles and >= 1 s */

  __disable_irq();
  sx = sy = sxs = sxc = sys = syc = ss = sc = 0.0;
  ymin = 4095; ymax = 0;
  tuning_word = (uint32_t)((double)f / FS_HZ * 4294967296.0);
  n_settle    = (uint32_t)(settle_s * FS_HZ);
  n_meas      = (uint32_t)lroundf(cycles * FS_HZ / f);
  n_count     = 0;
  lockin_busy = 1;
  __enable_irq();

  while (lockin_busy) { }

  double N  = (double)n_meas;
  double mx = sx / N, my = sy / N;
  double Xi = sxs - mx * ss, Xq = sxc - mx * sc;        /* remove DC leakage */
  double Yi = sys - my * ss, Yq = syc - my * sc;
  double ax = 2.0 * sqrt(Xi * Xi + Xq * Xq) / N;        /* peak amplitude, counts */
  double ay = 2.0 * sqrt(Yi * Yi + Yq * Yq) / N;

  BodePoint p = {0};
  p.f          = f;
  p.gain       = (ax > 0.0) ? (float)((ay * OUT_DIV) / (ax * K_ATT)) : 0.0f;
  p.gain_db    = (p.gain > 0.0f) ? 20.0f * log10f(p.gain) : -999.0f;
  p.vin_mvpp   = 2.0f * cnt_to_mv(ax) * K_ATT;
  p.vout_mvpp  = 2.0f * cnt_to_mv(ay) * OUT_DIV;
  p.vout_dc_mv = cnt_to_mv(my) * OUT_DIV;

  float ph = (float)((atan2(Yq, Yi) - atan2(Xq, Xi)) * 180.0 / M_PI);
  while (ph > 180.0f)   ph -= 360.0f;
  while (ph <= -180.0f) ph += 360.0f;
  p.phase_deg = ph;

  float out_max = cnt_to_mv(ymax) * OUT_DIV, out_min = cnt_to_mv(ymin) * OUT_DIV;
  p.clipped = (out_max > OUT_CLIP_MV) || (out_min < OUT_FLOOR_MV);
  return p;
}

/* Log-interpolated frequency where gain crosses 'level' between i and j */
static float cross_freq(int i, int j, float level)
{
  float t = (level - pts[i].gain) / (pts[j].gain - pts[i].gain);
  return powf(10.0f, log10f(pts[i].f) + t * (log10f(pts[j].f) - log10f(pts[i].f)));
}

static void run_sweep(void)
{
  printf("\r\nSweep start: Vin ~ %.2f mVpp, %d points (low freqs are slow)\r\n",
         vin_mvpp_setting(), N_FREQ);
  printf("     f(Hz)    gain    dB     phase   Vin(mVpp) Vout(mVpp) Vout_DC(mV)\r\n");

  for (int k = 0; k < N_FREQ; k++) {
    pts[k] = measure(freqs[k]);
    BodePoint *p = &pts[k];
    printf("%10.2f %7.1f %6.1f %8.1f %10.2f %10.1f %10.0f %s\r\n",
           p->f, p->gain, p->gain_db, p->phase_deg,
           p->vin_mvpp, p->vout_mvpp, p->vout_dc_mv, p->clipped ? "CLIP!" : "");
  }
  tuning_word = 0;

  /* -3 dB points */
  int kmax = 0;
  for (int k = 1; k < N_FREQ; k++) if (pts[k].gain > pts[kmax].gain) kmax = k;
  float g3 = 0.7071f * pts[kmax].gain;
  float fL = -1.0f, fH = -1.0f;
  for (int k = kmax; k > 0; k--)          if (pts[k-1].gain < g3) { fL = cross_freq(k-1, k, g3); break; }
  for (int k = kmax; k < N_FREQ - 1; k++) if (pts[k+1].gain < g3) { fH = cross_freq(k, k+1, g3); break; }

  printf("\r\nPeak gain = %.1f V/V (%.1f dB) at %.2f Hz\r\n",
         pts[kmax].gain, pts[kmax].gain_db, pts[kmax].f);
  if (fL > 0) printf("f_L (-3 dB) = %.2f Hz\r\n", fL); else printf("f_L below sweep range\r\n");
  if (fH > 0) printf("f_H (-3 dB) = %.2f Hz\r\n", fH); else printf("f_H above sweep range\r\n");

  /* Machine-readable block for plot_bode.py */
  printf("BEGIN\r\n");
  for (int k = 0; k < N_FREQ; k++)
    printf("DATA,%.3f,%.3f,%.2f,%.2f,%.3f,%.2f,%.0f,%d\r\n",
           pts[k].f, pts[k].gain, pts[k].gain_db, pts[k].phase_deg,
           pts[k].vin_mvpp, pts[k].vout_mvpp, pts[k].vout_dc_mv, pts[k].clipped);
  printf("END\r\n");
}

/* No signal: check DC operating point (should be ~V_ref ~1.3 V) */
static void dc_check(void)
{
  float save = dac_amp;
  dac_amp = 0.0f;
  BodePoint p = measure(1.0f);
  dac_amp = save;
  tuning_word = 0;
  printf("\r\nNo input: LM358 output DC = %.0f mV (expect ~1300 mV = V_ref)\r\n", p.vout_dc_mv);
  if (p.vout_dc_mv < 300.0f)  printf("  ! stuck low - check bias network / pin 3\r\n");
  if (p.vout_dc_mv > 3000.0f) printf("  ! stuck high - check feedback wiring\r\n");
}

static void print_menu(void)
{
  printf("\r\n=== ELEC2104 TestBench: STAGE II (filter/amp) ===\r\n");
  printf(" d : DC check (output bias, no signal)\r\n");
  printf(" s : Bode sweep %.1f - %.0f Hz\r\n", freqs[0], freqs[N_FREQ - 1]);
  printf(" + / - : double / halve input amplitude (now %.2f mVpp)\r\n> ", vin_mvpp_setting());
}

/* Sample-rate ISR: read Vin monitor + Vout, accumulate, write next DAC value */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM6) return;

  /* 1. Sample both channels (they show the DAC value written last tick) */
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1);
  uint16_t x = (uint16_t)HAL_ADC_GetValue(&hadc1);   /* rank 1: DAC monitor  */
  HAL_ADC_PollForConversion(&hadc1, 1);
  uint16_t y = (uint16_t)HAL_ADC_GetValue(&hadc1);   /* rank 2: LM358 output */

  uint32_t idx = phase_acc >> 24;

  if (lockin_busy) {
    if (n_settle) {
      n_settle--;
    } else if (n_count < n_meas) {
      float s = sin_lut[idx];
      float c = sin_lut[(idx + 64u) & 255u];
      sx  += x;      sy  += y;
      sxs += x * s;  sxc += x * c;
      sys += y * s;  syc += y * c;
      ss  += s;      sc  += c;
      if (y < ymin) ymin = y;
      if (y > ymax) ymax = y;
      if (++n_count >= n_meas) lockin_busy = 0;
    }
  }

  /* 2. Advance phase and output the next sine sample */
  phase_acc += tuning_word;
  idx = phase_acc >> 24;
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
                   (uint32_t)(DAC_MID + dac_amp * sin_lut[idx]));
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_ICACHE_Init();
  MX_LPUART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  for (int i = 0; i < 256; i++) sin_lut[i] = sinf(2.0f * (float)M_PI * i / 256.0f);

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)DAC_MID);
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R,
                   (uint32_t)(DAC2_DC_MV / VDDA_MV * 4095.0f));
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

  /* Sanity check: TIM6 must tick at exactly 2000 Hz */
  {
    uint32_t pclk1  = HAL_RCC_GetPCLK1Freq();
    uint32_t timclk = (READ_BIT(RCC->CFGR, RCC_CFGR_PPRE1) == RCC_HCLK_DIV1) ? pclk1 : 2u * pclk1;
    uint32_t f_upd  = timclk / ((htim6.Init.Prescaler + 1u) * (htim6.Init.Period + 1u));
    printf("\r\nTIM6 clock = %lu Hz, update = %lu Hz (need 2000)\r\n",
           (unsigned long)timclk, (unsigned long)f_upd);
  }

  HAL_TIM_Base_Start_IT(&htim6);
  print_menu();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t c;
    if (HAL_UART_Receive(&hlpuart1, &c, 1, 10) == HAL_OK) {
      switch (c) {
        case 'd': dc_check();  break;
        case 's': run_sweep(); break;
        case '+': if (dac_amp * 2.0f <= 2000.0f) dac_amp *= 2.0f; break;
        case '-': dac_amp *= 0.5f; break;
        default:  break;
      }
      if (c != '\r' && c != '\n') print_menu();
    }
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 55;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 109;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 499;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */