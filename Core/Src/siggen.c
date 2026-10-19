/**
 * siggen.c - Direct digital synthesis (DDS) sine on DAC1_OUT1
 *
 * A 32-bit phase accumulator advances by 'tuning_word' every TIM6 tick.
 * The top 8 bits pick one of 256 entries in a sine look-up table.
 *   f_out = tuning_word * FS_HZ / 2^32
 */
#include "siggen.h"
#include "app_config.h"
#include <math.h>

#define LUT_SIZE 256u

static float             lut[LUT_SIZE];
static volatile uint32_t phase_acc;
static volatile uint32_t tuning_word;
static volatile float    amp_cnt = DAC_AMP_DEFAULT;
static volatile float    freq_hz;

void siggen_init(void)
{
  for (uint32_t i = 0; i < LUT_SIZE; i++)
    lut[i] = sinf(2.0f * (float)M_PI * (float)i / (float)LUT_SIZE);

  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)DAC_MID);
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R,
                   (uint32_t)(DAC2_DC_MV / VDDA_MV * 4095.0f));
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
}

void siggen_set_freq(float f_hz)
{
  freq_hz     = (f_hz > 0.0f) ? f_hz : 0.0f;
  tuning_word = (uint32_t)((double)freq_hz / FS_HZ * 4294967296.0);
}

void siggen_stop(void)
{
  __disable_irq();
  tuning_word = 0;
  freq_hz     = 0.0f;
  phase_acc   = 0;            /* sin(0) = 0 -> output sits at DAC_MID */
  __enable_irq();
}

void siggen_tick(void)
{
  phase_acc += tuning_word;
  float v = DAC_MID + amp_cnt * lut[phase_acc >> 24];
  if (v < 0.0f)    v = 0.0f;
  if (v > 4095.0f) v = 4095.0f;
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)v);
}

uint32_t siggen_index(void)            { return phase_acc >> 24; }
float    siggen_sin(uint32_t idx)      { return lut[idx & (LUT_SIZE - 1u)]; }
float    siggen_cos(uint32_t idx)      { return lut[(idx + LUT_SIZE / 4u) & (LUT_SIZE - 1u)]; }

void siggen_set_amp(float counts)
{
  if (counts < 0.0f)        counts = 0.0f;
  if (counts > DAC_AMP_MAX) counts = DAC_AMP_MAX;
  amp_cnt = counts;
}

float siggen_get_amp(void)   { return amp_cnt; }
float siggen_get_freq(void)  { return freq_hz; }

float siggen_vin_mvpp(void)
{
  return 2.0f * amp_cnt * (VDDA_MV / 4095.0f) * K_ATT;
}
