/**
 * lockin.c - Software lock-in amplifier
 *
 * Each sample of x (source) and y (output) is multiplied by sin and cos of
 * the generator phase and summed. After N samples:
 *   amplitude = 2 * sqrt(I^2 + Q^2) / N,   phase = atan2(Q, I)
 * Noise and harmonics at other frequencies average out.
 */
#include "lockin.h"
#include "siggen.h"
#include "app_config.h"
#include <math.h>

static volatile uint32_t n_settle, n_meas, n_count;
static volatile uint8_t  busy;
static volatile uint16_t ymin, ymax;
static double sx, sy, sxs, sxc, sys, syc, ss, sc;

static inline float cnt_to_mv(double cnt) { return (float)(cnt * VDDA_MV / 4095.0); }

void lockin_on_sample(uint16_t x, uint16_t y, uint32_t idx)
{
  if (!busy) return;

  if (n_settle) {                  /* wait for the filter caps to settle */
    n_settle--;
    return;
  }

  float s = siggen_sin(idx);
  float c = siggen_cos(idx);
  sx  += x;      sy  += y;
  sxs += x * s;  sxc += x * c;
  sys += y * s;  syc += y * c;
  ss  += s;      sc  += c;
  if (y < ymin) ymin = y;
  if (y > ymax) ymax = y;

  if (++n_count >= n_meas) busy = 0;
}

int lockin_is_busy(void) { return busy; }

BodePoint lockin_measure(float f_hz)
{
  float settle_s = fmaxf(2.0f / f_hz, SETTLE_MIN_S);
  float cycles   = ceilf(fmaxf(4.0f, f_hz * MEAS_MIN_S));

  __disable_irq();
  sx = sy = sxs = sxc = sys = syc = ss = sc = 0.0;
  ymin = 4095; ymax = 0;
  n_settle = (uint32_t)(settle_s * FS_HZ);
  n_meas   = (uint32_t)lroundf(cycles * FS_HZ / f_hz);
  n_count  = 0;
  siggen_set_freq(f_hz);
  busy = 1;
  __enable_irq();

  while (busy) { }                 /* ISR does the work */

  double N  = (double)n_meas;
  double mx = sx / N, my = sy / N;
  double Xi = sxs - mx * ss, Xq = sxc - mx * sc;   /* remove DC leakage */
  double Yi = sys - my * ss, Yq = syc - my * sc;
  double ax = 2.0 * sqrt(Xi * Xi + Xq * Xq) / N;   /* peak, ADC counts  */
  double ay = 2.0 * sqrt(Yi * Yi + Yq * Yq) / N;

  BodePoint p = {0};
  p.f          = f_hz;
  p.gain       = (ax > 0.0) ? (float)((ay * OUT_DIV) / (ax * K_ATT)) : 0.0f;
  p.gain_db    = (p.gain > 0.0f) ? 20.0f * log10f(p.gain) : -999.0f;
  p.vin_mvpp   = 2.0f * cnt_to_mv(ax) * K_ATT;
  p.vout_mvpp  = 2.0f * cnt_to_mv(ay) * OUT_DIV;
  p.vout_dc_mv = cnt_to_mv(my) * OUT_DIV;

  float ph = (float)((atan2(Yq, Yi) - atan2(Xq, Xi)) * 180.0 / M_PI);
  while (ph > 180.0f)   ph -= 360.0f;
  while (ph <= -180.0f) ph += 360.0f;
  p.phase_deg = ph;

  float out_max = cnt_to_mv(ymax) * OUT_DIV;
  float out_min = cnt_to_mv(ymin) * OUT_DIV;
  p.clipped = (out_max > OUT_CLIP_MV) || (out_min < OUT_FLOOR_MV);
  return p;
}
