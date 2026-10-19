/**
 * bode.c - Stage II frequency sweep and DC operating-point check
 */
#include "bode.h"
#include "lockin.h"
#include "siggen.h"
#include "app_config.h"
#include <stdio.h>
#include <math.h>

#define N_FREQ 17

static const float freqs[N_FREQ] = {
  0.2f, 0.3f, 0.5f, 0.7f, 1.0f, 1.5f, 2.0f, 3.0f, 5.0f,
  7.0f, 10.0f, 15.0f, 20.0f, 30.0f, 50.0f, 70.0f, 100.0f
};

static BodePoint pts[N_FREQ];

static void print_header(void)
{
  printf("     f(Hz)    gain    dB     phase   Vin(mVpp) Vout(mVpp) Vout_DC(mV)\r\n");
}

static void print_point(const BodePoint *p)
{
  printf("%10.2f %7.1f %6.1f %8.1f %10.2f %10.1f %10.0f %s\r\n",
         p->f, p->gain, p->gain_db, p->phase_deg,
         p->vin_mvpp, p->vout_mvpp, p->vout_dc_mv, p->clipped ? "CLIP!" : "");
}

/* Log-interpolated frequency where gain crosses 'level' between i and j */
static float cross_freq(int i, int j, float level)
{
  float t = (level - pts[i].gain) / (pts[j].gain - pts[i].gain);
  return powf(10.0f, log10f(pts[i].f) + t * (log10f(pts[j].f) - log10f(pts[i].f)));
}

void bode_single(float f_hz)
{
  printf("\r\nSingle point at %.2f Hz, Vin ~ %.2f mVpp\r\n", f_hz, siggen_vin_mvpp());
  print_header();
  BodePoint p = lockin_measure(f_hz);
  siggen_stop();
  print_point(&p);
}

void bode_sweep(void)
{
  printf("\r\nSweep start: Vin ~ %.2f mVpp, %d points (low freqs are slow)\r\n",
         siggen_vin_mvpp(), N_FREQ);
  print_header();

  for (int k = 0; k < N_FREQ; k++) {
    pts[k] = lockin_measure(freqs[k]);
    print_point(&pts[k]);
  }
  siggen_stop();

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

void bode_dc_check(void)
{
  float save = siggen_get_amp();
  siggen_set_amp(0.0f);
  BodePoint p = lockin_measure(1.0f);
  siggen_set_amp(save);
  siggen_stop();

  printf("\r\nNo input: LM358 output DC = %.0f mV (expect ~1300 mV = V_ref)\r\n", p.vout_dc_mv);
  if (p.vout_dc_mv < 300.0f)  printf("  ! stuck low - check bias network / pin 3\r\n");
  if (p.vout_dc_mv > 3000.0f) printf("  ! stuck high - check feedback wiring\r\n");
}
