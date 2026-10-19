/**
 * lockin.h - Software lock-in amplifier: gain and phase at one frequency
 * (replaces reading Vpp and phase off the R&S RTB2002 oscilloscope)
 */
#ifndef LOCKIN_H
#define LOCKIN_H

#include <stdint.h>

typedef struct {
  float f;            /* Hz                                   */
  float gain;         /* V/V, Vout / Vin at node X            */
  float gain_db;
  float phase_deg;    /* Vout relative to Vin                 */
  float vin_mvpp;     /* at node X (after attenuator)         */
  float vout_mvpp;    /* at LM358 pin 1 (before divider)      */
  float vout_dc_mv;   /* mean of LM358 pin 1                  */
  int   clipped;      /* 1 = output hit a rail                */
} BodePoint;

void      lockin_on_sample(uint16_t x, uint16_t y, uint32_t idx);  /* ISR only */
BodePoint lockin_measure(float f_hz);                               /* blocking */
int       lockin_is_busy(void);

#endif /* LOCKIN_H */
