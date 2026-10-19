/**
 * siggen.h - Sine generator on DAC1_OUT1 + DC bias on DAC1_OUT2
 * (replaces the Rigol DG1062Z function generator)
 */
#ifndef SIGGEN_H
#define SIGGEN_H

#include <stdint.h>

void     siggen_init(void);              /* build LUT, start both DAC channels   */
void     siggen_set_freq(float f_hz);    /* 0 = hold current value               */
void     siggen_stop(void);              /* freq 0, output back to centre        */
void     siggen_tick(void);              /* ISR only: advance phase, write DAC   */

uint32_t siggen_index(void);             /* LUT index of the value now on PA4    */
float    siggen_sin(uint32_t idx);
float    siggen_cos(uint32_t idx);

void     siggen_set_amp(float counts);   /* peak amplitude in DAC counts         */
float    siggen_get_amp(void);
float    siggen_vin_mvpp(void);          /* expected Vpp at node X, in mV        */
float    siggen_get_freq(void);

#endif /* SIGGEN_H */
