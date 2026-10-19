/**
 * bode.h - Stage II tests built on siggen + lockin
 */
#ifndef BODE_H
#define BODE_H

void bode_sweep(void);        /* full sweep, -3 dB points, CSV block       */
void bode_single(float f_hz); /* one frequency, for debugging              */
void bode_dc_check(void);     /* output DC with no signal (~V_ref)         */

#endif /* BODE_H */
