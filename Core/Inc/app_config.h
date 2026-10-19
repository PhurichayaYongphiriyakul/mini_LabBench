/**
 * app_config.h - ELEC2104 TestBench: shared constants and HAL handles
 *
 * Wiring (Stage II test):
 *   PA4 DAC1_OUT1 : sine  --[R_SER]--+-- node X --> C1(+) of Stage II
 *                                    |
 *                                 [R_SH]
 *                                    |
 *   PA5 DAC1_OUT2 : DC 2.5 V --------+
 *   PC0 (A5) ADC1_IN1 <- jumper from PA4  (ADC rank 1, "x")
 *   PA3 (A0) ADC1_IN8 <- LM358 pin 1 via 10k/10k divider (ADC rank 2, "y")
 *   Nucleo 5V -> LM358 pin 8, all GNDs common
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "main.h"

/* Handles are defined by CubeMX in main.c */
extern ADC_HandleTypeDef  hadc1;
extern DAC_HandleTypeDef  hdac1;
extern UART_HandleTypeDef hlpuart1;
extern TIM_HandleTypeDef  htim6;

/* ---- Sampling ---------------------------------------------------------- */
#define FS_HZ            2000.0f   /* TIM6 update rate = sample rate        */
#define VDDA_MV          3300.0f   /* nominal analog supply                 */

/* ---- Signal generator (DAC) -------------------------------------------- */
#define DAC_MID          2048.0f   /* DAC1_OUT1 centre ~1.65 V              */
#define DAC_AMP_DEFAULT  620.0f    /* +/-620 counts = 1 Vpp at PA4          */
#define DAC_AMP_MAX      2000.0f
#define DAC2_DC_MV       2500.0f   /* DC bias at C1 input (like collector)  */

/* ---- Attenuator: put MEASURED resistor values here --------------------- */
#define R_SER            22000.0f
#define R_SH             100.0f
#define K_ATT            (R_SH / (R_SER + R_SH))

/* ---- Output divider and LM358 limits ----------------------------------- */
#define OUT_DIV          2.0f      /* (10k + 10k) / 10k                     */
#define OUT_CLIP_MV      3300.0f   /* LM358 @5V can't swing much above 3.5V */
#define OUT_FLOOR_MV     80.0f     /* ...or below ~50 mV                    */

/* ---- Lock-in timing ----------------------------------------------------- */
#define SETTLE_MIN_S     2.0f      /* settle >= 2 s and >= 2 cycles          */
#define MEAS_MIN_S       1.0f      /* measure >= 1 s and >= 4 cycles         */

#endif /* APP_CONFIG_H */
