/**
 * app.h - Entry points called from main.c, plus ADC sampling and debug info
 */
#ifndef APP_H
#define APP_H

#include <stdint.h>

void     app_init(void);            /* call once in USER CODE BEGIN 2     */
void     app_poll(void);            /* call in while(1), USER CODE BEGIN 3 */

void     app_print_info(void);      /* clocks, settings, ISR/ADC counters  */
void     app_print_raw(int n);      /* n raw x/y readings, 50 ms apart     */

#endif /* APP_H */
