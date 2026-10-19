/**
 * console.c - Serial menu. One key = one command.
 *
 * Debug commands are ordered from lowest level up, so a fault can be
 * isolated module by module:  i (clocks/ISR) -> a (ADC) -> g (DAC) ->
 * 1 (lock-in, one point) -> d / s (full Stage II tests).
 */
#include "console.h"
#include "app.h"
#include "app_config.h"
#include "siggen.h"
#include "bode.h"
#include <stdio.h>

/* Route printf to the ST-LINK virtual COM port */
int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;
  HAL_UART_Transmit(&hlpuart1, &c, 1, HAL_MAX_DELAY);
  return ch;
}

void console_menu(void)
{
  printf("\r\n=== ELEC2104 TestBench: STAGE II ===\r\n");
  printf(" -- debug, one module at a time --\r\n");
  printf(" i : info (clock, ISR rate, ADC errors)\r\n");
  printf(" a : print 20 raw ADC readings\r\n");
  printf(" g : generator ON at 1 Hz (no measuring)   x : generator OFF\r\n");
  printf(" 1 : lock-in at one frequency (5 Hz)\r\n");
  printf(" -- Stage II tests --\r\n");
  printf(" d : DC check (output bias, no signal)\r\n");
  printf(" s : full Bode sweep 0.2 - 100 Hz\r\n");
  printf(" + / - : double / halve input amplitude (now %.2f mVpp)\r\n> ",
         siggen_vin_mvpp());
}

void console_poll(void)
{
  uint8_t c;
  if (HAL_UART_Receive(&hlpuart1, &c, 1, 10) != HAL_OK) return;

  switch (c) {
    case 'i': app_print_info();                         break;
    case 'a': app_print_raw(20);                        break;
    case 'g': siggen_set_freq(1.0f);
              printf("\r\nGenerator ON, 1 Hz. Press 'a' to watch it.\r\n"); break;
    case 'x': siggen_stop(); printf("\r\nGenerator OFF\r\n"); break;
    case '1': bode_single(5.0f);                        break;
    case 'd': bode_dc_check();                          break;
    case 's': bode_sweep();                             break;
    case '+': siggen_set_amp(siggen_get_amp() * 2.0f);  break;
    case '-': siggen_set_amp(siggen_get_amp() * 0.5f);  break;
    case '\r':
    case '\n': return;
    default:  printf("\r\nUnknown key '%c'\r\n", c);   break;
  }
  console_menu();
}
