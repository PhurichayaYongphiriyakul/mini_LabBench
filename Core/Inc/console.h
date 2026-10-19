/**
 * console.h - printf over ST-LINK VCP (LPUART1) and single-key commands
 */
#ifndef CONSOLE_H
#define CONSOLE_H

void console_menu(void);
void console_poll(void);   /* non-blocking: returns after ~10 ms if no key */

#endif /* CONSOLE_H */
