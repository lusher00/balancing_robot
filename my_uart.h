#ifndef _MY_UART_H_
#define _MY_UART_H_

#include "stdint.h"

extern void my_uart_0_init(unsigned long baud, unsigned long config);
extern void my_uart_1_init(unsigned long baud, unsigned long config);
extern void UART0IntHandler(void);
extern void UART1IntHandler(void);
extern void UART0Send(char *pucBuffer, uint8_t len);
extern void UART1Send(char *pucBuffer, uint8_t len);

#endif