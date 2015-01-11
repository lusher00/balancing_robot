#ifndef _MY_TIMERS_H_
#define _MY_TIEMRS_H_

#include "stdint.h"

extern void my_timer0_init(void);
extern void my_timer1_init(void);
extern unsigned int myTimerValueGet(void);
extern void myTimerZero(void);
extern void delay100us(uint64_t uS_100);

#endif
