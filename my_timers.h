#ifndef _MY_TIMERS_H_
#define _MY_TIEMRS_H_

#include "stdint.h"

extern void my_timer0_init(void);
extern void my_timer1_init(void);
extern uint64_t myTimerValueGet(void);
extern void myTimerZero(void);

#endif
