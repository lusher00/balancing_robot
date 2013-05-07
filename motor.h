#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stdint.h"

#define PWM_PERIOD 500

extern void qei_init(void);
extern int32_t pos_get(void);
extern void motor_init(void);
extern void drive_motors(int16_t left, int16_t right);
extern void stop_motors(void);

#endif
