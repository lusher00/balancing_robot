#ifndef _RC_RADIO_H_
#define _RC_RADIO_H_

#include "stdint.h"

#define NUM_CONTROL_SURFACES 4

#define GPIO_PIN_ELEVATOR 	GPIO_PIN_5
//#define GPIO_PIN_RUDDER 	GPIO_PIN_2
#define GPIO_PIN_AILERON 	GPIO_PIN_2
//#define GPIO_PIN_THROTTLE 	GPIO_PIN_5

typedef enum {
	PULSE_ELEVATOR=0,
	PULSE_RUDDER=1,
	PULSE_AILERON=2,
	PULSE_THROTTLE=3
}RC_PULSE_t;

extern void rc_radio_init(void);
extern uint16_t get_rc_pulse_width(RC_PULSE_t pulse);
extern void PortJ_InterruptHandler(void);


#endif
