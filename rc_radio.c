#include "rc_radio.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "stdint.h"
#include "utils.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"

volatile uint16_t Tick1us = 0;
volatile int32_t pulse_width[NUM_CONTROL_SURFACES],
				 pulse_start[NUM_CONTROL_SURFACES],
				 pulse_stop[NUM_CONTROL_SURFACES];

static void timer2_init()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	TimerConfigure(TIMER2_BASE, TIMER_CFG_32_BIT_PER);
	// Interrupt every 1us
	TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet()/1000000);
	IntEnable(INT_TIMER2A);
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	TimerControlStall(TIMER2_BASE, TIMER_A, true);
	TimerEnable(TIMER2_BASE, TIMER_A);
}

void rc_radio_init()
{
	GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_5 | GPIO_PIN_2);
	GPIOIntTypeSet(GPIO_PORTH_BASE, GPIO_PIN_5 | GPIO_PIN_2, GPIO_BOTH_EDGES);
	GPIOPinIntEnable(GPIO_PORTH_BASE, GPIO_PIN_5 | GPIO_PIN_2);
	IntEnable(INT_GPIOH);
	timer2_init();
}

void Timer2IntHandler()
{
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	Tick1us+=1;
}

uint16_t get_rc_pulse_width(RC_PULSE_t pulse)
{
	switch(pulse){
		case PULSE_ELEVATOR:
			return ((uint16_t)pulse_width[PULSE_ELEVATOR]);
		case PULSE_RUDDER:
			return ((uint16_t)pulse_width[PULSE_RUDDER]);
		case PULSE_AILERON:
			return ((uint16_t)pulse_width[PULSE_AILERON]);
		case PULSE_THROTTLE:
			return ((uint16_t)pulse_width[PULSE_THROTTLE]);
		default:
			return 0;
	}
}

///////////////////////////////////////////////////////////////////
// PORT H INTERRUPT HANDLER
// Needs to be inserted in the vector table
///////////////////////////////////////////////////////////////////
void PortH_InterruptHandler()
{
	static volatile uint16_t pw1start, pw1stop, pw2start, pw2stop;

	// Save the interrupt status for later
	uint8_t status = GPIOPinIntStatus(GPIO_PORTH_BASE, false);

	// Clear the interrupt sources in the NVIC
	GPIOPinIntClear(GPIO_PORTH_BASE, status);

	if(status & GPIO_PIN_ELEVATOR)
	{
		// Was this a rising or falling edge?
		if(GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_ELEVATOR)){
			// It was a rising edge
			pulse_start[PULSE_ELEVATOR] = Tick1us;
		}else{
			// It was a falling edge
			pulse_stop[PULSE_ELEVATOR] = Tick1us;

			pulse_width[PULSE_ELEVATOR] = pulse_stop[PULSE_ELEVATOR] - pulse_start[PULSE_ELEVATOR];
			// Did we overflow?
			if(pulse_width[PULSE_ELEVATOR] < 0)
				pulse_width[PULSE_ELEVATOR] += 0xFFFF;
		}
	}

	if(status & GPIO_PIN_AILERON)
	{
		// Was this a rising or falling edge?
		if(GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_AILERON)){
			// It was a rising edge
			pulse_start[PULSE_AILERON] = Tick1us;
		}else{
			// It was a falling edge
			pulse_stop[PULSE_AILERON] = Tick1us;

			pulse_width[PULSE_AILERON] = pulse_stop[PULSE_AILERON] - pulse_start[PULSE_AILERON];
			// Did we overflow?
			if(pulse_width[PULSE_AILERON] < 0)
				pulse_width[PULSE_AILERON] += 0xFFFF;
		}
	}
}
