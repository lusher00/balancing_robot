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
				 pulse_stop[NUM_CONTROL_SURFACES],
				 pulse_offset[NUM_CONTROL_SURFACES];

volatile uint8_t pulse_offset_flag[NUM_CONTROL_SURFACES];
volatile uint16_t pulse_offset_cnt[NUM_CONTROL_SURFACES];

volatile int32_t pulse_offset_array[NUM_CONTROL_SURFACES][NUM_VALS_AVG_OFFSET];
static uint8_t done_averaging()
{
	uint8_t i=0, ret=1;

	while((i < NUM_CONTROL_SURFACES) && ret)
		ret &= pulse_offset_flag[i++];

	return ret;
}

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
	// Setup up GPIO to read the PWM coming from the radio
	GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_RADIO_PINS);
	GPIOIntTypeSet(GPIO_PORTH_BASE, GPIO_RADIO_PINS, GPIO_BOTH_EDGES);
	GPIOPinIntEnable(GPIO_PORTH_BASE, GPIO_RADIO_PINS);
	IntEnable(INT_GPIOH);

	// We need a timer to measure the width
	timer2_init();

	// Stay right here until we are done averaging values and have a valid offset.
	while(!done_averaging());
}

void Timer2IntHandler()
{
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	Tick1us+=1;
}

int16_t get_rc_pulse_width(RC_PULSE_t pulse, tBoolean raw)
{
	switch(pulse){
		case PULSE_ELEVATOR:
			if(raw)
				return (int16_t)(pulse_width[PULSE_ELEVATOR]);
			else
				return ((int16_t)(pulse_width[PULSE_ELEVATOR] - pulse_offset[PULSE_ELEVATOR])/PULSE_WIDTH_SCALE_FACTOR);
		case PULSE_RUDDER:
			if(raw)
				return (int16_t)(pulse_width[PULSE_RUDDER]);
			else
				return ((int16_t)(pulse_width[PULSE_RUDDER] - pulse_offset[PULSE_RUDDER])/PULSE_WIDTH_SCALE_FACTOR);
		case PULSE_AILERON:
			if(raw)
				return (int16_t)(pulse_width[PULSE_AILERON]);
			else
				return ((int16_t)(pulse_width[PULSE_AILERON] - pulse_offset[PULSE_AILERON])/PULSE_WIDTH_SCALE_FACTOR);
		case PULSE_THROTTLE:
			if(raw)
				return (int16_t)(pulse_width[PULSE_THROTTLE]);
			else
				return ((int16_t)(pulse_width[PULSE_THROTTLE] - pulse_offset[PULSE_THROTTLE])/PULSE_WIDTH_SCALE_FACTOR);
		default:
			return 0;
	}
}

void PortH_InterruptHandler()
{
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

			// Add up NUM_VALS_AVG_OFFSET to average and get an offset from 0
			if(pulse_offset_cnt[PULSE_ELEVATOR] < NUM_VALS_AVG_OFFSET){
				pulse_offset[PULSE_ELEVATOR] += pulse_width[PULSE_ELEVATOR];
				pulse_offset_array[PULSE_ELEVATOR][pulse_offset_cnt[PULSE_ELEVATOR]] = pulse_width[PULSE_ELEVATOR];
				pulse_offset_cnt[PULSE_ELEVATOR]++;
			}else if(!pulse_offset_flag[PULSE_ELEVATOR]){
				pulse_offset_flag[PULSE_ELEVATOR] = 1;
				pulse_offset[PULSE_ELEVATOR] /= NUM_VALS_AVG_OFFSET;
			}
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

			// Add up NUM_VALS_AVG_OFFSET to average and get an offset from 0
			if(pulse_offset_cnt[PULSE_AILERON] < NUM_VALS_AVG_OFFSET){
				pulse_offset_cnt[PULSE_AILERON]++;
				pulse_offset[PULSE_AILERON] += pulse_width[PULSE_AILERON];
			}else if(!pulse_offset_flag[PULSE_AILERON]){
				pulse_offset_flag[PULSE_AILERON] = 1;
				pulse_offset[PULSE_AILERON] /= NUM_VALS_AVG_OFFSET;
			}
		}
	}

	if(status & GPIO_PIN_THROTTLE)
	{
		// Was this a rising or falling edge?
		if(GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_THROTTLE)){
			// It was a rising edge
			pulse_start[PULSE_THROTTLE] = Tick1us;
		}else{
			// It was a falling edge
			pulse_stop[PULSE_THROTTLE] = Tick1us;

			pulse_width[PULSE_THROTTLE] = pulse_stop[PULSE_THROTTLE] - pulse_start[PULSE_THROTTLE];
			// Did we overflow?
			if(pulse_width[PULSE_THROTTLE] < 0)
				pulse_width[PULSE_THROTTLE] += 0xFFFF;

			// Add up NUM_VALS_AVG_OFFSET to average and get an offset from 0
			if(pulse_offset_cnt[PULSE_THROTTLE] < NUM_VALS_AVG_OFFSET){
				pulse_offset_cnt[PULSE_THROTTLE]++;
				pulse_offset[PULSE_THROTTLE] += pulse_width[PULSE_THROTTLE];
			}else if(!pulse_offset_flag[PULSE_THROTTLE]){
				pulse_offset_flag[PULSE_THROTTLE] = 1;
				pulse_offset[PULSE_THROTTLE] /= NUM_VALS_AVG_OFFSET;
			}
		}
	}

	if(status & GPIO_PIN_RUDDER)
	{
		// Was this a rising or falling edge?
		if(GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_RUDDER)){
			// It was a rising edge
			pulse_start[PULSE_RUDDER] = Tick1us;
		}else{
			// It was a falling edge
			pulse_stop[PULSE_RUDDER] = Tick1us;

			pulse_width[PULSE_RUDDER] = pulse_stop[PULSE_RUDDER] - pulse_start[PULSE_RUDDER];
			// Did we overflow?
			if(pulse_width[PULSE_RUDDER] < 0)
				pulse_width[PULSE_RUDDER] += 0xFFFF;

			// Add up NUM_VALS_AVG_OFFSET to average and get an offset from 0
			if(pulse_offset_cnt[PULSE_RUDDER] < NUM_VALS_AVG_OFFSET){
				pulse_offset_cnt[PULSE_RUDDER]++;
				pulse_offset[PULSE_RUDDER] += pulse_width[PULSE_RUDDER];
			}else if(!pulse_offset_flag[PULSE_RUDDER]){
				pulse_offset_flag[PULSE_RUDDER] = 1;
				pulse_offset[PULSE_RUDDER] /= NUM_VALS_AVG_OFFSET;
			}
		}
	}
}
