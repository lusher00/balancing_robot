#include "rc_radio.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "stdint.h"
#include "utils.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"


void timer2_init()
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
	GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_5);
	GPIOIntTypeSet(GPIO_PORTH_BASE, GPIO_PIN_5, GPIO_BOTH_EDGES);
	GPIOPinIntEnable(GPIO_PORTH_BASE, GPIO_PIN_5);
	IntEnable(INT_GPIOH);

	timer2_init();

}

volatile uint32_t Tick1us = 0;
void Timer2IntHandler()
{
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);


	Tick1us+=1;
}

volatile uint32_t pulse_width = 0;
void PortH_InterruptHandler()
{
	GPIOPinIntClear(GPIO_PORTH_BASE, GPIO_PIN_5);
	if(GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_5))
		Tick1us=0;
	else
		pulse_width = Tick1us;

}
