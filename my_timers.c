#include "my_timers.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_timer.h"
#include "inc/hw_ints.h"
#include "stdint.h"

volatile unsigned int Tick100us1 = 0;
volatile unsigned int Tick100us2 = 0;

void my_timer0_init()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);
	// Interrupt every 100uS
	TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/10000);
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	TimerControlStall(TIMER0_BASE, TIMER_A, true);
	TimerEnable(TIMER0_BASE, TIMER_A);

}

void my_timer1_init()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	TimerConfigure(TIMER1_BASE, TIMER_CFG_32_BIT_PER);
	// Interrupt every 1ms
	TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet()/1000);
	IntEnable(INT_TIMER1A);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	TimerControlStall(TIMER1_BASE, TIMER_A, true);
	TimerEnable(TIMER1_BASE, TIMER_A);

}

unsigned int myTimerValueGet()
{
	return Tick100us1;
}

void myTimerZero()
{
	Tick100us1 = 0;
}

void delay100us(uint64_t uS_100)
{
	Tick100us2 = 0;
	while(Tick100us2< uS_100);
}

void Timer0IntHandler()
{
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	Tick100us1+=1;
}

void Timer1IntHandler()
{
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	Tick100us2++;
}
