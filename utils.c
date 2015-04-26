#include "inc/hw_types.h"
#include "utils.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "stdint.h"

void led_init()
{
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
}

void led_toggle()
{
	static uint8_t toggle = 0;

	toggle = toggle ^ 1;
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, (toggle<<3));
}
