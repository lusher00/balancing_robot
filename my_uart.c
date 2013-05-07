#include "my_uart.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

void my_uart_0_init(unsigned long baud, unsigned long config)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    
    // Set GPIO PA0 and PA1 as UART0 pins.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
        
    // Configure UART0 for 115,200, 8-N-1 operation.
    // UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), baud, config);
    
    // Enable interrupts
    //IntEnable(INT_UART0);
    //UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}

void my_uart_1_init(unsigned long baud, unsigned long config)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    
    // Set GPIO PC6 and PC7 as UART1 pins.
    GPIOPinConfigure(GPIO_PC6_U1RX);
    GPIOPinConfigure(GPIO_PC7_U1TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);
        
    // Configure UART1 for 115,200, 8-N-1 operation.
    // UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), baud, config);
    
    // Enable interrupts
    //IntEnable(INT_UART1);
    //UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
}

// The UART interrupt handler.
void UART0IntHandler(void)
{
    unsigned long ulStatus;
    //char c;

    ulStatus = UARTIntStatus(UART0_BASE, true);     // Get the interrrupt status.
    UARTIntClear(UART0_BASE, ulStatus);             // Clear the asserted interrupts.
    
    /*
    // Loop while there are characters in the receive FIFO.
    while(UARTCharsAvail(UART0_BASE))
       	UARTCharPutNonBlocking(UART0_BASE, UARTCharGetNonBlocking(UART0_BASE));
    */
    
    //c = UARTCharGetNonBlocking(UART0_BASE);

}

// The UART interrupt handler.
void UART1IntHandler(void)
{
    unsigned long ulStatus;
 
    ulStatus = UARTIntStatus(UART1_BASE, true);     // Get the interrrupt status.
    UARTIntClear(UART1_BASE, ulStatus);             // Clear the asserted interrupts.
    // Loop while there are characters in the receive FIFO.
    while(UARTCharsAvail(UART1_BASE))
    	UARTCharPutNonBlocking(UART0_BASE, UARTCharGetNonBlocking(UART1_BASE));
 }

// Send a string to UART0.
void UART0Send(char *pucBuffer, uint8_t len)
{
    while(len--)
        UARTCharPut(UART0_BASE, *pucBuffer++);
}

// Send a string to UART1.
void UART1Send(char *pucBuffer, uint8_t len)
{
    while(len--)
        UARTCharPut(UART1_BASE, *pucBuffer++);
}
