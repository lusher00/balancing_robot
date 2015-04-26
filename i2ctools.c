#include "inc/hw_types.h"
#include "i2ctools.h"
#include "driverlib/i2c.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"

// 80MHz clock so 80,000 is about 1ms
void cheapDelay(unsigned long delay)
{
    while(delay--);
}

void i2c_init()
{
    // Trying to get around this damn I2C lockup issue
    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_0);
    GPIOPinTypeGPIOInput(GPIO_PORTG_BASE, GPIO_PIN_1);
    while(!GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_1))
    {
        // Toggle the clock at 100kHz until the slave releases SDA
        GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_PIN_0);
        cheapDelay(400);
        GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, ~GPIO_PIN_0);
        cheapDelay(400);
    }
 
    // Initialize the I2C channel the sensor is connected to
    SysCtlPeripheralEnable( SYSCTL_PERIPH_I2C1 );
    GPIOPinConfigure( GPIO_PG0_I2C1SCL );
    GPIOPinConfigure( GPIO_PG1_I2C1SDA );
    GPIOPinTypeI2C( GPIO_PORTG_BASE, GPIO_PIN_0 | GPIO_PIN_1 );
    
    // Set the clock (false = "slow" = 100kbps)
    I2CMasterInitExpClk( I2C1_MASTER_BASE, SysCtlClockGet(), false );
    I2CMasterEnable( I2C1_MASTER_BASE );
    
}

void i2c_tx_single(unsigned char SlaveAddr, unsigned char dest, unsigned char data)
{
    I2CMasterSlaveAddrSet( I2C1_MASTER_BASE, SlaveAddr, I2C_SEND );
    
    I2CMasterDataPut( I2C1_MASTER_BASE, dest );
    I2CMasterControl( I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START );
    while(I2CMasterBusy(I2C1_MASTER_BASE));
    
    I2CMasterDataPut( I2C1_MASTER_BASE, data );
    I2CMasterControl( I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH );
    while(I2CMasterBusy(I2C1_MASTER_BASE));
}

unsigned long i2c_rx_single(unsigned char SlaveAddr, unsigned char dest)
{
    I2CMasterSlaveAddrSet( I2C1_MASTER_BASE, SlaveAddr, I2C_SEND );
    I2CMasterDataPut( I2C1_MASTER_BASE, dest );
    I2CMasterControl( I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND );
    while (I2CMasterBusy( I2C1_MASTER_BASE ));
    
    I2CMasterSlaveAddrSet( I2C1_MASTER_BASE, SlaveAddr, I2C_RECEIVE );
    I2CMasterControl( I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE );
    while(I2CMasterBusy( I2C1_MASTER_BASE ));
    
    return I2CMasterDataGet(I2C1_MASTER_BASE);
}
void i2c_rx_multi(unsigned char SlaveAddr, unsigned char dest, unsigned char num_bytes, unsigned long *data)
{
    unsigned int i=0;
    
    // Set the address
    I2CMasterSlaveAddrSet( I2C1_MASTER_BASE, SlaveAddr, I2C_SEND );
    I2CMasterDataPut( I2C1_MASTER_BASE, dest );
    I2CMasterControl( I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND );
    while (I2CMasterBusy( I2C1_MASTER_BASE ));
    
    // Set the address again to tell the device to start sending data
    I2CMasterSlaveAddrSet( I2C1_MASTER_BASE, SlaveAddr, I2C_RECEIVE );
    
    I2CMasterControl( I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START );
    while(I2CMasterBusy( I2C1_MASTER_BASE ));
    *data++ = I2CMasterDataGet(I2C1_MASTER_BASE);
        
    while(i++ < (num_bytes-2))
    {
        I2CMasterControl( I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT );
        while(I2CMasterBusy( I2C1_MASTER_BASE ));
        *data++ = I2CMasterDataGet(I2C1_MASTER_BASE);
    }
    
    I2CMasterControl( I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH );
    while(I2CMasterBusy( I2C1_MASTER_BASE ));
    *data++ = I2CMasterDataGet(I2C1_MASTER_BASE);
}
