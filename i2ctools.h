#ifndef _I2C_TOOLS_
#define _I2C_TOOLS_

#define I2C_SEND        0
#define I2C_RECEIVE     1

extern void i2c_init(void);
extern void i2c_tx_single(unsigned char SlaveAddr, unsigned char dest, unsigned char data);
extern unsigned long i2c_rx_single(unsigned char SlaveAddr, unsigned char dest);
extern void i2c_rx_multi(unsigned char SlaveAddr, unsigned char dest, unsigned char num_bytes, unsigned long *data);

#endif
