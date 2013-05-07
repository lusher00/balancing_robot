#ifndef _ADXL345_H_
#define _ADXL345_H_

#include "stdint.h"

#define ACCEL_ADDRESS   0x53

#define I2C_SEND        0
#define I2C_RECEIVE     1

extern void accel_init(void);
extern void accel_get_xyz_raw(int16_t* accel_x_raw, int16_t* accel_y_raw, int16_t* accel_z_raw);
extern void accel_get_xyz_avg(int16_t* accel_x_avg, int16_t* accel_y_avg, int16_t* accel_z_avg);
extern void accel_get_xyz_cal(int16_t* accel_x_cal, int16_t* accel_y_cal, int16_t* accel_z_cal, uint8_t bAvg);

#endif