#ifndef _L3G4200D_H_
#define _L3G4200D_H_

//#include "stdint.h"

#define GYRO_ADDRESS           0x69

#define L3G4200D_WHO_AM_I      0x0F
#define L3G4200D_CTRL_REG1     0x20
#define L3G4200D_CTRL_REG2     0x21
#define L3G4200D_CTRL_REG3     0x22
#define L3G4200D_CTRL_REG4     0x23
#define L3G4200D_CTRL_REG5     0x24
#define L3G4200D_REFERENCE     0x25
#define L3G4200D_OUT_TEMP      0x26
#define L3G4200D_STATUS_REG    0x27
#define L3G4200D_OUT_X_L       0x28
#define L3G4200D_OUT_X_H       0x29
#define L3G4200D_OUT_Y_L       0x2A
#define L3G4200D_OUT_Y_H       0x2B
#define L3G4200D_OUT_Z_L       0x2C
#define L3G4200D_OUT_Z_H       0x2D
#define L3G4200D_FIFO_CTRL_REG 0x2E
#define L3G4200D_FIFO_SRC_REG  0x2F
#define L3G4200D_INT1_CFG      0x30
#define L3G4200D_INT1_SRC      0x31
#define L3G4200D_INT1_THS_XH   0x32
#define L3G4200D_INT1_THS_XL   0x33
#define L3G4200D_INT1_THS_YH   0x34
#define L3G4200D_INT1_THS_YL   0x35
#define L3G4200D_INT1_THS_ZH   0x36
#define L3G4200D_INT1_THS_ZL   0x37
#define L3G4200D_INT1_DURATION 0x38

#define I2C_SEND        0
#define I2C_RECEIVE     1

#define L3G4200D_DPS_250       0x00
#define L3G4200D_DPS_500       0x10
#define L3G4200D_DPS_1000      0x20
#define L3G4200D_DPS_2000      0x30

extern double g_gyroScale;

//extern int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
//extern uint8_t gyro_idx_new, gyro_idx_old;
//extern int16_t gyro_x_vals[10], gyro_y_vals[10], gyro_z_vals[10];
//extern int16_t gyro_x_avg, gyro_y_avg, gyro_z_avg;
//extern int16_t gyro_x_tot, gyro_y_tot, gyro_z_tot;
//extern float gyro_x_cal, gyro_y_cal, gyro_z_cal;

extern void gyro_get_xyz_raw(int16_t* gyro_x_raw, int16_t* gyro_y_raw, int16_t* gyro_z_raw);
extern void gyro_get_xyz_avg(int16_t* gyro_x_avg, int16_t* gyro_y_avg, int16_t* gyro_z_avg);
extern void gyro_get_xyz_cal(int16_t* gyro_x_cal, int16_t* gyro_y_cal, int16_t* gyro_z_cal, uint8_t bAvg);

extern void gyro_init(void);

#endif