#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/i2c.h"
#include "adxl345.h"
#include "i2ctools.h"

#define VALS_TO_AVG 10

int16_t accel_x_offset, accel_y_offset, accel_z_offset;

void accel_init()
{
    uint32_t i;
    int16_t accel_x=0, accel_y=0, accel_z=0;
    int32_t accel_x_cal=0, accel_y_cal=0, accel_z_cal=0;
  
    i2c_tx_single(ACCEL_ADDRESS, 0x2D, 0x08);
    i2c_tx_single(ACCEL_ADDRESS, 0x31, 0x08);
    // Disable lowpower mode, set the ODR at 800Hz.
    // 0b1000 = 25Hz and each LSB represents a doubling in bandwidth
    i2c_tx_single(ACCEL_ADDRESS, 0x2C, 0x1D);
        
    //  Calibrate the accelerometer
    for(i=0; i<1000; i++)
    {
        accel_get_xyz_raw(&accel_x, &accel_y, &accel_z);
        accel_x_cal += accel_x;
        accel_y_cal += accel_y;
        accel_z_cal += accel_z;
    }
    
    accel_x_offset = accel_x_cal / 1000;
    accel_y_offset = accel_y_cal / 1000;
    accel_z_offset = accel_z_cal / 1000 - 240; // @2g range
}

void accel_get_xyz_raw(int16_t* accel_x_raw, int16_t* accel_y_raw, int16_t* accel_z_raw)
{
    unsigned long data[6];
    
    i2c_rx_multi(ACCEL_ADDRESS, 0x32, 6, data);
    
    *accel_x_raw = ((int16_t)data[0]);
    *accel_x_raw |= ((int16_t)data[1])<<8;

    *accel_y_raw = ((int16_t)data[2]);
    *accel_y_raw |= ((int16_t)data[3])<<8;

    *accel_z_raw = ((int16_t)data[4]);
    *accel_z_raw |= ((int16_t)data[5])<<8;
}

uint8_t accel_idx_new=0, accel_idx_old=1;
int16_t accel_x_vals[VALS_TO_AVG], accel_y_vals[VALS_TO_AVG], accel_z_vals[VALS_TO_AVG];
int16_t accel_x_tot=0, accel_y_tot=0, accel_z_tot=0;
void accel_get_xyz_avg(int16_t* accel_x_avg, int16_t* accel_y_avg, int16_t* accel_z_avg)
{
    int16_t accel_x, accel_y, accel_z;
    
    
    accel_get_xyz_raw(&accel_x, &accel_y, &accel_z);
    
    accel_x_vals[accel_idx_new] = accel_x;
    accel_x_tot = accel_x_tot + accel_x_vals[accel_idx_new];
    accel_x_tot = accel_x_tot - accel_x_vals[accel_idx_old];    
    *accel_x_avg = accel_x_tot / VALS_TO_AVG;
    
    accel_y_vals[accel_idx_new] = accel_y;
    accel_y_tot = accel_y_tot + accel_y_vals[accel_idx_new];
    accel_y_tot = accel_y_tot - accel_y_vals[accel_idx_old];    
    *accel_y_avg = accel_y_tot / VALS_TO_AVG;
 
    accel_z_vals[accel_idx_new] = accel_z;
    accel_z_tot = accel_z_tot + accel_z_vals[accel_idx_new];
    accel_z_tot = accel_z_tot - accel_z_vals[accel_idx_old];    
    *accel_z_avg = accel_z_tot / VALS_TO_AVG;

    // Update the indices 
    if(++accel_idx_new >= VALS_TO_AVG) 
        accel_idx_new = 0;
    if(++accel_idx_old >= VALS_TO_AVG) 
        accel_idx_old = 0;

}

void accel_get_xyz_cal(int16_t* accel_x_cal, int16_t* accel_y_cal, int16_t* accel_z_cal, uint8_t bAvg)
{
    int16_t accel_x, accel_y, accel_z;
    
    if(bAvg)
        accel_get_xyz_avg(&accel_x, &accel_y, &accel_z);
    else
        accel_get_xyz_raw(&accel_x, &accel_y, &accel_z);
    
    *accel_x_cal = accel_x - accel_x_offset;
    *accel_y_cal = accel_y - accel_y_offset;
    *accel_z_cal = accel_z - accel_z_offset;

}
