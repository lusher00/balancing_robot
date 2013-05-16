#include "stdint.h"
#include "driverlib/i2c.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "l3g4200d.h"
#include "i2ctools.h"

int16_t gyro_x_offset=0, gyro_y_offset=0, gyro_z_offset=0;

void gyro_get_xyz_raw(int16_t* gyro_x_raw, int16_t* gyro_y_raw, int16_t* gyro_z_raw)
{
    unsigned long data[6];
    
    i2c_rx_multi(GYRO_ADDRESS, L3G4200D_OUT_X_L | 0x80 , 6, data);
    
    *gyro_x_raw = ((int16_t)data[0])<<8;
    *gyro_x_raw |= (int16_t)data[1];
   
    *gyro_y_raw = ((int16_t)data[2])<<8;
    *gyro_y_raw |= (int16_t)data[3];
   
    *gyro_z_raw = ((int16_t)data[4])<<8;
    *gyro_z_raw |= (int16_t)data[5];    
}

uint8_t gyro_idx_new=0, gyro_idx_old=1;
int16_t gyro_x_vals[10], gyro_y_vals[10], gyro_z_vals[10];
int16_t gyro_x_tot=0, gyro_y_tot=0, gyro_z_tot=0;
void gyro_get_xyz_avg(int16_t* gyro_x_avg, int16_t* gyro_y_avg, int16_t* gyro_z_avg)
{
    int16_t gyro_x, gyro_y, gyro_z;
        
    gyro_get_xyz_raw(&gyro_x, &gyro_y, &gyro_z);
    
    gyro_x_vals[gyro_idx_new] = gyro_x;
    gyro_x_tot = gyro_x_tot + gyro_x_vals[gyro_idx_new];
    gyro_x_tot = gyro_x_tot - gyro_x_vals[gyro_idx_old];    
    *gyro_x_avg = gyro_x_tot / 10;
    
    gyro_y_vals[gyro_idx_new] = gyro_y;
    gyro_y_tot = gyro_y_tot + gyro_y_vals[gyro_idx_new];
    gyro_y_tot = gyro_y_tot - gyro_y_vals[gyro_idx_old];    
    *gyro_y_avg = gyro_y_tot / 10;
 
    gyro_z_vals[gyro_idx_new] = gyro_z;
    gyro_z_tot = gyro_z_tot + gyro_z_vals[gyro_idx_new];
    gyro_z_tot = gyro_z_tot - gyro_z_vals[gyro_idx_old];    
    *gyro_z_avg = gyro_z_tot / 10;

    // Update the indices 
    if(++gyro_idx_new > 9) 
        gyro_idx_new = 0;
    if(++gyro_idx_old > 9) 
        gyro_idx_old = 0;

}

void gyro_get_xyz_cal(int16_t* gyro_x_cal, int16_t* gyro_y_cal, int16_t* gyro_z_cal, uint8_t bAvg)
{
    int16_t gyro_x, gyro_y, gyro_z;
    
    if(bAvg)
        gyro_get_xyz_avg(&gyro_x, &gyro_y, &gyro_z);
    else
        gyro_get_xyz_raw(&gyro_x, &gyro_y, &gyro_z);
    
    *gyro_x_cal = gyro_x - gyro_x_offset;
    *gyro_y_cal = gyro_y - gyro_y_offset;
    *gyro_z_cal = gyro_z - gyro_z_offset;

}

void gyro_init()
{
    int i;
    int16_t gyro_x=0, gyro_y=0, gyro_z=0;
    int32_t gyro_x_cal=0, gyro_y_cal=0, gyro_z_cal=0;
    
    //  Power down
    i2c_tx_single(GYRO_ADDRESS, L3G4200D_CTRL_REG1, 0x00);
        
    //  Configure CTRL_REG2
    i2c_tx_single(GYRO_ADDRESS, L3G4200D_CTRL_REG2, 0x09);
        
    //  Configure CTRL_REG3
    i2c_tx_single(GYRO_ADDRESS, L3G4200D_CTRL_REG3, 0x08);
        
    //  Configure CTRL_REG4
    //  Continuous update
    //  MSB @ lower address
    //  L3G4200D_DPS_250, L3G4200D_DPS_500, L3G4200D_DPS_1000, L3G4200D_DPS_2000
    i2c_tx_single(GYRO_ADDRESS, L3G4200D_CTRL_REG4, 0xC0 | L3G4200D_DPS_250);
    
    // Just set this here so I can globaly change the gyro resolution in one place.
    g_gyroScale = 0.00875; //250dps;
    //g_gyroScale = 0.01750; //500dps;
    //g_gyroScale = 0.07000; //2000dps;
    
    //  Configure CTRL_REG5
    // Enable HP & LP filters
    i2c_tx_single(GYRO_ADDRESS, L3G4200D_CTRL_REG5, 0x12);
        
    //  Configure CTRL_REG1
    //  Wake up the device 0b00001000
    //  Enable ZYX axes    0b00000ZYX
    //  and set the output data rate to 800Hz
    i2c_tx_single(GYRO_ADDRESS, L3G4200D_CTRL_REG1, 0x4F);
        
    //  Calibrate the gyro
    for(i=0; i<1000; i++)
    {
        gyro_get_xyz_raw(&gyro_x, &gyro_y, &gyro_z);
        gyro_x_cal += gyro_x;
        gyro_y_cal += gyro_y;
        gyro_z_cal += gyro_z;
    }
    
    gyro_x_offset = gyro_x_cal / 1000;
    gyro_y_offset = gyro_y_cal / 1000;
    gyro_z_offset = gyro_z_cal / 1000;
}
