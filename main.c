/*******************************************************************
 * Stellaris driverlibs
 *******************************************************************/
#include "grlib/grlib.h"
#include "driverlib/i2c.h"
#include "driverlib/qei.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
//#include "drivers/kitronix320x240x16_ssd2119_8bit.h"
//#include "drivers/set_pinout.h"
#include "driverlib/flash.h"
#include "driverlib/pwm.h"
#include "softeeprom.h"
#include "driverlib/uart.h"
#include "softeeprom.h"
     

     
/*******************************************************************
 * Stellaris hardware
 *******************************************************************/
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

     
/*******************************************************************
 * Standard C libs
 *******************************************************************/
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "math.h"


/*******************************************************************
 * Custom headers
 *******************************************************************/
#include "pid.h"
#include "kalman.h"
#include "adxl345.h"
#include "l3g4200d.h"
#include "motor.h"
#include "i2ctools.h"     
#include "command_handler.h"     
#include "my_uart.h"
#include "my_timers.h"
#include "main.h"

         
/*******************************************************************
 * MACROS
 *******************************************************************/

#define EEPROM_START_ADDR  (0x20000)
#define EEPROM_PAGE_SIZE (0x2000)
#define EEPROM_END_ADDR  (EEPROM_START_ADDR + 4*EEPROM_PAGE_SIZE)


#define K_P 0.0
#define K_I 0.0
#define K_D 0.0

#define RAD_TO_DEG (180.0/3.14159)



t_piddata pid_motor;

uint64_t delta_t, sum_delta_t;

double R, filtered_ang, accel_pitch_ang;

short kP, kI, kD;
short zero_ang;
unsigned short *psKP, *psKI, *psKD, *psANG;
tBoolean *pbFound;

double right_mot_gain=1.0, left_mot_gain=1.0;

int motor_val;
int16_t gyro_x, gyro_y, gyro_z;
int16_t accel_x, accel_y, accel_z;
uint8_t halted_latch = 0;

double g_gyroScale;




////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// MAIN() ////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
int
main(void)
{
    long lEEPROMRetStatus;

    // Set the clocking to run at 80 MHz from the PLL.
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
        
    // Initialize the device pinout appropriately for this board.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    
    // Setup the user LED
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
    
    // Enable processor interrupts.
    IntMasterEnable();
        
    // Setup the UART's        
    my_uart_0_init(115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    my_uart_1_init(115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    
    // Enable the command handler
    command_handler_init();
    
    // Start the timers
    my_timer0_init();
    my_timer1_init();
    
    i2c_init();
    motor_init();
    //qei_init();
    gyro_init();
    accel_init();
    
    // Initialize the EEPROM emulation region.
    lEEPROMRetStatus = SoftEEPROMInit(EEPROM_START_ADDR, EEPROM_END_ADDR, EEPROM_PAGE_SIZE);
    if(lEEPROMRetStatus != 0)
    {
        UART0Send("EEprom ERROR!\n", 14);
    }
    
    SoftEEPROMRead(kP_ID, psKP, pbFound);
    SoftEEPROMRead(kI_ID, psKI, pbFound);
    SoftEEPROMRead(kD_ID, psKD, pbFound);
    SoftEEPROMRead(ANG_ID, psANG, pbFound);
    
    kP = (short)*psKP;
    kI = (short)*psKP;
    kD = (short)*psKP;
    zero_ang = (short)*psANG;
    
    pid_init(kP, kI, kD, &pid_motor);
    
    
    UART0Send("Hello World!\n", 13);
    
    zero_ang = 1656;
    
    print_params();
    
        
    while(1)
    {
        delta_t = myTimerValueGet();
        myTimerZero();
        sum_delta_t += delta_t;

        
        // Read our sensors
        accel_get_xyz_cal(&accel_x, &accel_y, &accel_z, false); 
        gyro_get_xyz_cal(&gyro_x, &gyro_y, &gyro_z, true);
                
        // Calculate the pitch angle with the accelerometer only
        R = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
        accel_pitch_ang = (acos(-accel_x / R)*(RAD_TO_DEG)) - (double)zero_ang/100.0;
        
        // Kalman filter
        filtered_ang = kalman(accel_pitch_ang, ((double)gyro_y)*g_gyroScale, (double)delta_t/10000.0);
        
        // Tell the HMI what's going on every 100ms
        if(sum_delta_t >= 1000)
        {
            print_update();
            print_angle();
            sum_delta_t = 0;
        }
    
        // If we are leaning more than +/- 30 deg off center it's hopeless. SHES GOING DOWN!
        // Turn off the motors in hopes of some damage control
        if( (filtered_ang > 15.0) || (filtered_ang < -15.0) )
        {
            if(halted_latch) continue;
            stop_motors();
            halted_latch = 1;
            continue; 
        }
        halted_latch = 0;
        
        motor_val = ang_controller(0.0, filtered_ang, delta_t, &pid_motor);
        drive_motors((int)((double)motor_val*left_mot_gain), (int)((double)motor_val*right_mot_gain));
        
    }

}
