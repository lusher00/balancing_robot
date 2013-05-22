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
#include "utils.h"
#include "rc_radio.h"


/*******************************************************************
 * MACROS
 *******************************************************************/
#define EEPROM_START_ADDR  (0x20000)
#define EEPROM_PAGE_SIZE (0x2000)
#define EEPROM_END_ADDR  (EEPROM_START_ADDR + 4*EEPROM_PAGE_SIZE)

#define FALL_ANG 15.0

#define RAD_TO_DEG (180.0/3.14159)
#define CONV_TO_ANG(x) (double)x/100.0
#define CONV_TO_SEC(x) (double)x/10000.0


/*******************************************************************
 * GLOBALS
 *******************************************************************/
t_piddata pid_ang, pid_pos_left, pid_pos_right;
double kP, kI, kD;

uint32_t delta_t, sum_delta_t;

double R, filtered_ang, accel_pitch_ang, zero_ang, rest_ang;

double right_mot_gain=1.0, left_mot_gain=1.0;

int16_t motor_left, motor_right, motor_val;

int16_t gyro_x, gyro_y, gyro_z;
int16_t accel_x, accel_y, accel_z;

double g_gyroScale;

double COMP_C = 0.9975;

const double cMaxLean = 5.0;

double calc_rest_angle(int32_t commanded_pos)
{
	double ang=0.0;
	double error=0.0;
	int32_t pos_right=0, pos_left=0, pos=0;
	static double sum_error = 0.0, last_error=0.0;

	pos_right = QEIPositionGet(QEI0_BASE);
	pos_left = QEIPositionGet(QEI1_BASE);

	pos = (pos_right + pos_left) / 2;

	error = commanded_pos - (double)pos;
	sum_error += error;

	ang = error * 0.0 + sum_error * 0.001 * delta_t + (error - last_error) * 0.01 / delta_t;

	last_error = error;

	if(ang > cMaxLean)
		ang = cMaxLean;
	else if(ang < -cMaxLean)
		ang = -cMaxLean;

	return ang;
}


/*******************************************************************
 * MAIN()
 *******************************************************************/
int
main(void)
{
	long lEEPROMRetStatus;
	uint16_t i=0;
	uint8_t halted_latch = 0;

	// Set the clocking to run at 80 MHz from the PLL.
	// (Well we were at 80MHz with SYSCTL_SYSDIV_2_5 but according to the errata you can't
	// write to FLASH at frequencies greater than 50MHz so I slowed it down. I supposed we
	// could slow the clock down when writing to FLASH but then we need to find out     how long
	// it takes for the clock to stabilize. This on at the bottom of my list of things to do
	// for now.)
	SysCtlClockSet(SYSCTL_SYSDIV_4_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

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
	qei_init();
	gyro_init();
	accel_init();
	led_init();
	rc_radio_init();

	// Initialize the EEPROM emulation region.
	lEEPROMRetStatus = SoftEEPROMInit(EEPROM_START_ADDR, EEPROM_END_ADDR, EEPROM_PAGE_SIZE);
	if(lEEPROMRetStatus != 0) UART0Send("EEprom ERROR!\n", 14);

#if 0
	// If ever we wanted to write some parameters to FLASH without the HMI
	// we could do it here.
	SoftEEPROMWriteDouble(kP_ID, 10.00);
	SoftEEPROMWriteDouble(kI_ID, 10.00);
	SoftEEPROMWriteDouble(kD_ID, 10.00);
	SoftEEPROMWriteDouble(ANG_ID, 0.0);
#endif

	kP = SoftEEPROMReadDouble(kP_ID);
	kI = SoftEEPROMReadDouble(kI_ID);
	kD = SoftEEPROMReadDouble(kD_ID);
	zero_ang = SoftEEPROMReadDouble(ANG_ID);

	pid_init(kP, kI, kD, &pid_ang);
	//pid_init(0.0, 0.0, 0.0, &pid_pos_left);
	//pid_init(0.0, 0.0, 0.0, &pid_pos_right);

	//UART0Send("Hello World!\n", 13);

	// Tell the HMI what the initial parameters are.
	print_params(1);


	while(1)
	{
		delta_t = myTimerValueGet();
		myTimerZero();
		sum_delta_t += delta_t;

		// Read our sensors
		accel_get_xyz_cal(&accel_x, &accel_y, &accel_z, true);
		gyro_get_y_cal(&gyro_y, false);

		// Calculate the pitch angle with the accelerometer only
		R = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
		accel_pitch_ang = (acos(-accel_x / R)*(RAD_TO_DEG)) - zero_ang;
		//accel_pitch_ang = (double)((atan2(accel_x, -accel_z))*RAD_TO_DEG - 90.0);

		// Kalman filter
		filtered_ang = kalman((double)accel_pitch_ang, ((double)gyro_y)*g_gyroScale, CONV_TO_SEC(delta_t));
		//filtered_ang = (COMP_C*(filtered_ang+((double)gyro_y*g_gyroScale*CONV_TO_SEC(delta_t)))) + ((1.0-COMP_C)*(double)accel_pitch_ang);

		// Skip the rest of the process until the angle stabilizes
		if(i < 250) { i++; continue; }

		// Tell the HMI what's going on every 100ms
		if(sum_delta_t >= 1000)
		{
			//print_update(1);
			//print_debug2(1);
			print_pulse_width(0);
			led_toggle();
			//print_angle();
			sum_delta_t = 0;
		}

		// See if the HMI has anything to say
		command_handler();

		continue;

		rest_ang = calc_rest_angle(0);

		// If we are leaning more than +/- FALL_ANG deg off center it's hopeless.
		// Turn off the motors in hopes of some damage control
		if( abs(filtered_ang) > FALL_ANG )
		{
			if(halted_latch) continue;
			stop_motors();
			halted_latch = 1;
			continue;
		}
		halted_latch = 0;

		motor_val = pid_controller(0, filtered_ang, delta_t, &pid_ang);

		motor_left = motor_val;// + pid_controller(0, QEIPositionGet(QEI1_BASE), delta_t, &pid_pos_left);
		motor_right = motor_val;// + pid_controller(0, QEIPositionGet(QEI0_BASE), delta_t, &pid_pos_right);

		drive_motors(motor_left, motor_right);
	}
}
