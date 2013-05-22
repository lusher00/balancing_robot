#include "inc/hw_memmap.h"
#include "command_handler.h"
#include "uartstdio.h"
#include "my_uart.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "pid.h"
#include "driverlib/qei.h"
#include "softeeprom.h"
#include "main.h"
#include "rc_radio.h"

char cmd_rx_buff[32];
char *pResult;
char delim[] = {";"};

// Need to find a way to eliminate all these externs...
extern double zero_ang;
extern double right_mot_gain, left_mot_gain;
extern t_piddata pid_ang;
extern double filtered_ang, rest_ang;
extern int motor_val;

extern double kP, kI, kD;


extern uint32_t delta_t;
extern int16_t gyro_x, gyro_y, gyro_z;
extern int16_t accel_x, accel_y, accel_z;

extern uint32_t pulse_width;



char buff[128];

void command_handler_init()
{
	UARTStdioInit(1);
	UARTEchoSet(false);
}

void print_debug(uint8_t uart)
{
	memset(buff, '\0', sizeof(buff));

	sprintf(buff, "%s; %5d; %5d; %5d; %5d; %5d; %5d; %5d; %3.3lf; %3d; %3d; %3.3lf\r\n",
			"DEBUG",
			delta_t,
			accel_x,
			accel_y,
			accel_z,
			gyro_x,
			gyro_y,
			gyro_z,
			filtered_ang,
			QEIPositionGet(QEI0_BASE), // right
			QEIPositionGet(QEI1_BASE), // left
			rest_ang
			);
	UARTSend(buff, strlen(buff), uart);
}

void print_debug2(uint8_t uart)
{
	memset(buff, '\0', sizeof(buff));

	sprintf(buff, "%s; %5d; %3.3lf; %3.3lf; %3d; %3d\r\n",
			"DEBUG2",
			delta_t,
			filtered_ang,
			rest_ang,
			QEIPositionGet(QEI0_BASE), // right
			QEIPositionGet(QEI1_BASE) // left
			);
	UARTSend(buff, strlen(buff), uart);
}

void print_pulse_width(uint8_t uart)
{
	memset(buff, '\0', sizeof(buff));

		sprintf(buff, "%s; %5d\r\n",
				"PULSE WIDTH = ",
				pulse_width
				);
	UARTSend(buff, strlen(buff), uart);
}

void print_params(uint8_t uart)
{
	memset(buff, '\0', sizeof(buff));

	sprintf(buff, "%s; %d; %3.3lf; %3.3lf; %3.3lf; %3.3lf\r\n",
			"PARAMS",
			6,
			kP,
			kI,
			kD,
			zero_ang);
	UARTSend(buff, strlen(buff), uart);
}

void print_update(uint8_t uart)
{
	memset(buff, 0, sizeof(buff));

	sprintf(buff, "UPDATE; %d; %3.3lf; %d; %3.3lf; %3.3lf; %3.3lf; \r\n",
			//"UPDATE",
			8,
			filtered_ang,
			motor_val,
			p_get(&pid_ang),
			i_get(&pid_ang),
			d_get(&pid_ang)
	);

	UARTSend(buff, strlen(buff), uart);
}

void print_angle(uint8_t uart)
{
	char buff[32];

	sprintf(buff, "ANG = %3.3lf\r\n", filtered_ang);
	UARTSend(buff, strlen(buff), uart);
}

void command_handler()
{
	// Do we have any UART data to update the PID parameters?
	if(UARTPeek('\r') != -1)
	{
		// We've got a carriage return atleast
		if(UARTgets(cmd_rx_buff, sizeof(cmd_rx_buff)))
		{
			// Yup, its there. Let's parse it out
			pResult = (char*)strtok(cmd_rx_buff, delim);

			// Changed the angle at which the bot balances
			if(strcmp(pResult, "ANG") == 0){
				zero_ang = atof(strtok(0,delim));
				//SoftEEPROMWrite(ANG_ID, zero_ang);

			// Changed the angle at which the bot balances
			}else if(strcmp(pResult, "ANG!") == 0){
				zero_ang = atof(strtok(0,delim));
				SoftEEPROMWriteDouble(ANG_ID, zero_ang);

					// Update P, I and D gains
			}else if(strcmp(pResult, "PID!") == 0){
				kP = atof(strtok(0, delim));
				kI = atof(strtok(0, delim));
				kD = atof(strtok(0, delim));

				SoftEEPROMWriteDouble(kP_ID, kP);
				SoftEEPROMWriteDouble(kI_ID, kI);
				SoftEEPROMWriteDouble(kD_ID, kD);

				pid_update(kP, kI, kD, &pid_ang);

			// Update P gain
			}else if(strcmp(pResult, "P") == 0){
				kP = atof(strtok(0, delim));
				p_update(kP, &pid_ang);

			// Update P gain
			}else if(strcmp(pResult, "P!") == 0){
				kP = atof(strtok(0, delim));
				p_update(kP, &pid_ang);
				SoftEEPROMWriteDouble(kP_ID, kP);

			// Update I gain
			}else if(strcmp(pResult, "I") == 0){
				kI = atof(strtok(0, delim));
				i_update(kI, &pid_ang);

			// Update I gain
			}else if(strcmp(pResult, "I!") == 0){
				kI = atof(strtok(0, delim));
				i_update(kI, &pid_ang);
				SoftEEPROMWriteDouble(kI_ID, kI);

			// Update D gain
			}else if(strcmp(pResult, "D") == 0){
				kD = atof(strtok(0, delim));
				d_update(kD, &pid_ang);

			// Update D gain
			}else if(strcmp(pResult, "D!") == 0){
				kD = atof(strtok(0, delim));
				d_update(kD, &pid_ang);
				SoftEEPROMWriteDouble(kD_ID, kD);

			// Update the position
			}else if(strcmp(pResult, "POS") == 0){
				QEIPositionSet(QEI0_BASE, atoi(strtok(0, delim)));
				QEIPositionSet(QEI1_BASE, atoi(strtok(0, delim)));

			// Zero out everything
			}else if(strcmp(pResult, "ZERO") == 0){
				//QEIPositionSet(QEI0_BASE, 0);
				//QEIPositionSet(QEI1_BASE, 0);
				pid_update(0.0, 0.0, 0.0, &pid_ang);
				i_reset(&pid_ang);

			}else if(strcmp(pResult, "MOTL") == 0){
				left_mot_gain = atof(strtok(0, delim));

			}else if(strcmp(pResult, "MOTR") == 0){
				right_mot_gain = atof(strtok(0, delim));

			}
		}
	}
}
