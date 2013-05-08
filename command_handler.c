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

char cmd_rx_buff[32];
char *pResult;
char delim[] = {";"};

// Need to find a way to eliminate all these externs...
extern short zero_ang;
extern double right_mot_gain, left_mot_gain;
extern t_piddata pid_motor;
extern double filtered_ang;
extern int motor_val;

extern short kP, kI, kD;

void command_handler_init()
{
    UARTStdioInit(1);
    UARTEchoSet(false);
}

void print_params()
{
    char buff[128];
    
    sprintf(buff, "%s; %d; %d; %d; %d; %d\r\n", 
        "PARAMS", 
        6, 
        kP, 
        kI, 
        kD, 
        zero_ang);
    UART1Send(buff, strlen(buff));
}

void print_update()
{
    char buff[128];
    
    sprintf(buff, "%s; %d; %3.3lf; %d; %3.3lf; %3.3lf; %3.3lf; \r\n", 
        "UPDATE",
        8,
        filtered_ang, 
        motor_val,
        p_get(&pid_motor),
        i_get(&pid_motor),
        d_get(&pid_motor)
          );
    
    UART1Send(buff, strlen(buff));
}

void print_angle()
{
    char buff[32];
    
    sprintf(buff, "ANG = %3.3lf\r\n", filtered_ang);
    UART0Send(buff, strlen(buff));
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
                zero_ang = atoi(strtok(0,delim));
                SoftEEPROMWrite(ANG_ID, zero_ang);
            
            // Update P, I and D gains   
            }else if(strcmp(pResult, "PID") == 0){
                kP = atoi(strtok(0, delim));
                SoftEEPROMWrite(kP_ID, kP);
                kI = atoi(strtok(0, delim));
                SoftEEPROMWrite(kI_ID, kI);
                kD = atoi(strtok(0, delim));
                SoftEEPROMWrite(kD_ID, kD);
                
                pid_update(kP, kI, kD, &pid_motor);
            
            // Update P gain
            }else if(strcmp(pResult, "P") == 0){
                kP = atoi(strtok(0, delim));
                p_update(kP, &pid_motor);
            
            // Update I gain    
            }else if(strcmp(pResult, "I") == 0){
                kI = atoi(strtok(0, delim));
                i_update(kI, &pid_motor);
            
            // Update D gain    
            }else if(strcmp(pResult, "D") == 0){
                kD = atoi(strtok(0, delim));
                d_update(kD, &pid_motor);
           
            // Update the position    
            }else if(strcmp(pResult, "POS") == 0){
                QEIPositionSet(QEI0_BASE, atoi(strtok(0, delim)));
                QEIPositionSet(QEI1_BASE, atoi(strtok(0, delim)));
            
            // Zero out everything    
            }else if(strcmp(pResult, "ZERO") == 0){
                QEIPositionSet(QEI0_BASE, 0);
                QEIPositionSet(QEI1_BASE, 0);
                pid_update(0.0, 0.0, 0.0, &pid_motor);
                i_reset(&pid_motor);
            
            }else if(strcmp(pResult, "MOTL") == 0){
                left_mot_gain = atof(strtok(0, delim));
                
            }else if(strcmp(pResult, "MOTR") == 0){
                right_mot_gain = atof(strtok(0, delim));
                
            }
        }
    }
}
