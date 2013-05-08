#include "motor.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/qei.h"
#include "stdint.h"

void qei_init()
{
    // Enable the quadrature encoder interface  
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);

    // Configure the pins used by the QEI
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_4);  
    GPIOPinTypeQEI(GPIO_PORTE_BASE, GPIO_PIN_2 | GPIO_PIN_3);  
    GPIOPinTypeQEI(GPIO_PORTH_BASE, GPIO_PIN_3);
    
    GPIOPinConfigure(GPIO_PC4_PHA0);
    GPIOPinConfigure(GPIO_PH3_PHB0);
    GPIOPinConfigure(GPIO_PE3_PHA1);
    GPIOPinConfigure(GPIO_PE2_PHB1);
    
    // Configure the QEI itself
    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_RESET | QEI_CONFIG_NO_SWAP), 0xFFFFFFFF);
    QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_RESET | QEI_CONFIG_NO_SWAP), 0xFFFFFFFF);
    
    QEIEnable(QEI0_BASE);
    QEIEnable(QEI1_BASE);
    
    QEIVelocityEnable(QEI0_BASE);
    QEIVelocityEnable(QEI1_BASE);
    
    // 1856 cnts/rev *
    // QEIVelocityGet returns ints in units of cnts per (80,000/80,000,000) seconds
    // (cnts/ms)*(1 rev/1856 cnts)*(1000ms/sec)*(60sec/min)
    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, 800000);
    QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_1, 800000);
    
    QEIPositionSet(QEI0_BASE, 0);
    QEIPositionSet(QEI1_BASE, 0);
}

int32_t pos_get()
{
    static int32_t current_pos=0, last_pos=0, abs_pos=0;

    last_pos = current_pos;
    current_pos = QEIPositionGet(QEI0_BASE);
    abs_pos += (current_pos - last_pos);
        
    return abs_pos;
}

void motor_init()
{
    // Setup the DIR pins on the motor controller
    // PJ6 = 2INA, PJ5 = 2INB, PJ3 = 1INA, PJ4 = 1INB
    GPIOPinTypeGPIOOutput(GPIO_PORTJ_BASE, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_3, ~GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_4, ~GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_6, ~GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_5, ~GPIO_PIN_5);
    
    // Enable the PWM Generator
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_4);

    // Set PG7 to be controlled by PWM Generator 3
    GPIOPinConfigure(GPIO_PG7_PWM7);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_7);
    PWMGenConfigure(PWM_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC | PWM_GEN_MODE_DBG_RUN);
    PWMDeadBandDisable(PWM_BASE, PWM_GEN_3);
    // Set PWM frequency
    PWMGenPeriodSet(PWM_BASE, PWM_GEN_3, PWM_PERIOD);
    // Initiall pulsewidth is 0
    PWMPulseWidthSet(PWM_BASE, PWM_OUT_7, 0);
    PWMOutputState(PWM_BASE, PWM_OUT_7_BIT, true);
    PWMGenEnable(PWM_BASE, PWM_GEN_3);
    
    // Set PH1 to be controlled by PWM Generator 1   
    GPIOPinConfigure(GPIO_PH1_PWM3);
    GPIOPinTypePWM(GPIO_PORTH_BASE, GPIO_PIN_1);
    PWMGenConfigure(PWM_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC | PWM_GEN_MODE_DBG_RUN);
    PWMDeadBandDisable(PWM_BASE, PWM_GEN_1);
    // Set PWM frequency
    PWMGenPeriodSet(PWM_BASE, PWM_GEN_1, PWM_PERIOD);
    // Initiall pulsewidth is 0
    PWMPulseWidthSet(PWM_BASE, PWM_OUT_3, 0);
    PWMOutputState(PWM_BASE, PWM_OUT_3_BIT, true);
    PWMGenEnable(PWM_BASE, PWM_GEN_1);
}

void drive_motors(int16_t left, int16_t right)
{
    if(left >= 0){
        GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_6, ~GPIO_PIN_6);
        GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_5, GPIO_PIN_5);
    }else{
        GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_6, GPIO_PIN_6);
        GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_5, ~GPIO_PIN_5);
        left *= -1;
    }
    
    if(right >= 0){
        GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_3, GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_4, ~GPIO_PIN_4);
    }else{
        GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_3, ~GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_4, GPIO_PIN_4);
        right *= -1;
    }
        
    if(right < 4) right = 0;
    if(right > PWM_PERIOD) right = PWM_PERIOD;
    if(left < 4) left = 0;
    if(left > PWM_PERIOD) left = PWM_PERIOD;
    
    PWMPulseWidthSet(PWM_BASE, PWM_OUT_7, left);
    PWMPulseWidthSet(PWM_BASE, PWM_OUT_3, right);
}

void stop_motors()
{
    PWMPulseWidthSet(PWM_BASE, PWM_OUT_7, 0);
    PWMPulseWidthSet(PWM_BASE, PWM_OUT_3, 0);
}
