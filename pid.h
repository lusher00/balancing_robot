#ifndef PID_H
#define PID_H

#include "stdint.h"


#define ERROR_ARRAY_SIZE 1024

#define SCALING_FACTOR  1024.0

typedef struct MOTOR_DATA{
    int16_t lastProcessValue;
    int32_t sumError;
    uint16_t P_Factor;
    uint16_t I_Factor;
    uint16_t D_Factor;
    int16_t maxError;
    int32_t maxSumError;
}t_motorData;



#define MAX_INT         32767.0
#define MAX_LONG        2147483647.0
#define MAX_I_TERM      (MAX_LONG / 2)
  
typedef struct PID_DATA{
  
  double lastProcessValue;
  
  short P_Factor;
  short I_Factor;
  short D_Factor;
    
  double p_term;
  double d_term;
  double i_term;
  
  double error;
  double lastError;
  double error_array[ERROR_ARRAY_SIZE];
  double sumError;
  
  uint16_t integrator_idx;
  uint16_t integrator_idx_old;
  
  double maxError;
  double maxSumError;
    
} t_piddata;



// Boolean values
#define FALSE           0
#define TRUE            1

extern void pid_init(short p_factor, short i_factor, short d_factor, t_piddata *pid);
extern int16_t ang_controller(double setPoint, double processValue, double delta_t, t_piddata *pid_st);

extern void pid_update(short p_factor, short i_factor, short d_factor, t_piddata *pid);
extern void i_reset(t_piddata *pid);
extern void p_update(short p_factor, t_piddata *pid);
extern void i_update(short i_factor, t_piddata *pid);
extern void d_update(short d_factor, t_piddata *pid);
extern double p_get(t_piddata *pid);
extern double i_get(t_piddata *pid);
extern double d_get(t_piddata *pid);

#endif
