#ifndef PID_H
#define PID_H

#include "stdint.h"


#define ERROR_ARRAY_SIZE 1024

#define SCALING_FACTOR  1.0

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
  
  double P_Factor;
  double I_Factor;
  double D_Factor;
    
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

extern void pid_init(double p_factor, double i_factor, double d_factor, t_piddata *pid);
extern int16_t pid_controller(double setPoint, double processValue, double delta_t, t_piddata *pid_st);

extern void pid_update(double p_factor, double i_factor, double d_factor, t_piddata *pid);
extern void i_reset(t_piddata *pid);
extern void p_update(double p_factor, t_piddata *pid);
extern void i_update(double i_factor, t_piddata *pid);
extern void d_update(double d_factor, t_piddata *pid);
extern double p_get(t_piddata *pid);
extern double i_get(t_piddata *pid);
extern double d_get(t_piddata *pid);

#endif
