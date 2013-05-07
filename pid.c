#include "pid.h"
#include "motor.h"


// Set up PID controller parameters
void pid_init(short p_factor, short i_factor, short d_factor, t_piddata *pid)
{
   // Start values for PID controller
   pid->sumError = 0.0;
   pid->lastProcessValue = 0.0;
   pid->lastError = 0.0;
   
   // Tuning constants for PID loop
   pid->P_Factor = p_factor;
   pid->I_Factor = i_factor;
   pid->D_Factor = d_factor;
   
   pid->integrator_idx = 0;
   pid->integrator_idx_old = 1;
   
   // Limits to avoid overflow
   pid->maxError = (double)(MAX_INT / (pid->P_Factor + 1));
   pid->maxSumError = (double)(MAX_I_TERM / (pid->I_Factor + 1));
}

void pid_update(short p_factor, short i_factor, short d_factor, t_piddata *pid)
{
    // Tuning constants for PID loop
   pid->P_Factor = p_factor;
   pid->I_Factor = i_factor;
   pid->D_Factor = d_factor;    
}

void i_reset(t_piddata *pid)
{
   pid->i_term = 0.0;
}

void p_update(short p_factor, t_piddata *pid)
{
   pid->P_Factor = p_factor;
}

void i_update(short i_factor, t_piddata *pid)
{
   pid->I_Factor = i_factor;
}

void d_update(short d_factor, t_piddata *pid)
{
   pid->D_Factor = d_factor;    
}

double p_get(t_piddata *pid)
{
   return (pid->p_term/SCALING_FACTOR);    
}

double i_get(t_piddata *pid)
{
   return (pid->i_term/SCALING_FACTOR);    
}

double d_get(t_piddata *pid)
{
   return (pid->d_term/SCALING_FACTOR);    
}

int16_t ang_controller(double setPoint, double processValue, double delta_t, t_piddata *pid_st)
{
   int32_t ret=0;
   double temp=0.0;
   
   // Calculate the current error
   pid_st->error = (setPoint - processValue);
   
   // Calculate Pterm and limit error overflow
   if (pid_st->error > pid_st->maxError){
      pid_st->p_term = MAX_INT;
   }else if (pid_st->error < -pid_st->maxError){
      pid_st->p_term = -MAX_INT;
   }else{
      pid_st->p_term = pid_st->P_Factor * pid_st->error;
   }
   
   // Calculate Iterm and limit integral runaway
   temp = pid_st->sumError + pid_st->error;
   if(temp > pid_st->maxSumError){
      pid_st->i_term = MAX_I_TERM;
      pid_st->sumError = pid_st->maxSumError;
   }
   else if(temp < -pid_st->maxSumError){
      pid_st->i_term = -MAX_I_TERM;
      pid_st->sumError = -pid_st->maxSumError;
   }
   else{
      pid_st->sumError = pid_st->I_Factor * temp;
      pid_st->i_term = pid_st->sumError;
   }
   
   // Calculate Dterm
   pid_st->d_term = pid_st->D_Factor * (processValue - pid_st->lastProcessValue) / (delta_t*10.0);
   
   // Save the current process value for later
   pid_st->lastProcessValue = processValue;
   
   // Save the current error for later
   pid_st->lastError = pid_st->error;

   // Add up P I & D and divide by a scaling factor
   ret = (int32_t)((pid_st->p_term + pid_st->i_term + pid_st->d_term) / SCALING_FACTOR);
   
   // Limit the size of the return value
   if(ret > PWM_PERIOD)
      ret = PWM_PERIOD;
   else if(ret < -PWM_PERIOD)
      ret = -PWM_PERIOD;
   
   return((int16_t)ret);
}

