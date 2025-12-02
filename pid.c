#include "pid.h"

static double clamp(double x, double min_val, double max_val)
{
    if (x > max_val) return max_val;
    if (x < min_val) return min_val;
    return x;
}

void PID_Init(PID *pid,
              double Kp,
              double Ki,
              double Kd,
              double Ts,
              double out_min,
              double out_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->Ts = Ts;

    pid->integrator = 0.0;
    pid->prev_error = 0.0;
    pid->prev_measurement = 0.0;

    pid->out_min = out_min;
    pid->out_max = out_max;
}

void PID_SetTunings(PID *pid, double Kp, double Ki, double Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void PID_SetOutputLimits(PID *pid, double out_min, double out_max)
{
    pid->out_min = out_min;
    pid->out_max = out_max;

    pid->integrator = clamp(pid->integrator, out_min, out_max);
}

void PID_Reset(PID *pid)
{
    pid->integrator = 0.0;
    pid->prev_error = 0.0;
    pid->prev_measurement = 0.0;
}

double PID_Update(PID *pid, double setpoint, double measurement)
{
    double error = setpoint - measurement;

    // Integral term (trapezoidal rule) 
    pid->integrator += 0.5 * pid->Ki * pid->Ts * (error + pid->prev_error);
    pid->integrator = clamp(pid->integrator, pid->out_min, pid->out_max);

    // Derivative term
    double derivative = (measurement - pid->prev_measurement) / pid->Ts;

    double output = pid->Kp * error + pid->integrator - pid->Kd * derivative;
    output = clamp(output, pid->out_min, pid->out_max);

    // Save for next step 
    pid->prev_error = error;
    pid->prev_measurement = measurement;

    return output;
}