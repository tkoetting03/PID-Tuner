#ifndef PID_H
#define PID_H

typedef struct {
    double Kp;
    double Ki;
    double Kd;

    double Ts;

    double integrator;
    double prev_error;
    double prev_measurement;

    double out_min;
    double out_max;
} PID;

void PID_Init(PID *pid,
              double Kp,
              double Ki,
              double Kd,
              double Ts,
              double out_min,
              double out_max);

void PID_SetTunings(PID *pid, double Kp, double Ki, double Kd);

void PID_SetOutputLimits(PID *pid, double out_min, double out_max);

void PID_Reset(PID *pid);

double PID_Update(PID *pid, double setpoint, double measurement);

#endif 