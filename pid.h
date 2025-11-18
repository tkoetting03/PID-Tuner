#ifndef PID_H
#define PID_H

typedef struct {
    double Kp;
    double Ki;
    double Kd;

    double Ts;              /* sample time (seconds) */

    double integrator;
    double prev_error;
    double prev_measurement;

    double out_min;
    double out_max;
} PID;

/* Initialize PID object */
void PID_Init(PID *pid,
              double Kp,
              double Ki,
              double Kd,
              double Ts,
              double out_min,
              double out_max);

/* Change tunings at runtime */
void PID_SetTunings(PID *pid, double Kp, double Ki, double Kd);

/* Change output limits at runtime */
void PID_SetOutputLimits(PID *pid, double out_min, double out_max);

/* Reset internal state (I term, history) */
void PID_Reset(PID *pid);

/* One PID update step */
double PID_Update(PID *pid, double setpoint, double measurement);

#endif /* PID_H */