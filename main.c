#include <stdio.h>
#include "pid.h"
#include "pid.c"


/* Simple first-order plant: y' = (-y + u)/tau */
static double plant_step(double y, double u, double Ts, double tau)
{
    double dy = (-y + u) / tau;
    return y + Ts * dy;
}

int main(void)
{
    PID pid;
    double Ts = 0.01;       /* 10 ms sample time */
    double out_min = -10.0;
    double out_max =  10.0;

    PID_Init(&pid, 1.0, 0.0, 0.0, Ts, out_min, out_max);

    double tau = 0.5;       /* plant time constant (s) */
    double setpoint = 1.0;  /* step target */

    while (1) {
        double Kp, Ki, Kd;
        printf("\nEnter Kp Ki Kd (or Ctrl+D to quit): ");
        if (scanf("%lf %lf %lf", &Kp, &Ki, &Kd) != 3) {
            printf("Done.\n");
            break;
        }

        PID_SetTunings(&pid, Kp, Ki, Kd);
        PID_Reset(&pid);

        double y = 0.0;   /* plant output */
        double t = 0.0;

        printf("t\tsetpoint\ty\tu\n");
        printf("-----------------------------\n");

        for (int k = 0; k < 400; ++k) {   /* ~4 seconds */
            double u = PID_Update(&pid, setpoint, y);
            y = plant_step(y, u, Ts, tau);

            printf("%.3f\t%.3f\t%.3f\t%.3f\n", t, setpoint, y, u);

            t += Ts;
        }
    }

    return 0;
}