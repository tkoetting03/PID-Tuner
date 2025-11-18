#include <iostream>
#include "pid.hpp"

/* Simple first-order plant: y' = (-y + u)/tau */
double plant_step(double y, double u, double Ts, double tau)
{
    double dy = (-y + u) / tau;
    return y + Ts * dy;
}

int main()
{
    double Ts      = 0.01;  // 10 ms
    double out_min = -10.0;
    double out_max =  10.0;

    PID pid(1.0, 0.0, 0.0, Ts, out_min, out_max);

    double tau      = 0.5;
    double setpoint = 1.0;

    while (true) {
        double Kp, Ki, Kd;
        std::cout << "\nEnter Kp Ki Kd (Ctrl+D / Ctrl+Z to quit): ";
        if (!(std::cin >> Kp >> Ki >> Kd)) {
            std::cout << "Done.\n";
            break;
        }

        pid.setTunings(Kp, Ki, Kd);
        pid.reset();

        double y = 0.0;
        double t = 0.0;

        std::cout << "t\tsetpoint\ty\tu\n";
        std::cout << "-----------------------------\n";

        for (int k = 0; k < 400; ++k) {
            double u = pid.update(setpoint, y);
            y = plant_step(y, u, Ts, tau);

            std::cout << t << "\t"
                      << setpoint << "\t"
                      << y << "\t"
                      << u << "\n";

            t += Ts;
        }
    }

    return 0;
}