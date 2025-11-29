#ifndef PID_HPP
#define PID_HPP

class PID {
public:
    PID(double Kp, double Ki, double Kd,
        double Ts,
        double out_min, double out_max)
    : Kp_(Kp), Ki_(Ki), Kd_(Kd),
      Ts_(Ts),
      integrator_(0.0),
      prev_error_(0.0),
      prev_measurement_(0.0),
      out_min_(out_min),
      out_max_(out_max)
    {}

    void setTunings(double Kp, double Ki, double Kd) {
        Kp_ = Kp;
        Ki_ = Ki;
        Kd_ = Kd;
    }

    void setOutputLimits(double out_min, double out_max) {
        out_min_ = out_min;
        out_max_ = out_max;
        integrator_ = clamp(integrator_);
    }

    void reset() {
        integrator_      = 0.0;
        prev_error_      = 0.0;
        prev_measurement_ = 0.0;
    }

    double update(double setpoint, double measurement) {
        double error = setpoint - measurement;

        integrator_ += 0.5 * Ki_ * Ts_ * (error + prev_error_);
        integrator_ = clamp(integrator_);

        double derivative = (measurement - prev_measurement_) / Ts_;

        double output = Kp_ * error + integrator_ - Kd_ * derivative;
        output = clamp(output);

        prev_error_ = error;
        prev_measurement_ = measurement;

        return output;
    }

private:
    double clamp(double x) const {
        if (x > out_max_) return out_max_;
        if (x < out_min_) return out_min_;
        return x;
    }

    double Kp_;
    double Ki_;
    double Kd_;
    double Ts_;

    double integrator_;
    double prev_error_;
    double prev_measurement_;

    double out_min_;
    double out_max_;
};

#endif