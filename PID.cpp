// PID.cpp
class PIDController {
public:
    PIDController(double kp, double ki, double kd) 
        : kp(kp), ki(ki), kd(kd), integral(0), prev_error(0) {}
    
    double compute(double setpoint, double measurement) {
        double error = setpoint - measurement;
        integral += error;
        double derivative = error - prev_error;
        prev_error = error;
        return kp*error + ki*integral + kd*derivative;
    }
private:
    double kp, ki, kd;
    double integral, prev_error;
};
