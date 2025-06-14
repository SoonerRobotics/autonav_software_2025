
#define now() (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())).count();

class PIDController {
public:
    PIDController() {} // for stupid fricking compiler

    PIDController(double kP, double kI, double kD) {
        this->kP = kP;
        this->kI = kI;
        this->kD = kD;
    }

    void setSetpoint(double setpoint) {
        this->setpoint = setpoint;
    }

    double calculate(double reading) {
        double timeElapsed = 0.0;
        if (lastTime == 0.0) {
            timeElapsed = 0.1;
        } else {
            timeElapsed = lastTime - now();
        }
        lastTime = now();

        double error = setpoint - reading;

        double errorDerivative = (error - this->lastError) / timeElapsed;

        this->lastError = error;

        return (this->kP * error) + (this->kI * 0.0 /*TODO FIXME*/) + (this->kD * errorDerivative);
    }

    bool atSetpoint() {
        return this->lastError < 10.0;
    }


private:
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;

    double setpoint = 0.0;
    double lastError = 0.0;
    double lastTime = 0.0;
};