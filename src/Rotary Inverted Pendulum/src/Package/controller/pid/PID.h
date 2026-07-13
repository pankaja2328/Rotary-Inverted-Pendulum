#ifndef PID_H
#define PID_H

class PID {
public:
    PID(double kp, double ki, double kd, double minOut = -1000.0, double maxOut = 1000.0);

    void setTunings(double kp, double ki, double kd);
    void setOutputLimits(double minOut, double maxOut);
    void reset();

    double compute(double input, double setpoint, double dt);

    double getKp() const { return _kp; }
    double getKi() const { return _ki; }
    double getKd() const { return _kd; }

private:
    double _kp;
    double _ki;
    double _kd;

    double _minOut;
    double _maxOut;

    double _integral;
    double _lastError;
    double _lastInput;
    bool _firstRun;
};

#endif // PID_H
