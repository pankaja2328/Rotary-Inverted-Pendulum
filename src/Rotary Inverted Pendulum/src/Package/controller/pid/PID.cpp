#include "PID.h"
#include <math.h>

PID::PID(double kp, double ki, double kd, double minOut, double maxOut)
    : _kp(kp), _ki(ki), _kd(kd), _minOut(minOut), _maxOut(maxOut),
      _integral(0.0), _lastError(0.0), _lastInput(0.0), _firstRun(true) {}

void PID::setTunings(double kp, double ki, double kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
}

void PID::setOutputLimits(double minOut, double maxOut) {
  _minOut = minOut;
  _maxOut = maxOut;
}

void PID::reset() {
  _integral = 0.0;
  _lastError = 0.0;
  _lastInput = 0.0;
  _firstRun = true;
}

double PID::compute(double input, double setpoint, double dt) {
  if (dt <= 0.0) {
    return 0.0;
  }

  double error = setpoint - input;

  // FIX: deadband was 36.0 degrees, which zeroed ALL correction within
  // +/-36 deg of setpoint -- exactly the region an inverted pendulum needs
  // continuous fine correction in. Reduced to a small noise-rejection band
  // only. Tune this down further (even to 0.0) if the encoder is clean.
  constexpr double errorDeadband = 5;
  if (fabs(error) <= errorDeadband) {
    error = 0.0;
    // NOTE: no longer zeroing _integral here -- wiping the integral term
    // every single loop while near setpoint prevented it from ever holding
    // a steady-state correction. Integral is now only reset explicitly
    // via PID::reset(), not implicitly inside the deadband.
  }

  double pOut = _kp * error;

  if (!_firstRun) {
    _integral += error * dt;
  } else {
    _firstRun = false;
  }

  double iOut = _ki * _integral;

  double dOut = 0.0;
  if (!_firstRun) {
    dOut = _kd * ((error - _lastError) / dt);
  }

  double output = pOut + iOut + dOut;

  if (output > _maxOut) {
    output = _maxOut;
    _integral -= error * dt;
  } else if (output < _minOut) {
    output = _minOut;
    _integral -= error * dt;
  }

  _lastError = error;
  _lastInput = input;

  return output;
}