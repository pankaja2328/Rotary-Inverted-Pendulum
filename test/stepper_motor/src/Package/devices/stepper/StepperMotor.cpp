#include "StepperMotor.h"

StepperMotor::StepperMotor(int stepPin, int dirPin, int enPin)
  : _stepPin(stepPin), _dirPin(dirPin), _enPin(enPin), _enabled(false),
    _mode(MODE_POSITION), _targetSpeed(200.0f), _targetPosition(0),
    _acceleration(500.0f), _minPulse(20.0f), _startingPulse(50.0f), _startPosition(0),
    _sweepMin(-200), _sweepMax(200), _sweepingToMax(true),
    _stepper(AccelStepper::DRIVER, stepPin, dirPin) {}

void StepperMotor::begin(uint32_t updateInterval_us) {
  if (_enPin != -1) {
    pinMode(_enPin, OUTPUT);
    enable(false); // Start disabled for safety
  }

  _stepper.setMaxSpeed(2000.0f);
  _stepper.setAcceleration(_acceleration);
  _stepper.setCurrentPosition(0);

  // Start hardware timer — callback fires every updateInterval_us microseconds
  if (!_timer.begin(_timerCallback, this, updateInterval_us, "StepperUpdate")) {
    Serial.println("[StepperMotor] ERROR: Failed to start hardware timer!");
  }
}

// Static trampoline: esp_timer passes the StepperMotor instance as arg
void StepperMotor::_timerCallback(void* arg) {
  StepperMotor* motor = static_cast<StepperMotor*>(arg);
  motor->update();
}

void StepperMotor::enable(bool enable) {
  _enabled = enable;
  if (_enPin != -1) {
    // Most stepper drivers (A4988, DRV8825, TMC2208) are active LOW for ENABLE
    digitalWrite(_enPin, _enabled ? LOW : HIGH);
  }
}

void StepperMotor::setMode(Mode mode) {
  _mode = mode;
  if (_mode == MODE_SPEED) {
    _stepper.setSpeed(_targetSpeed);
    if (_targetSpeed != 0.0f) {
      enable(true);
    } else {
      enable(false);
    }
  } else if (_mode == MODE_SWEEP) {
    _sweepingToMax = true;
    _startPosition = _stepper.currentPosition();
    _stepper.moveTo(_sweepMax);
    enable(true);
  } else if (_mode == MODE_POSITION) {
    _startPosition = _stepper.currentPosition();
    _stepper.moveTo(_targetPosition);
    if (_stepper.distanceToGo() != 0) {
      enable(true);
    } else {
      enable(false);
    }
  }
}

void StepperMotor::setTargetPosition(long steps) {
  _targetPosition = steps;
  if (_mode == MODE_POSITION) {
    _startPosition = _stepper.currentPosition();
    _stepper.moveTo(_targetPosition);
    if (_stepper.distanceToGo() != 0) {
      enable(true);
    }
  }
}

void StepperMotor::setTargetSpeed(float stepsPerSec) {
  _targetSpeed = stepsPerSec;
  float limit = abs(_targetSpeed) * 2.0f;
  if (limit < 1000.0f) limit = 1000.0f;
  _stepper.setMaxSpeed(limit);
  if (_mode == MODE_SPEED) {
    _stepper.setSpeed(_targetSpeed);
    if (_targetSpeed != 0.0f) {
      enable(true);
    }
  }
}

void StepperMotor::setAcceleration(float stepsPerSecSq) {
  _acceleration = stepsPerSecSq;
  _stepper.setAcceleration(_acceleration);
}

void StepperMotor::setMinPulse(float minPulse) {
  _minPulse = minPulse;
}

void StepperMotor::setStartingPulse(float startingPulse) {
  _startingPulse = startingPulse;
}

void StepperMotor::setSweepLimits(long minVal, long maxVal) {
  _sweepMin = minVal;
  _sweepMax = maxVal;
  if (_mode == MODE_SWEEP) {
    _startPosition = _stepper.currentPosition();
    if (_sweepingToMax) {
      _stepper.moveTo(_sweepMax);
    } else {
      _stepper.moveTo(_sweepMin);
    }
    enable(true);
  }
}

void StepperMotor::stop() {
  _stepper.stop();
  if (_mode == MODE_SPEED) {
    setTargetSpeed(0.0f);
  }
  enable(false);
}

void StepperMotor::update() {
  if (!_enabled) return;

  if (_mode == MODE_POSITION) {
    long distToGo = _stepper.distanceToGo();
    if (distToGo == 0) {
      enable(false);
      return;
    }

    long currentPos = _stepper.currentPosition();
    long totalDist = abs(_targetPosition - _startPosition);
    long distTraveled = abs(currentPos - _startPosition);
    long distLeft = abs(distToGo);

    float Vmax = abs(_targetSpeed);
    float Vstart = min(_startingPulse, Vmax);
    float Vmin = min(_minPulse, Vmax);
    float a = _acceleration;

    float d_accel = (a > 0.0f) ? ((Vmax * Vmax - Vstart * Vstart) / (2.0f * a)) : 0.0f;
    float d_decel = (a > 0.0f) ? ((Vmax * Vmax - Vmin * Vmin) / (2.0f * a)) : 0.0f;

    float d_accel_eff = d_accel;
    float d_decel_eff = d_decel;
    float Vpeak = Vmax;

    if (totalDist < (d_accel + d_decel) && (d_accel + d_decel) > 0.0f) {
      float ratio = (float)totalDist / (d_accel + d_decel);
      d_accel_eff = d_accel * ratio;
      d_decel_eff = d_decel * ratio;
      Vpeak = Vstart + (Vmax - Vstart) * ratio;
    }

    float speedVal = Vpeak;
    if (d_accel_eff > 0.0f && distTraveled < d_accel_eff) {
      float p = (float)distTraveled / d_accel_eff;
      float f = 0.5f * (1.0f - cos(PI * p));
      speedVal = Vstart + (Vpeak - Vstart) * f;
    } else if (d_decel_eff > 0.0f && distLeft < d_decel_eff) {
      float p = (float)distLeft / d_decel_eff;
      float f = 0.5f * (1.0f - cos(PI * p));
      speedVal = Vmin + (Vpeak - Vmin) * f;
    }

    if (distToGo < 0) {
      speedVal = -speedVal;
    }

    _stepper.setSpeed(speedVal);
    _stepper.runSpeed();

  } else if (_mode == MODE_SPEED) {
    if (_targetSpeed != 0.0f) {
      _stepper.runSpeed();
    } else {
      enable(false);
    }
  } else if (_mode == MODE_SWEEP) {
    long distToGo = _stepper.distanceToGo();
    if (distToGo == 0) {
      _sweepingToMax = !_sweepingToMax;
      _startPosition = _stepper.currentPosition();
      _stepper.moveTo(_sweepingToMax ? _sweepMax : _sweepMin);
      distToGo = _stepper.distanceToGo();
    }

    long currentPos = _stepper.currentPosition();
    long targetPos = _sweepingToMax ? _sweepMax : _sweepMin;
    long totalDist = abs(targetPos - _startPosition);
    long distTraveled = abs(currentPos - _startPosition);
    long distLeft = abs(distToGo);

    float Vmax = abs(_targetSpeed);
    float Vstart = min(_startingPulse, Vmax);
    float Vmin = min(_minPulse, Vmax);
    float a = _acceleration;

    float d_accel = (a > 0.0f) ? ((Vmax * Vmax - Vstart * Vstart) / (2.0f * a)) : 0.0f;
    float d_decel = (a > 0.0f) ? ((Vmax * Vmax - Vmin * Vmin) / (2.0f * a)) : 0.0f;

    float d_accel_eff = d_accel;
    float d_decel_eff = d_decel;
    float Vpeak = Vmax;

    if (totalDist < (d_accel + d_decel) && (d_accel + d_decel) > 0.0f) {
      float ratio = (float)totalDist / (d_accel + d_decel);
      d_accel_eff = d_accel * ratio;
      d_decel_eff = d_decel * ratio;
      Vpeak = Vstart + (Vmax - Vstart) * ratio;
    }

    float speedVal = Vpeak;
    if (d_accel_eff > 0.0f && distTraveled < d_accel_eff) {
      float p = (float)distTraveled / d_accel_eff;
      float f = 0.5f * (1.0f - cos(PI * p));
      speedVal = Vstart + (Vpeak - Vstart) * f;
    } else if (d_decel_eff > 0.0f && distLeft < d_decel_eff) {
      float p = (float)distLeft / d_decel_eff;
      float f = 0.5f * (1.0f - cos(PI * p));
      speedVal = Vmin + (Vpeak - Vmin) * f;
    }

    if (distToGo < 0) {
      speedVal = -speedVal;
    }

    _stepper.setSpeed(speedVal);
    _stepper.runSpeed();
  }
}
