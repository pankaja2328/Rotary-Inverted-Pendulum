#include "DRV8825.h"
#include <math.h>

DRV8825::DRV8825(int stepPin, int dirPin, int enablePin)
    : _stepPin(stepPin), _dirPin(dirPin), _enablePin(enablePin),
      _enabled(false), _requestedRate(0.0f), _currentRate(0.0f),
      _prevRampUs(0), _prevStepUs(0), _positionSteps(0) {}

void DRV8825::begin() {
    pinMode(_stepPin,   OUTPUT);
    pinMode(_dirPin,    OUTPUT);
    pinMode(_enablePin, OUTPUT);
    digitalWrite(_stepPin,  LOW);
    digitalWrite(_dirPin,   LOW);
    disable();
}

void DRV8825::enable() {
    _enabled = true;
    digitalWrite(_enablePin, LOW);   // DRV8825 ENABLE is active LOW
}

void DRV8825::disable() {
    _enabled = false;
    digitalWrite(_enablePin, HIGH);
}

void DRV8825::setRate(float stepsPerSecond) {
    _requestedRate = stepsPerSecond;
}

void DRV8825::updateRamp(float accelerationStepsS2) {
    unsigned long now = micros();
    if (_prevRampUs == 0) {
        _prevRampUs  = now;
        _currentRate = 0.0f;
        return;
    }
    unsigned long elapsed = (unsigned long)(now - _prevRampUs);
    if (elapsed < 500UL) return;  // update at most every 500 us

    float elapsedS  = elapsed / 1000000.0f;
    float maxChange = accelerationStepsS2 * elapsedS;
    float diff      = _requestedRate - _currentRate;
    if (diff >  maxChange) diff =  maxChange;
    if (diff < -maxChange) diff = -maxChange;
    _currentRate += diff;
    _prevRampUs   = now;
}

void DRV8825::step() {
    float absRate = fabsf(_currentRate);
    if (absRate < 8.0f) return;

    unsigned long stepIntervalUs =
        (unsigned long)(1000000.0f / absRate);

    unsigned long now = micros();
    if ((unsigned long)(now - _prevStepUs) < stepIntervalUs) return;

    int dir = _currentRate > 0.0f ? 1 : -1;
    digitalWrite(_dirPin, dir > 0 ? HIGH : LOW);
    delayMicroseconds(3);
    digitalWrite(_stepPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(_stepPin, LOW);

    _positionSteps += dir;
    _prevStepUs     = now;
}

void DRV8825::clearRateCommand() {
    _requestedRate = 0.0f;
    _currentRate   = 0.0f;
    _prevStepUs    = micros();
    _prevRampUs    = micros();
}
