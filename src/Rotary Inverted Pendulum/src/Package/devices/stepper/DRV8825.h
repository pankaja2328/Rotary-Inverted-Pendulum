#pragma once
#include <Arduino.h>

/*
  DRV8825 - Direct loop-driven stepper motor class.
  Replicates the working monolithic code rate-ramp approach.
  No hardware timer ISR - pulses are bit-banged in the main loop.

  Call order each loop():
    1. drv.updateRamp(accel);  // advance the rate ramp
    2. drv.step();             // emit STEP pulse if interval elapsed
*/

class DRV8825 {
public:
    DRV8825(int stepPin, int dirPin, int enablePin);

    void begin();
    void enable();   // ENABLE pin driven LOW (active LOW on DRV8825)
    void disable();  // ENABLE pin driven HIGH
    bool isEnabled() const { return _enabled; }

    // Set desired rate (steps/s, signed: positive=CW, negative=CCW)
    void setRate(float stepsPerSecond);

    // Advance acceleration ramp toward requested rate - call from loop()
    void updateRamp(float accelerationStepsS2 = 7000.0f);

    // Emit one STEP pulse if the step interval has elapsed - call from loop()
    void step();

    // Instantly zero both requested and current rate, reset step timer
    void clearRateCommand();

    long  getPositionSteps()     const { return _positionSteps; }
    void  setPositionSteps(long pos)   { _positionSteps = pos; }
    float getCurrentRateStepsS() const { return _currentRate; }
    float getRequestedRateStepsS() const { return _requestedRate; }

private:
    int  _stepPin;
    int  _dirPin;
    int  _enablePin;
    bool _enabled;

    float         _requestedRate;
    float         _currentRate;
    unsigned long _prevRampUs;
    unsigned long _prevStepUs;
    long          _positionSteps;
};
