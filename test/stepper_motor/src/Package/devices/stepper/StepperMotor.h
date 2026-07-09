#pragma once
#include <Arduino.h>
#include <AccelStepper.h>
#include "../../services/timer/TimerService.h"

/**
 * @file  StepperMotor.h
 * @brief Device driver wrapping AccelStepper for STEP/DIR drivers (e.g. A4988, DRV8825).
 *
 * Motion updates are driven by an esp_timer hardware callback (via TimerService)
 * instead of the main loop, giving deterministic, jitter-free step timing.
 */
class StepperMotor {
public:
  enum Mode {
    MODE_POSITION,
    MODE_SPEED,
    MODE_SWEEP
  };

  StepperMotor(int stepPin = 14, int dirPin = 12, int enPin = 13);

  /**
   * @brief Initialise driver, enable pin, and start the hardware timer.
   * @param updateInterval_us  Timer period in microseconds (default 200 us = 5 kHz).
   */
  void begin(uint32_t updateInterval_us = 200);
  void update();

  // Mode settings
  void setMode(Mode mode);
  Mode getMode() const { return _mode; }

  // Enable/Disable output
  void enable(bool enable);
  bool isEnabled() const { return _enabled; }

  // Position control (MODE_POSITION)
  void setTargetPosition(long steps);
  long getTargetPosition() const { return _targetPosition; }
  long getCurrentPosition() { return _stepper.currentPosition(); }

  // Speed control (MODE_SPEED)
  void setTargetSpeed(float stepsPerSec);
  float getTargetSpeed() const { return _targetSpeed; }
  float getCurrentSpeed() { return _stepper.speed(); }

  // Acceleration
  void setAcceleration(float stepsPerSecSq);
  float getAcceleration() const { return _acceleration; }

  // S-Curve parameters
  void setMinPulse(float minPulse);
  float getMinPulse() const { return _minPulse; }

  void setStartingPulse(float startingPulse);
  float getStartingPulse() const { return _startingPulse; }

  // Sweep limits (MODE_SWEEP)
  void setSweepLimits(long minVal, long maxVal);
  long getSweepMin() const { return _sweepMin; }
  long getSweepMax() const { return _sweepMax; }

  void stop();

private:
  int _stepPin;
  int _dirPin;
  int _enPin;
  bool _enabled;

  Mode _mode;
  float _targetSpeed;
  long _targetPosition;
  float _acceleration;

  float _minPulse;
  float _startingPulse;
  long _startPosition;

  long _sweepMin;
  long _sweepMax;
  bool _sweepingToMax;

  AccelStepper _stepper;
  TimerService _timer;

  /**
   * @brief Static trampoline passed to esp_timer.
   *        Recovers the StepperMotor instance from the void* arg and calls update().
   */
  static void _timerCallback(void* arg);
};
