#pragma once
#include <Arduino.h>
#include "../../services/timer/TimerService.h"

enum class StepperMode {
  IDLE = 0,
  ACCEL = 1,
  CONSTANT = 2,
  DECEL = 3,
  STOPPING = 4
};

class StepperMotor {
public:
  StepperMotor(int stepPin, int dirPin, int enPin);
  ~StepperMotor();

  void begin();
  void setEnable(bool enable);
  bool isEnabled() const;
  
  StepperMode getMode() const;
  long getCurrentPosition() const;
  float getCurrentSpeed() const;

  // Move command similar to reference code (intervals in 20uS ticks)
  void moveRelative(uint32_t steps, bool direction, uint32_t min_interval_ticks, uint32_t start_interval_ticks, uint32_t accel_steps);
  void stop();

private:
  int _stepPin;
  int _dirPin;
  int _enPin;
  bool _enabled;

  // Controller state variables (volatile since they are modified in ISR/timer task)
  volatile StepperMode _state;
  volatile bool _direction;
  volatile uint32_t _targetSteps;
  volatile uint32_t _stepCount;
  volatile long _currentPosition;

  volatile uint32_t _c0;
  volatile uint32_t _c_min;
  volatile int32_t _n;
  volatile uint32_t _d;
  volatile uint32_t _di;
  volatile uint32_t _stepDelay;
  volatile int32_t _accelCount;
  volatile uint32_t _acceleration;
  volatile bool _pulseActive;

  TimerService _timerService;

  static void timerCallback(void* arg);
  void handleTimerInterrupt();
};
