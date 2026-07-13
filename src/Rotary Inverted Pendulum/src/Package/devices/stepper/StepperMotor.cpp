#include "StepperMotor.h"

StepperMotor::StepperMotor(int stepPin, int dirPin, int enPin)
  : _stepPin(stepPin), _dirPin(dirPin), _enPin(enPin), _enabled(false),
    _state(StepperMode::IDLE), _direction(false), _targetSteps(0), _stepCount(0),
    _currentPosition(0), _c0(0), _c_min(0), _n(0), _d(0), _di(0), _stepDelay(0),
    _accelCount(0), _acceleration(0), _pulseActive(false) {}

StepperMotor::~StepperMotor() {
  _timerService.stop();
}

void StepperMotor::begin() {
  pinMode(_stepPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  pinMode(_enPin, OUTPUT);
  
  digitalWrite(_stepPin, LOW);
  digitalWrite(_dirPin, LOW);
  digitalWrite(_enPin, HIGH); // Coils initially disabled (active low)
  
  // Start the 20 microsecond periodic timer
  _timerService.begin(timerCallback, this, 20, "StepperTimer");
}

void StepperMotor::setEnable(bool enable) {
  _enabled = enable;
  digitalWrite(_enPin, enable ? LOW : HIGH); // Active low enable pin
}

bool StepperMotor::isEnabled() const {
  return _enabled;
}

StepperMode StepperMotor::getMode() const {
  return _state;
}

long StepperMotor::getCurrentPosition() const {
  return _currentPosition;
}

float StepperMotor::getCurrentSpeed() const {
  if (_state == StepperMode::IDLE || _stepDelay == 0) {
    return 0.0f;
  }
  return 1000000.0f / (_stepDelay * 20.0f);
}

void StepperMotor::moveRelative(uint32_t steps, bool direction, uint32_t min_interval_ticks, uint32_t start_interval_ticks, uint32_t accel_steps) {
  if (steps == 0 || min_interval_ticks == 0) {
    return;
  }
  
  // Wait/stop if busy
  if (_state != StepperMode::IDLE) {
    stop();
  }

  if (start_interval_ticks < min_interval_ticks) {
    start_interval_ticks = min_interval_ticks;
  }
  if (accel_steps == 0) {
    start_interval_ticks = min_interval_ticks;
  }

  _targetSteps = steps;
  _stepCount = 0;
  _direction = direction;
  _c0 = start_interval_ticks;
  _c_min = min_interval_ticks;
  _d = _c0;
  _di = 0;
  _n = 0;

  if (accel_steps == 0 || start_interval_ticks <= min_interval_ticks) {
    _accelCount = 0;
    _d = _c_min;
    _acceleration = 0;
  } else {
    _accelCount = accel_steps;
    if (_accelCount * 2 > (int32_t)steps) {
      _accelCount = steps / 2;
    }
    if (_accelCount == 0) {
      _d = _c_min;
      _acceleration = 0;
    } else {
      uint32_t v_start = 1048576 / _c0;
      uint32_t v_max = 1048576 / _c_min;
      _acceleration = (v_max - v_start) / _accelCount;
    }
  }

  // Set direction pin
  digitalWrite(_dirPin, _direction ? HIGH : LOW);
  
  // Enable coils if disabled
  setEnable(true);

  _pulseActive = false;
  _stepDelay = _d;

  if (accel_steps == 0 || start_interval_ticks <= min_interval_ticks || _accelCount == 0) {
    _state = StepperMode::CONSTANT;
  } else {
    _state = StepperMode::ACCEL;
  }
}

void StepperMotor::stop() {
  _state = StepperMode::IDLE;
  digitalWrite(_stepPin, LOW);
  _pulseActive = false;
}

void StepperMotor::timerCallback(void* arg) {
  static_cast<StepperMotor*>(arg)->handleTimerInterrupt();
}

void StepperMotor::handleTimerInterrupt() {
  if (_state == StepperMode::IDLE) {
    return;
  }
  
  _di++;
  if (!_pulseActive) {
    if (_di >= (_stepDelay / 2)) {
      digitalWrite(_stepPin, HIGH);
      _pulseActive = true;
    }
  } else {
    if (_di >= _stepDelay) {
      digitalWrite(_stepPin, LOW);
      _pulseActive = false;
      _di = 0;
      _stepCount++;

      // Update position
      if (_direction) {
        _currentPosition++;
      } else {
        _currentPosition--;
      }

      if (_stepCount >= _targetSteps) {
        _state = StepperMode::IDLE;
        setEnable(false);
        return;
      }

      switch (_state) {
      case StepperMode::ACCEL:
        _n++;
        if (_n >= _accelCount) {
          _d = _c_min;
          _state = StepperMode::CONSTANT;
        } else {
          uint32_t v_start = 1048576 / _c0;
          uint32_t v_n = v_start + (_n * _acceleration);
          _d = (v_n == 0) ? _c_min : (1048576 / v_n);
        }
        if (_state == StepperMode::CONSTANT &&
            _stepCount >= (_targetSteps - _accelCount)) {
          _state = StepperMode::DECEL;
          _n = _accelCount;
        }
        break;

      case StepperMode::CONSTANT:
        if (_stepCount >= (_targetSteps - _accelCount)) {
          _state = StepperMode::DECEL;
          _n = _accelCount;
        }
        break;

      case StepperMode::DECEL:
        if (_n > 0)
          _n--;
        if (_n == 0) {
          _d = _c0;
        } else {
          uint32_t v_start = 1048576 / _c0;
          uint32_t v_n = v_start + (_n * _acceleration);
          _d = (v_n == 0) ? _c0 : (1048576 / v_n);
        }
        break;

      case StepperMode::STOPPING:
        _state = StepperMode::IDLE;
        setEnable(false);
        return;

      default:
        break;
      }
      
      _stepDelay = _d;
      if (_stepDelay == 0) {
        _stepDelay = 1;
      }
    }
  }
}
