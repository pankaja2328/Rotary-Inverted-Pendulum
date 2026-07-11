#include "KY040.h"

static const int8_t enc_states[] = {
    0,  // 00 -> 00 (no movement)
    1,  // 00 -> 01 (CW)
    -1, // 00 -> 10 (CCW)
    0,  // 00 -> 11 (invalid)
    -1, // 01 -> 00 (CCW)
    0,  // 01 -> 01 (no movement)
    0,  // 01 -> 10 (invalid)
    1,  // 01 -> 11 (CW)
    1,  // 10 -> 00 (CW)
    0,  // 10 -> 01 (invalid)
    0,  // 10 -> 10 (no movement)
    -1, // 10 -> 11 (CCW)
    0,  // 11 -> 00 (invalid)
    -1, // 11 -> 01 (CCW)
    1,  // 11 -> 10 (CW)
    0   // 11 -> 11 (no movement)
};

KY040::KY040(int clkPin, int dtPin, int swPin)
    : _clkPin(clkPin), _dtPin(dtPin), _swPin(swPin), _position(0),
      _lastPosition(0), _lastPinState(0), _buttonPressed(false),
      _lastButtonPressTime(0), _interruptCount(0) {}

void KY040::begin() {
  pinMode(_clkPin, INPUT_PULLUP);
  pinMode(_dtPin, INPUT_PULLUP);
  if (_swPin >= 0) {
    pinMode(_swPin, INPUT_PULLUP);
  }

  // Read initial state
  _lastPinState = (digitalRead(_dtPin) << 1) | digitalRead(_clkPin);

  // Attach interrupts for encoder pins (CLK and DT) on state change
  attachInterruptArg(digitalPinToInterrupt(_clkPin), isr, this, CHANGE);
  attachInterruptArg(digitalPinToInterrupt(_dtPin), isr, this, CHANGE);

  // Attach interrupt for switch button (SW) on falling edge (pressed state)
  if (_swPin >= 0) {
    attachInterruptArg(digitalPinToInterrupt(_swPin), buttonIsr, this, FALLING);
  }
}

long KY040::getPosition() const { return _position; }

unsigned long KY040::getInterruptCount() const { return _interruptCount; }

long KY040::getDelta() {
  long currentPos = getPosition();
  long delta = currentPos - _lastPosition;
  _lastPosition = currentPos;
  return delta;
}

void KY040::setPosition(long position) {
  _position = position;
  _lastPosition = position;
}

bool KY040::isButtonPressed() {
  if (_swPin < 0)
    return false;

  if (_buttonPressed) {
    _buttonPressed = false;
    return true;
  }
  return false;
}

void IRAM_ATTR KY040::isr(void *arg) {
  KY040 *encoder = static_cast<KY040 *>(arg);
  encoder->handleInterrupt();
}

void IRAM_ATTR KY040::handleInterrupt() {
  _interruptCount++;
  uint8_t pinstate = (digitalRead(_dtPin) << 1) | digitalRead(_clkPin);

  _lastPinState = ((_lastPinState << 2) & 0x0F) | pinstate;

  _position += enc_states[_lastPinState];
}

void IRAM_ATTR KY040::buttonIsr(void *arg) {
  KY040 *encoder = static_cast<KY040 *>(arg);
  encoder->handleButtonInterrupt();
}

void IRAM_ATTR KY040::handleButtonInterrupt() {
  unsigned long now = millis();
  if (now - _lastButtonPressTime > 50) {
    _buttonPressed = true;
    _lastButtonPressTime = now;
  }
}
