#include "KY040.h"

// Full quadrature state table: index = (prev_state << 2) | cur_state
// Values: +1/-1 = valid single step, +2/-2 = skipped state (fast move),
//          0 = no movement or invalid/bounce transition.
//
// Standard Gray-code quadrature sequence (CW): 00 -> 01 -> 11 -> 10 -> 00
//                                              (CCW is the reverse)
static const int8_t enc_states[16] = {
     0,  // 00->00  no movement
     1,  // 00->01  CW  (half-step)
    -1,  // 00->10  CCW (half-step)
     2,  // 00->11  CW  skipped state (fast move, count 2)
    -1,  // 01->00  CCW (half-step)
     0,  // 01->01  no movement
    -2,  // 01->10  CCW skipped state (fast move, count -2)
     1,  // 01->11  CW  (half-step)
     1,  // 10->00  CW  (half-step)
    -2,  // 10->01  CCW skipped state (fast move, count -2)
     0,  // 10->10  no movement
    -1,  // 10->11  CCW (half-step)
     2,  // 11->00  CW  skipped state (fast move, count 2) -- NOTE: treat as bounce? set 0 if bouncy
    -1,  // 11->01  CCW (half-step)
     1,  // 11->10  CW  (half-step)
     0   // 11->11  no movement
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

long KY040::getRawCount() const {
  noInterrupts();
  long count = _position;
  interrupts();
  return count;
}

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

  // Read both pins simultaneously from the GPIO register to avoid
  // the latency and overhead of two separate digitalRead() calls.
  // This is critical for high-speed movement where pin states can
  // change between two sequential reads.
#if defined(ESP32)
  uint32_t reg = GPIO.in;   // single atomic 32-bit read of all GPIO 0-31 states
  uint8_t clk = (reg >> _clkPin) & 1;
  uint8_t dt  = (reg >> _dtPin)  & 1;
#else
  uint8_t clk = digitalRead(_clkPin);
  uint8_t dt  = digitalRead(_dtPin);
#endif

  // New 2-bit pin state: DT is the MSB, CLK is the LSB
  uint8_t pinstate = (dt << 1) | clk;

  // Shift previous state into upper 2 bits and append current state
  uint8_t newState = ((_lastPinState << 2) & 0x0F) | pinstate;

  _position   += enc_states[newState];
  _lastPinState = newState;
}

void IRAM_ATTR KY040::buttonIsr(void *arg) {
  KY040 *encoder = static_cast<KY040 *>(arg);
  encoder->handleButtonInterrupt();
}

void IRAM_ATTR KY040::handleButtonInterrupt() {
  unsigned long now = millis();
  if (now - _lastButtonPressTime > 1) {
    _buttonPressed = true;
    _lastButtonPressTime = now;
  }
}
