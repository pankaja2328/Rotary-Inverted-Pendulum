#include "EncoderTracker.h"
#include "Config.h"

volatile long encoderCount = 0;
volatile uint8_t previousEncoderState = 0;
volatile unsigned long lastEncoderInterruptMicros = 0;

const int8_t QUADRATURE_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

inline uint8_t readEncoderPins() {
  uint8_t a = digitalRead(ENCODER_A_PIN);
  uint8_t b = digitalRead(ENCODER_B_PIN);
  return (a << 1) | b;
}

void IRAM_ATTR encoderISR() {
  unsigned long now = micros();
  if ((now - lastEncoderInterruptMicros) < 150) { // 150 microseconds debounce filter
    return;
  }
  lastEncoderInterruptMicros = now;

  uint8_t newEncoderState = readEncoderPins();
  uint8_t transition = (previousEncoderState << 2) | newEncoderState;
  encoderCount += QUADRATURE_TABLE[transition];
  previousEncoderState = newEncoderState;
}

void initEncoder() {
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  previousEncoderState = readEncoderPins();
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoderISR, CHANGE);
}

long readEncoderCount() {
  noInterrupts();
  long countCopy = encoderCount;
  interrupts();
  return countCopy;
}

void resetEncoderCount() {
  noInterrupts();
  encoderCount = 0;
  lastEncoderInterruptMicros = micros();
  previousEncoderState = readEncoderPins();
  interrupts();
}
