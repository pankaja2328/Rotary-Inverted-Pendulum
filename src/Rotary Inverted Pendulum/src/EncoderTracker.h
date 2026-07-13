#ifndef ENCODER_TRACKER_H
#define ENCODER_TRACKER_H

#include <Arduino.h>

void initEncoder();
uint8_t readEncoderPins();
void IRAM_ATTR encoderISR();
long readEncoderCount();
void resetEncoderCount();

#endif // ENCODER_TRACKER_H
