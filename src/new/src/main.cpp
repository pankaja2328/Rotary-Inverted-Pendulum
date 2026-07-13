#include <Arduino.h>
#include "Config.h"
#include "EncoderTracker.h"
#include "StepperFollower.h"
#include "SerialInterface.h"

unsigned long previousPrintMillis = 0;

void setup() {
  Serial.begin(115200);

  initEncoder();
  initStepper();

  previousPrintMillis = millis();

  Serial.println();
  Serial.println("ESP32 Rotary Inverted Pendulum - Balance Controller");
  Serial.println("STEP = GPIO25 | DIR = GPIO26 | ENABLE = GPIO27");
  Serial.println("Encoder A = GPIO18 | Encoder B = GPIO19");
  Serial.println("Driver enabled by GPIO27 driven LOW at boot.");
  Serial.println("Tracking starts DISABLED. Hold pendulum upright, send 'Z', then 'E'.");

  printHelp();
}

void loop() {
  processSerialCommands();

  updateFollower();    // Runs balance PID to calculate required step rate
  runStepper();         // Generates step pulses based on PID calculation

  unsigned long currentMillis = millis();
  if ((unsigned long)(currentMillis - previousPrintMillis) >= PRINT_INTERVAL_MS) {
    previousPrintMillis = currentMillis;
    printData();
  }
}
