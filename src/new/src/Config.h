#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =======================================================
// PIN CONFIGURATION (DRV8825 saha KY-040 Encoder)
// =======================================================
const uint8_t STEP_PIN = 15;  // DRV8825 STEP pin eka GPIO 26 ta
const uint8_t DIR_PIN = 2;    // DRV8825 DIR pin eka GPIO 27 ta
const uint8_t ENABLE_PIN = 4; // DRV8825 EN pin eka GPIO 25 ta

const uint8_t ENCODER_A_PIN = 18; // KY-040 CLK pin eka GPIO 18 ta
const uint8_t ENCODER_B_PIN = 19; // KY-040 DT pin eka GPIO 19 ta

// DRV8825 wada karanne Active LOW kramayatai (LOW = Driver ON)
const uint8_t DRIVER_ENABLE = LOW;
const uint8_t DRIVER_DISABLE = HIGH;

// =======================================================
// MOTOR SETTINGS (32 Microstepping)
// =======================================================
const float MOTOR_FULL_STEPS_PER_REV = 200.0;
const float MICROSTEPS =
    32.0; // ⚠️ M0, M1, M2 thunama 3.3V nisa 32.0 karana ladi
const float GEAR_RATIO = 1.0;

const float MOTOR_STEPS_PER_REV =
    MOTOR_FULL_STEPS_PER_REV * MICROSTEPS * GEAR_RATIO;

// ⚠️ Wadagath: Pendulum balance wenne nathuwa thawa vegath watenna balanawa nam
// meka -1 karanna
const int MOTOR_DIRECTION_SIGN = -1;

// =======================================================
// ENCODER SETTINGS (KY-040)
// =======================================================
const float ENCODER_COUNTS_PER_REV = 80.0;
const float ENCODER_DIRECTION_SIGN = 1.0;

// =======================================================
// BALANCE PID TUNING CONSTANTS
// =======================================================
extern float HANGING_KP;
extern float HANGING_KI;
extern float HANGING_KD;

extern float INVERTED_KP;
extern float INVERTED_KI;
extern float INVERTED_KD;

const float BALANCE_INTEGRAL_LIMIT = 200.0;
const float ANGLE_FAIL_LIMIT_DEG = 45.0;

// =======================================================
// MOTION LIMITS
// =======================================================
const float MAX_STEP_RATE = 4000.0;
const float MIN_STEP_RATE = 20.0;
const unsigned int STEP_PULSE_WIDTH_US = 3;

const unsigned long PRINT_INTERVAL_MS = 100UL;

#endif // CONFIG_H
