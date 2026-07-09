#pragma once
#include <Arduino.h>
#include "../../filters/kalman/KalmanFilter.h"

/**
 * @file  MPU6050.h
 * @brief Driver for the MPU6050 IMU sensor.
 *
 * Reads raw accelerometer + gyroscope data over I2C and internally
 * fuses them through Kalman Filters to produce stable Pitch and Roll
 * angles, plus a gyro-integrated Yaw angle.
 */
class MPU6050 {
public:
  // ── Construction / Configuration ─────────────────────────────────────────
  MPU6050(int sdaPin = 21, int sclPin = 22);

  /**
   * @brief Initialise I2C, wake the sensor, and seed Kalman filters.
   * @return true on success, false if sensor not found.
   */
  bool begin();

  /**
   * @brief Read sensor and update filtered angles.
   *        Call this once per loop iteration.
   */
  void update();

  // ── Angle accessors ───────────────────────────────────────────────────────
  float getPitch() const { return _pitch; }
  float getRoll()  const { return _roll;  }
  float getYaw()   const { return _yaw;   }

private:
  // I2C pins
  int _sdaPin, _sclPin;

  // MPU6050 register addresses
  static const uint8_t ADDR         = 0x68;
  static const uint8_t REG_PWR_MGMT = 0x6B;
  static const uint8_t REG_ACCEL_H  = 0x3B;

  // Raw register values
  int16_t _ax, _ay, _az, _temp, _gx, _gy, _gz;

  // Filtered angles (degrees)
  float _pitch = 0.0f;
  float _roll  = 0.0f;
  float _yaw   = 0.0f;

  // Kalman filters for pitch and roll
  KalmanFilter _kPitch;
  KalmanFilter _kRoll;

  // Timing
  unsigned long _lastMicros = 0;

  // Internal helpers
  void _readRaw();
  void _seedKalman();
};
