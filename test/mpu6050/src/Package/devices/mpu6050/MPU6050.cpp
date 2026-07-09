#include "MPU6050.h"
#include <Wire.h>
#include <math.h>

MPU6050::MPU6050(int sdaPin, int sclPin)
  : _sdaPin(sdaPin), _sclPin(sclPin), _ax(0), _ay(0), _az(0), _temp(0), _gx(0), _gy(0), _gz(0) {}

bool MPU6050::begin() {
  Wire.begin(_sdaPin, _sclPin);

  Wire.beginTransmission(ADDR);
  byte error = Wire.endTransmission();
  if (error != 0) {
    return false;
  }

  // Wake up MPU6050
  Wire.beginTransmission(ADDR);
  Wire.write(REG_PWR_MGMT);
  Wire.write(0);
  Wire.endTransmission(true);

  // Initial read & seed Kalman
  _seedKalman();
  return true;
}

void MPU6050::_readRaw() {
  Wire.beginTransmission(ADDR);
  Wire.write(REG_ACCEL_H);
  Wire.endTransmission(false);
  Wire.requestFrom((int)ADDR, (int)14, (int)1);

  _ax   = (Wire.read() << 8) | Wire.read();
  _ay   = (Wire.read() << 8) | Wire.read();
  _az   = (Wire.read() << 8) | Wire.read();
  _temp = (Wire.read() << 8) | Wire.read();
  _gx   = (Wire.read() << 8) | Wire.read();
  _gy   = (Wire.read() << 8) | Wire.read();
  _gz   = (Wire.read() << 8) | Wire.read();
}

void MPU6050::_seedKalman() {
  Wire.beginTransmission(ADDR);
  Wire.write(REG_ACCEL_H);
  Wire.endTransmission(false);
  Wire.requestFrom((int)ADDR, 6, 1);
  int16_t axRaw = (Wire.read() << 8) | Wire.read();
  int16_t ayRaw = (Wire.read() << 8) | Wire.read();
  int16_t azRaw = (Wire.read() << 8) | Wire.read();

  float ax = axRaw / 16384.0f;
  float ay = ayRaw / 16384.0f;
  float az = azRaw / 16384.0f;

  _pitch = atan2(ay, sqrtf(ax*ax + az*az)) * 180.0f / M_PI;
  _roll  = atan2(-ax, az) * 180.0f / M_PI;

  _kPitch.angle = _pitch;
  _kRoll.angle  = _roll;
  _yaw = 0.0f;
  _lastMicros = micros();
}

void MPU6050::update() {
  unsigned long now = micros();
  float dt = (now - _lastMicros) / 1000000.0f;
  if (dt < 0.010f) return; // 100 Hz cap
  _lastMicros = now;

  _readRaw();

  // Convert raw values to physical units
  float ax = _ax / 16384.0f;
  float ay = _ay / 16384.0f;
  float az = _az / 16384.0f;
  float gx = _gx / 131.0f;
  float gy = _gy / 131.0f;
  float gz = _gz / 131.0f;

  // Calculate accelerometer pitch and roll
  float pitchAcc = atan2(ay, sqrtf(ax*ax + az*az)) * 180.0f / M_PI;
  float rollAcc  = atan2(-ax, az) * 180.0f / M_PI;

  // Apply Kalman Filters
  _pitch = _kPitch.update(pitchAcc, gy, dt);
  _roll  = _kRoll.update(rollAcc, gx, dt);

  // Integrate yaw (gyroscope only)
  _yaw += gz * dt;
  if (_yaw > 180.0f)  _yaw -= 360.0f;
  if (_yaw < -180.0f) _yaw += 360.0f;
}
