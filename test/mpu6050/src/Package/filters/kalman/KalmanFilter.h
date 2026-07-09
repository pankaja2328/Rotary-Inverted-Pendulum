#pragma once

/**
 * @file  KalmanFilter.h
 * @brief 2-D discrete Kalman Filter for angle estimation.
 *
 * State vector: [ angle, gyro_bias ]
 * Measurement : accelerometer-derived angle
 * Input       : gyroscope rate (deg/s)
 */
class KalmanFilter {
public:
  // ── Tunable noise parameters ─────────────────────────────────────────────
  float Q_angle   = 0.001f;  // Process noise – angle    (lower → trust gyro more)
  float Q_bias    = 0.003f;  // Process noise – gyro bias
  float R_measure = 0.03f;   // Measurement noise (lower → trust accel more)

  // ── State ─────────────────────────────────────────────────────────────────
  float angle = 0.0f;        // Filtered angle estimate (degrees)
  float bias  = 0.0f;        // Gyro bias estimate     (deg/s)
  float rate  = 0.0f;        // Unbiased rate

  // Error covariance matrix
  float P[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};

  /**
   * @brief Run one filter iteration.
   * @param newAngle  Accelerometer-derived angle (degrees)
   * @param newRate   Gyroscope rate              (deg/s)
   * @param dt        Time delta since last call  (seconds)
   * @return          Filtered angle              (degrees)
   */
  float update(float newAngle, float newRate, float dt);
};
