#include "KalmanFilter.h"

float KalmanFilter::update(float newAngle, float newRate, float dt) {
  // ── Predict ──────────────────────────────────────────────────────────────
  rate    = newRate - bias;
  angle  += dt * rate;

  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // ── Update (measurement fusion) ──────────────────────────────────────────
  float S    = P[0][0] + R_measure;
  float K[2] = { P[0][0] / S, P[1][0] / S };
  float y    = newAngle - angle;

  angle   += K[0] * y;
  bias    += K[1] * y;

  float P00 = P[0][0], P01 = P[0][1];
  P[0][0] -= K[0] * P00;
  P[0][1] -= K[0] * P01;
  P[1][0] -= K[1] * P00;
  P[1][1] -= K[1] * P01;

  return angle;
}
