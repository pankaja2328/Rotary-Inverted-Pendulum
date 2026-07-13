#ifndef STEPPER_FOLLOWER_H
#define STEPPER_FOLLOWER_H

#include <Arduino.h>

extern bool trackingEnabled;
extern long motorStepPosition;
extern int8_t currentDirection;
extern float currentStepRate;

extern float uprightSetpointDeg;
extern float pendulumAngleDeg;
extern float pendulumErrorDeg;

enum TrackingMode {
  MODE_NONE,
  MODE_HANGING,
  MODE_INVERTED
};

extern TrackingMode currentMode;

void initStepper();
void updateFollower();
void runStepper();
void zeroPosition();
void enableTracking(TrackingMode mode);
void disableTracking();

float wrapAngle180(float angle);

#endif // STEPPER_FOLLOWER_H
