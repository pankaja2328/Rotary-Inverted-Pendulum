#include "StepperFollower.h"
#include "Config.h"
#include "EncoderTracker.h"

bool trackingEnabled = false;   // start disabled - enable manually once pendulum is held upright
TrackingMode currentMode = MODE_NONE;
long motorStepPosition = 0;
float currentStepRate = 0.0;

int8_t currentDirection = 0;
unsigned long previousStepMicros = 0;
unsigned long previousPIDMicros = 0;

float integralError = 0.0;
float lastError = 0.0;

float uprightSetpointDeg = 0.0;
float pendulumAngleDeg = 0.0;
float pendulumErrorDeg = 0.0;

float wrapAngle180(float angle) {
  while (angle > 180.0)  angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

void initStepper() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(ENABLE_PIN, DRIVER_DISABLE);

  unsigned long now = micros();
  previousStepMicros = now;
  previousPIDMicros = now;
}

// Defines "wherever the pendulum is right now" as vertical (0 deg).
// Hold the pendulum upright by hand, then send 'Z'.
void zeroPosition() {
  resetEncoderCount();
  uprightSetpointDeg = 0.0;
  motorStepPosition = 0;
  currentStepRate = 0.0;
  currentDirection = 0.0;
  integralError = 0.0;
  lastError = 0.0;
  pendulumAngleDeg = 0.0;
  pendulumErrorDeg = 0.0;
  
  Serial.println("Upright setpoint calibrated to current pendulum position.");
}

void enableTracking(TrackingMode mode) {
  currentMode = mode;
  trackingEnabled = true;
  integralError = 0.0;
  lastError = 0.0;
  currentStepRate = 0.0;
  digitalWrite(ENABLE_PIN, DRIVER_ENABLE);
  Serial.print("Tracking Enabled. Mode: ");
  Serial.println(mode == MODE_INVERTED ? "INVERTED" : "HANGING");
}

void disableTracking() {
  trackingEnabled = false;
  currentMode = MODE_NONE;
  currentStepRate = 0.0;
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(ENABLE_PIN, DRIVER_DISABLE);
  Serial.println("Tracking Disabled.");
}

void updateFollower() {
  // Always read the pendulum angle, even when tracking is disabled,
  // so serial output reflects live sensor data for debugging.
  long currentEncoderCount = readEncoderCount();
  pendulumAngleDeg = ENCODER_DIRECTION_SIGN * (float)currentEncoderCount * 360.0 / ENCODER_COUNTS_PER_REV;
  pendulumErrorDeg = wrapAngle180(pendulumAngleDeg - uprightSetpointDeg);

  // Auto-zero if the pendulum is stuck in a detent (stationary for 500ms)
  static float lastStationaryAngle = -999.0;
  static unsigned long stationaryStartTime = millis();
  
  // 5.0 degrees threshold (since 1 encoder tick is 4.5 degrees)
  if (fabs(pendulumAngleDeg - lastStationaryAngle) > 5.0) {
    lastStationaryAngle = pendulumAngleDeg;
    stationaryStartTime = millis();
  } else if (millis() - stationaryStartTime > 500) {
    if (fabs(pendulumErrorDeg) > 0.1) {
      Serial.println("Pendulum stationary for 500ms. Auto-zeroing to stop motor.");
      zeroPosition();
      lastStationaryAngle = pendulumAngleDeg; // Update so we don't immediately detect a jump
    }
    stationaryStartTime = millis();
  }

  if (!trackingEnabled) return;

  unsigned long currentMicros = micros();
  // PID logic
  float dt = (float)(currentMicros - previousPIDMicros) / 1000000.0;
  previousPIDMicros = currentMicros;

  float error = pendulumErrorDeg;

  // Safety cutoff - pendulum has fallen too far, stop fighting it
  if (fabs(error) > ANGLE_FAIL_LIMIT_DEG) {
    disableTracking();
    Serial.println("SAFETY CUTOFF: angle limit exceeded. Tracking disabled.");
    return;
  }

  integralError += error * dt;

  // Integral Windup Protection
  if (fabs(error) > 10.0) {
    integralError = 0.0;
  } else {
    if (integralError > BALANCE_INTEGRAL_LIMIT) integralError = BALANCE_INTEGRAL_LIMIT;
    if (integralError < -BALANCE_INTEGRAL_LIMIT) integralError = -BALANCE_INTEGRAL_LIMIT;
  }

  float derivative = (error - lastError) / dt;
  lastError = error;

  float kp = (currentMode == MODE_INVERTED) ? INVERTED_KP : HANGING_KP;
  float ki = (currentMode == MODE_INVERTED) ? INVERTED_KI : HANGING_KI;
  float kd = (currentMode == MODE_INVERTED) ? INVERTED_KD : HANGING_KD;

  float pidOutput = (kp * error) + (ki * integralError) + (kd * derivative);

  // Add arm centering force to stop the arm from drifting/spinning continuously
  float motorAngleDeg = motorStepPosition * 360.0 / MOTOR_STEPS_PER_REV;
  float armCenteringForce = -1.5 * motorAngleDeg; 
  pidOutput += armCenteringForce;

  int8_t targetDir = (pidOutput >= 0) ? 1 : -1;
  bool directionPinLevel = (targetDir * MOTOR_DIRECTION_SIGN) > 0;
  if (currentDirection != targetDir) {
    digitalWrite(DIR_PIN, directionPinLevel ? HIGH : LOW);
    currentDirection = targetDir;
  }

  currentStepRate = fabs(pidOutput);
  if (currentStepRate > MAX_STEP_RATE) currentStepRate = MAX_STEP_RATE;
}

void runStepper() {
  if (!trackingEnabled || currentStepRate < MIN_STEP_RATE) {
    return;
  }

  unsigned long currentMicros = micros();
  unsigned long stepIntervalMicros = (unsigned long)(1000000.0 / currentStepRate);

  if ((unsigned long)(currentMicros - previousStepMicros) >= stepIntervalMicros) {
    previousStepMicros += stepIntervalMicros;

    if ((unsigned long)(currentMicros - previousStepMicros) > stepIntervalMicros * 4UL) {
      previousStepMicros = currentMicros;
    }

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_WIDTH_US);
    digitalWrite(STEP_PIN, LOW);

    motorStepPosition += currentDirection;
  }
}
