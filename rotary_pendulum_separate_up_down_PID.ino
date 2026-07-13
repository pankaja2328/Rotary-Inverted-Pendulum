/*
  ============================================================================
  ROTARY INVERTED PENDULUM
  ESP32 + DRV8825 + NEMA17 + KY-040
  Swing-up + automatic upright capture + upright PID + down PID
  ============================================================================

  DRV8825:
  STEP   -> ESP32 GPIO26
  DIR    -> ESP32 GPIO27
  ENABLE -> ESP32 GPIO25
  RESET and SLEEP connected together -> ESP32 3V3
  M0, M1, M2 -> GND (full-step mode)

  KY-040:
  CLK -> ESP32 GPIO32
  DT  -> ESP32 GPIO33
  +   -> ESP32 3V3
  GND -> ESP32 GND
  SW  -> Not connected

  Commands:
  C = Start four-position encoder calibration
  N = Save the next calibration position
  Z = Set the current pendulum position as DOWN = 0 degrees
  H = Set the current rotary-arm position as centre
  S = Start swing-up; automatically catch and balance at UP
  U = Start upright capture manually when pendulum is near UP
  D = Start downward PID stabilization
  P = Print measurements
  X = Emergency stop
  K = Reverse swing-up direction
  V = Reverse upright capture/PID direction
  B = Reverse downward PID direction
  R = Reset controller
  M = Print command menu

  IMPORTANT:
  - The KY-040 is low-resolution and has mechanical detents.
  - The arm position is estimated from commanded steps only.
  - Keep X ready as an emergency stop.
*/

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

const int STEP_PIN   = 26;
const int DIR_PIN    = 27;
const int ENABLE_PIN = 25;

const int ENCODER_CLK_PIN = 32;
const int ENCODER_DT_PIN  = 33;

// ============================================================================
// MOTOR SETTINGS
// ============================================================================

const int MOTOR_STEPS_PER_REVOLUTION = 200;
const float MOTOR_STEP_ANGLE_DEG = 1.8;
const unsigned long STEP_PULSE_US = 5;

// Swing-up speed: 900 steps/s = approximately 270 RPM in full-step mode.
const float MAX_SWING_SPEED_STEPS_S = 700.0;

// High speed is available for catching a rotating pendulum near upright.
const float MAX_CATCH_SPEED_STEPS_S = 1000.0;

// Upright holding speed limit.
const float MAX_UP_BALANCE_SPEED_STEPS_S = 850.0;

// Downward stabilization uses a gentler speed.
const float MAX_DOWN_BALANCE_SPEED_STEPS_S = 350.0;

// Acceleration limit for the stepper speed command.
const float MOTOR_ACCELERATION_STEPS_S2 = 7000.0;

// ============================================================================
// MOTOR ANGLE LIMITS — PRESERVED
// ============================================================================

/*
  Hard limit:
  75 steps x 1.8 degrees = 135 degrees from centre.
  Total permitted arm range = 270 degrees.
*/
const long HARD_ARM_LIMIT_STEPS = 75;

/*
  Soft PID limit:
  Normal PID action is prevented from driving farther outward after +/-65 steps.
  The remaining 10 steps provide a final safety margin.
*/
const long SOFT_ARM_LIMIT_STEPS = 65;

/*
  DOWN-STABLE motor-arm limit only:

  The motor is in full-step mode, so one step = 1.8 degrees.
  An exact 120.0 degrees is not possible in full-step mode.

  66 steps x 1.8 degrees = 118.8 degrees,
  which is the closest safe value below 120 degrees.

  This limit applies only in DOWN_STABILIZING mode.
  All UP, CATCHING and SWING-UP limits remain unchanged.
*/
const long DOWN_ARM_LIMIT_STEPS = 66;

/*
  Swing-up target:
  Maximum +/-45 steps = +/-81 degrees from centre.
  Maximum side-to-side swing-up movement = 162 degrees.
*/
const int INITIAL_SWING_AMPLITUDE_STEPS = 12;
const int MAX_SWING_AMPLITUDE_STEPS = 45;
const int SWING_AMPLITUDE_INCREMENT_STEPS = 3;

// ============================================================================
// SWING-UP SETTINGS
// ============================================================================

const float BOTTOM_WINDOW_DEG = 30.0;
const float BOTTOM_REARM_DEG  = 45.0;

const unsigned long MINIMUM_SHIFT_INTERVAL_MS = 120;
const unsigned long STARTUP_SHIFT_INTERVAL_MS = 500;

// ============================================================================
// UPRIGHT CAPTURE SETTINGS
// ============================================================================

/*
  The old code could fail to enter PID because the KY-040 velocity estimate
  produced large spikes. This version enters a separate CATCHING state as soon
  as the pendulum enters the upright region.
*/
const float UP_CAPTURE_ENTRY_DEG = 36.0;
const float UP_CAPTURE_ABORT_DEG = 72.0;

// Conditions required before changing from CATCHING to normal UP_BALANCING.
const float UP_CAPTURE_STABLE_ERROR_DEG = 12.0;
const float UP_CAPTURE_STABLE_SPEED_DEG_S = 180.0;
const unsigned long UP_CAPTURE_STABLE_TIME_MS = 80;
const unsigned long UP_CAPTURE_TIMEOUT_MS = 1800;

// Aggressive PD values used only during the initial capture.
float CATCH_KP = 9.0;
float CATCH_KD = 0.18;
const float CATCH_ARM_CENTRE_GAIN = 0.5;

// ============================================================================
// UPRIGHT PID SETTINGS
// ============================================================================

const float UP_FALL_WINDOW_DEG = 48.0;

float UP_KP = 6.0;
float UP_KI = 0.0;
float UP_KD = 0.30;

const float UP_PID_INTEGRAL_LIMIT = 25.0;
const float UP_ARM_CENTRE_GAIN = 1.2;

// Half of one 6-degree KY-040 count. This reduces unnecessary chattering.
const float UP_DEADBAND_DEG = 3.0;

// ============================================================================
// DOWNWARD PID SETTINGS
// ============================================================================

const float DOWN_START_WINDOW_DEG = 30.0;
const float DOWN_EXIT_WINDOW_DEG = 60.0;

float DOWN_KP = 1.2;
float DOWN_KI = 0.0;
float DOWN_KD = 0.10;

const float DOWN_PID_INTEGRAL_LIMIT = 20.0;
const float DOWN_ARM_CENTRE_GAIN = 1.5;
const float DOWN_DEADBAND_DEG = 3.0;

// Direction signs. Change using K, V and B without editing the code.
int swingPolarity = 1;
int upBalancePolarity = 1;
int downBalancePolarity = 1;

// ============================================================================
// ENCODER SETTINGS AND VARIABLES
// ============================================================================

float encoderCountsPerRevolution = 60.0;
int encoderDirection = 1;

volatile long encoderCount = 0;
volatile uint8_t previousEncoderState = 0;

long encoderZeroCount = 0;
long previousEncoderCount = 0;
long previousVelocityCount = 0;

bool encoderZeroSet = false;

float pendulumAngleDeg = 0.0;
float pendulumVelocityDegS = 0.0;

int lastPendulumDirection = 0;

unsigned long previousEncoderUpdateUs = 0;
unsigned long previousVelocityUpdateUs = 0;

// Angle is updated quickly, but velocity uses a longer window to reduce spikes.
const unsigned long ANGLE_UPDATE_INTERVAL_US = 5000;
const unsigned long VELOCITY_UPDATE_INTERVAL_US = 25000;

// ============================================================================
// CALIBRATION VARIABLES
// ============================================================================

bool calibrationActive = false;
int calibrationIndex = 0;
long calibrationCounts[5];

const char *calibrationNames[5] =
{
  "DOWN",
  "RIGHT HORIZONTAL",
  "UP",
  "LEFT HORIZONTAL",
  "DOWN AGAIN"
};

// ============================================================================
// MOTOR VARIABLES
// ============================================================================

long armPositionSteps = 0;
long swingTargetSteps = 0;

float requestedMotorRateStepsS = 0.0;
float currentMotorRateStepsS = 0.0;

unsigned long previousMotorRampUs = 0;
unsigned long previousStepUs = 0;

// ============================================================================
// CONTROLLER VARIABLES
// ============================================================================

enum ControllerState
{
  WAITING_FOR_ZERO,
  READY,
  DOWN_STABILIZING,
  SWING_UP,
  CATCHING,
  UP_BALANCING,
  STOPPED
};

ControllerState controllerState = WAITING_FOR_ZERO;

bool armCentreSet = false;
bool bottomCrossingArmed = false;

// True when CATCHING/UP_BALANCING was entered from S swing-up mode.
bool autoReturnToSwingUp = false;

int swingTargetSide = 0;
int swingAmplitudeSteps = INITIAL_SWING_AMPLITUDE_STEPS;

float pidIntegral = 0.0;

unsigned long lastSwingShiftMs = 0;
unsigned long previousControllerUpdateUs = 0;
unsigned long previousPrintMs = 0;

unsigned long captureStartMs = 0;
unsigned long captureStableStartMs = 0;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

float clampFloat(float value, float minimumValue, float maximumValue)
{
  if (value < minimumValue) return minimumValue;
  if (value > maximumValue) return maximumValue;
  return value;
}

long clampLong(long value, long minimumValue, long maximumValue)
{
  if (value < minimumValue) return minimumValue;
  if (value > maximumValue) return maximumValue;
  return value;
}

float wrapAngleTo180(float angle)
{
  while (angle > 180.0) angle -= 360.0;
  while (angle <= -180.0) angle += 360.0;
  return angle;
}

// Conventional error: reference - measurement.
float calculateUprightError(float angle)
{
  return wrapAngleTo180(180.0 - angle);
}

float calculateDownwardError(float angle)
{
  return wrapAngleTo180(0.0 - angle);
}

bool controllerIsActive()
{
  return controllerState == DOWN_STABILIZING ||
         controllerState == SWING_UP ||
         controllerState == CATCHING ||
         controllerState == UP_BALANCING;
}

void clearMotorCommand()
{
  requestedMotorRateStepsS = 0.0;
  currentMotorRateStepsS = 0.0;
  previousStepUs = micros();
  previousMotorRampUs = micros();
}

void resetVelocityEstimator()
{
  long count = encoderCount;
  previousEncoderCount = count;
  previousVelocityCount = count;
  previousEncoderUpdateUs = micros();
  previousVelocityUpdateUs = previousEncoderUpdateUs;
  pendulumVelocityDegS = 0.0;
}

// ============================================================================
// ENCODER INTERRUPT
// ============================================================================

void IRAM_ATTR encoderInterrupt()
{
  uint8_t clkState = digitalRead(ENCODER_CLK_PIN);
  uint8_t dtState  = digitalRead(ENCODER_DT_PIN);

  uint8_t currentState = (clkState << 1) | dtState;
  uint8_t transition = (previousEncoderState << 2) | currentState;

  if (transition == 0b0001 ||
      transition == 0b0111 ||
      transition == 0b1110 ||
      transition == 0b1000)
  {
    encoderCount++;
  }

  if (transition == 0b0010 ||
      transition == 0b1011 ||
      transition == 0b1101 ||
      transition == 0b0100)
  {
    encoderCount--;
  }

  previousEncoderState = currentState;
}

long readEncoderSafely()
{
  noInterrupts();
  long count = encoderCount;
  interrupts();
  return count;
}

// ============================================================================
// ENCODER MEASUREMENT
// ============================================================================

void updateEncoderMeasurements()
{
  unsigned long currentTimeUs = micros();

  if (previousEncoderUpdateUs == 0)
  {
    long currentCount = readEncoderSafely();
    previousEncoderUpdateUs = currentTimeUs;
    previousVelocityUpdateUs = currentTimeUs;
    previousEncoderCount = currentCount;
    previousVelocityCount = currentCount;
    return;
  }

  if ((unsigned long)(currentTimeUs - previousEncoderUpdateUs) >=
      ANGLE_UPDATE_INTERVAL_US)
  {
    long currentCount = readEncoderSafely();

    long relativeCount =
        (currentCount - encoderZeroCount) * encoderDirection;

    float unwrappedAngle =
        ((float)relativeCount * 360.0) /
        encoderCountsPerRevolution;

    pendulumAngleDeg = wrapAngleTo180(unwrappedAngle);

    long countDifference =
        (currentCount - previousEncoderCount) * encoderDirection;

    if (countDifference > 0)
    {
      lastPendulumDirection = 1;
    }
    else if (countDifference < 0)
    {
      lastPendulumDirection = -1;
    }

    previousEncoderCount = currentCount;
    previousEncoderUpdateUs = currentTimeUs;
  }

  if ((unsigned long)(currentTimeUs - previousVelocityUpdateUs) >=
      VELOCITY_UPDATE_INTERVAL_US)
  {
    long currentCount = readEncoderSafely();
    unsigned long elapsedUs = currentTimeUs - previousVelocityUpdateUs;

    long countDifference =
        (currentCount - previousVelocityCount) * encoderDirection;

    float elapsedSeconds = elapsedUs / 1000000.0;

    float measuredVelocity =
        ((float)countDifference * 360.0) /
        (encoderCountsPerRevolution * elapsedSeconds);

    // Longer measurement window + filtering reduces KY-040 velocity spikes.
    pendulumVelocityDegS =
        0.65 * pendulumVelocityDegS +
        0.35 * measuredVelocity;

    previousVelocityCount = currentCount;
    previousVelocityUpdateUs = currentTimeUs;
  }
}

// ============================================================================
// MOTOR ENABLE / DISABLE
// ============================================================================

void enableMotor()
{
  digitalWrite(ENABLE_PIN, LOW); // DRV8825 ENABLE is active LOW.
}

void disableMotor()
{
  digitalWrite(ENABLE_PIN, HIGH);
}

// ============================================================================
// MOTOR RATE RAMP
// ============================================================================

void updateMotorRateToward(float desiredRate)
{
  unsigned long currentTimeUs = micros();

  if (previousMotorRampUs == 0)
  {
    previousMotorRampUs = currentTimeUs;
    currentMotorRateStepsS = 0.0;
    return;
  }

  unsigned long elapsedUs = currentTimeUs - previousMotorRampUs;

  if (elapsedUs < 500)
  {
    return;
  }

  float elapsedSeconds = elapsedUs / 1000000.0;
  float maximumChange = MOTOR_ACCELERATION_STEPS_S2 * elapsedSeconds;

  float difference = desiredRate - currentMotorRateStepsS;
  difference = clampFloat(difference, -maximumChange, maximumChange);

  currentMotorRateStepsS += difference;
  previousMotorRampUs = currentTimeUs;
}

// ============================================================================
// SWING TARGET
// ============================================================================

void setSwingTargetSide(int side)
{
  side = side >= 0 ? 1 : -1;
  swingTargetSide = side;

  long requestedTarget = (long)side * swingAmplitudeSteps;

  swingTargetSteps = clampLong(
      requestedTarget,
      -MAX_SWING_AMPLITUDE_STEPS,
      MAX_SWING_AMPLITUDE_STEPS
  );

  clearMotorCommand();
  lastSwingShiftMs = millis();

  Serial.print("New swing target: ");
  Serial.print(side > 0 ? "RIGHT" : "LEFT");
  Serial.print(" | Target=");
  Serial.print(swingTargetSteps);
  Serial.print(" steps | Angle=");
  Serial.print(swingTargetSteps * MOTOR_STEP_ANGLE_DEG, 1);
  Serial.println(" degrees");
}

void increaseSwingAmplitude()
{
  swingAmplitudeSteps += SWING_AMPLITUDE_INCREMENT_STEPS;

  if (swingAmplitudeSteps > MAX_SWING_AMPLITUDE_STEPS)
  {
    swingAmplitudeSteps = MAX_SWING_AMPLITUDE_STEPS;
  }
}

// ============================================================================
// POSITION-CONTROLLED SWING MOTOR
// ============================================================================

void serviceSwingPositionMotor()
{
  long distanceSteps = swingTargetSteps - armPositionSteps;

  if (distanceSteps == 0)
  {
    clearMotorCommand();
    return;
  }

  int requiredDirection = distanceSteps > 0 ? 1 : -1;
  long remainingSteps = labs(distanceSteps);

  float brakingSpeed = sqrtf(
      2.0 * MOTOR_ACCELERATION_STEPS_S2 * remainingSteps
  );

  float desiredSpeed = brakingSpeed;

  if (desiredSpeed > MAX_SWING_SPEED_STEPS_S)
  {
    desiredSpeed = MAX_SWING_SPEED_STEPS_S;
  }

  if (remainingSteps > 5 && desiredSpeed < 250.0)
  {
    desiredSpeed = 250.0;
  }

  float desiredRate = requiredDirection * desiredSpeed;
  updateMotorRateToward(desiredRate);

  float absoluteRate = fabs(currentMotorRateStepsS);

  if (absoluteRate < 10.0)
  {
    return;
  }

  int actualDirection = currentMotorRateStepsS > 0.0 ? 1 : -1;

  if (actualDirection != requiredDirection)
  {
    currentMotorRateStepsS = 0.0;
    return;
  }

  if (actualDirection > 0 && armPositionSteps >= HARD_ARM_LIMIT_STEPS)
  {
    clearMotorCommand();
    Serial.println("POSITIVE ARM LIMIT REACHED");
    return;
  }

  if (actualDirection < 0 && armPositionSteps <= -HARD_ARM_LIMIT_STEPS)
  {
    clearMotorCommand();
    Serial.println("NEGATIVE ARM LIMIT REACHED");
    return;
  }

  unsigned long stepIntervalUs =
      (unsigned long)(1000000.0 / absoluteRate);

  unsigned long currentTimeUs = micros();

  if ((unsigned long)(currentTimeUs - previousStepUs) < stepIntervalUs)
  {
    return;
  }

  digitalWrite(DIR_PIN, actualDirection > 0 ? HIGH : LOW);
  delayMicroseconds(3);

  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(STEP_PIN, LOW);

  armPositionSteps += actualDirection;
  previousStepUs = currentTimeUs;

  if (armPositionSteps == swingTargetSteps)
  {
    clearMotorCommand();

    Serial.print("SWING TARGET REACHED: ");
    Serial.print(armPositionSteps);
    Serial.print(" steps, ");
    Serial.print(armPositionSteps * MOTOR_STEP_ANGLE_DEG, 1);
    Serial.println(" degrees");
  }
}

// ============================================================================
// RATE-CONTROLLED PID MOTOR
// ============================================================================

void serviceRateControlledMotor(float maximumSpeed)
{
  float desiredRate = clampFloat(
      requestedMotorRateStepsS,
      -maximumSpeed,
      maximumSpeed
  );

  /*
    DOWN_STABILIZING uses its own approximately +/-120-degree limit.
    CATCHING and UP_BALANCING keep the original soft limit.
  */
  long activeSoftLimitSteps =
      controllerState == DOWN_STABILIZING
          ? DOWN_ARM_LIMIT_STEPS
          : SOFT_ARM_LIMIT_STEPS;

  // Block only outward motion; always permit recovery toward the centre.
  if (armPositionSteps >= activeSoftLimitSteps && desiredRate > 0.0)
  {
    desiredRate = 0.0;
  }

  if (armPositionSteps <= -activeSoftLimitSteps && desiredRate < 0.0)
  {
    desiredRate = 0.0;
  }

  /*
    Stop any remaining outward velocity immediately at the active limit.
    This prevents the acceleration ramp from carrying the arm past the limit.
  */
  if (armPositionSteps >= activeSoftLimitSteps &&
      currentMotorRateStepsS > 0.0)
  {
    currentMotorRateStepsS = 0.0;
  }

  if (armPositionSteps <= -activeSoftLimitSteps &&
      currentMotorRateStepsS < 0.0)
  {
    currentMotorRateStepsS = 0.0;
  }

  updateMotorRateToward(desiredRate);

  float absoluteRate = fabs(currentMotorRateStepsS);

  if (absoluteRate < 8.0)
  {
    return;
  }

  int direction = currentMotorRateStepsS > 0.0 ? 1 : -1;

  if (direction > 0 && armPositionSteps >= HARD_ARM_LIMIT_STEPS)
  {
    clearMotorCommand();
    Serial.println("POSITIVE HARD ARM LIMIT REACHED");
    return;
  }

  if (direction < 0 && armPositionSteps <= -HARD_ARM_LIMIT_STEPS)
  {
    clearMotorCommand();
    Serial.println("NEGATIVE HARD ARM LIMIT REACHED");
    return;
  }

  unsigned long stepIntervalUs =
      (unsigned long)(1000000.0 / absoluteRate);

  unsigned long currentTimeUs = micros();

  if ((unsigned long)(currentTimeUs - previousStepUs) < stepIntervalUs)
  {
    return;
  }

  digitalWrite(DIR_PIN, direction > 0 ? HIGH : LOW);
  delayMicroseconds(3);

  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(STEP_PIN, LOW);

  armPositionSteps += direction;
  previousStepUs = currentTimeUs;
}

void serviceStepperMotor()
{
  if (controllerState == SWING_UP)
  {
    serviceSwingPositionMotor();
  }
  else if (controllerState == CATCHING)
  {
    serviceRateControlledMotor(MAX_CATCH_SPEED_STEPS_S);
  }
  else if (controllerState == UP_BALANCING)
  {
    serviceRateControlledMotor(MAX_UP_BALANCE_SPEED_STEPS_S);
  }
  else if (controllerState == DOWN_STABILIZING)
  {
    serviceRateControlledMotor(MAX_DOWN_BALANCE_SPEED_STEPS_S);
  }
}

// ============================================================================
// FOUR-POSITION CALIBRATION
// ============================================================================

void startCalibration()
{
  clearMotorCommand();
  disableMotor();

  calibrationActive = true;
  calibrationIndex = 0;
  controllerState = STOPPED;

  Serial.println();
  Serial.println("FOUR-POSITION CALIBRATION STARTED");
  Serial.println("Rotate in the SAME direction for all positions.");
  Serial.print("Place pendulum at ");
  Serial.print(calibrationNames[0]);
  Serial.println(" and send N.");
}

void finishCalibration()
{
  long quarterCounts[4];

  for (int i = 0; i < 4; i++)
  {
    quarterCounts[i] = labs(
        calibrationCounts[i + 1] - calibrationCounts[i]
    );
  }

  long totalQuarterCounts =
      quarterCounts[0] + quarterCounts[1] +
      quarterCounts[2] + quarterCounts[3];

  float averageQuarter = totalQuarterCounts / 4.0;

  if (averageQuarter < 2.0)
  {
    Serial.println("CALIBRATION FAILED: encoder movement was too small.");
    calibrationActive = false;
    return;
  }

  encoderCountsPerRevolution = averageQuarter * 4.0;

  long completeTurnDifference =
      calibrationCounts[4] - calibrationCounts[0];

  encoderDirection = completeTurnDifference >= 0 ? 1 : -1;

  encoderZeroCount = calibrationCounts[4];
  pendulumAngleDeg = 0.0;
  encoderZeroSet = true;
  calibrationActive = false;
  controllerState = READY;

  resetVelocityEstimator();

  Serial.println();
  Serial.println("CALIBRATION COMPLETED");
  Serial.print("DOWN to RIGHT counts: ");
  Serial.println(quarterCounts[0]);
  Serial.print("RIGHT to UP counts: ");
  Serial.println(quarterCounts[1]);
  Serial.print("UP to LEFT counts: ");
  Serial.println(quarterCounts[2]);
  Serial.print("LEFT to DOWN counts: ");
  Serial.println(quarterCounts[3]);
  Serial.print("Counts per revolution: ");
  Serial.println(encoderCountsPerRevolution, 1);
  Serial.print("Encoder direction: ");
  Serial.println(encoderDirection);
  Serial.println("Final DOWN position is now 0 degrees.");
}

void saveCalibrationPoint()
{
  if (!calibrationActive)
  {
    Serial.println("Send C before using N.");
    return;
  }

  calibrationCounts[calibrationIndex] = readEncoderSafely();

  Serial.print("Saved ");
  Serial.print(calibrationNames[calibrationIndex]);
  Serial.print(" at raw count ");
  Serial.println(calibrationCounts[calibrationIndex]);

  calibrationIndex++;

  if (calibrationIndex >= 5)
  {
    finishCalibration();
    return;
  }

  Serial.print("Move to ");
  Serial.print(calibrationNames[calibrationIndex]);
  Serial.println(" in the SAME direction and send N.");
}

// ============================================================================
// ZERO AND ARM CENTRE
// ============================================================================

void setDownwardZero()
{
  if (controllerIsActive())
  {
    Serial.println("Send X before setting zero.");
    return;
  }

  if (calibrationActive)
  {
    Serial.println("Complete calibration first.");
    return;
  }

  encoderZeroCount = readEncoderSafely();
  pendulumAngleDeg = 0.0;
  encoderZeroSet = true;
  controllerState = READY;

  resetVelocityEstimator();

  Serial.println("Current pendulum position set as DOWN = 0 degrees.");
}

void setArmCentre()
{
  if (controllerIsActive())
  {
    Serial.println("Send X before setting the arm centre.");
    return;
  }

  armPositionSteps = 0;
  swingTargetSteps = 0;
  armCentreSet = true;
  clearMotorCommand();

  Serial.println("Current rotary-arm position set as centre.");
  Serial.print("Soft arm limit: +/-");
  Serial.print(SOFT_ARM_LIMIT_STEPS * MOTOR_STEP_ANGLE_DEG, 1);
  Serial.println(" degrees");
  Serial.print("Hard arm limit: +/-");
  Serial.print(HARD_ARM_LIMIT_STEPS * MOTOR_STEP_ANGLE_DEG, 1);
  Serial.println(" degrees");
  Serial.print("Total maximum arm movement: ");
  Serial.print(2 * HARD_ARM_LIMIT_STEPS * MOTOR_STEP_ANGLE_DEG, 1);
  Serial.println(" degrees");
}

// ============================================================================
// MODE ENTRY FUNCTIONS
// ============================================================================

void enterSwingUpMode()
{
  controllerState = SWING_UP;
  autoReturnToSwingUp = true;
  pidIntegral = 0.0;
  bottomCrossingArmed = false;
  swingAmplitudeSteps = INITIAL_SWING_AMPLITUDE_STEPS;
  captureStableStartMs = 0;

  enableMotor();
  clearMotorCommand();

  Serial.println();
  Serial.println("SWING-UP MODE STARTED");
  Serial.println("The controller will automatically enter CATCHING near UP.");

  setSwingTargetSide(swingPolarity);
}

void enterUprightCapture(bool returnToSwingUpIfLost)
{
  controllerState = CATCHING;
  autoReturnToSwingUp = returnToSwingUpIfLost;
  pidIntegral = 0.0;
  captureStartMs = millis();
  captureStableStartMs = 0;

  clearMotorCommand();
  enableMotor();

  Serial.println();
  Serial.println("UPRIGHT REGION ENTERED");
  Serial.println("CATCHING CONTROLLER STARTED");
}

void enterUpBalancingMode()
{
  controllerState = UP_BALANCING;
  pidIntegral = 0.0;
  captureStableStartMs = 0;
  clearMotorCommand();
  enableMotor();

  Serial.println();
  Serial.println("UPRIGHT CAPTURED");
  Serial.println("UP PID BALANCING STARTED");
}

void enterDownStabilizingMode()
{
  controllerState = DOWN_STABILIZING;
  autoReturnToSwingUp = false;
  pidIntegral = 0.0;
  clearMotorCommand();
  enableMotor();

  Serial.println();
  Serial.println("DOWN PID STABILIZATION STARTED");
  Serial.print("DOWN motor-arm limit: +/-");
  Serial.print(DOWN_ARM_LIMIT_STEPS);
  Serial.print(" steps = +/-");
  Serial.print(DOWN_ARM_LIMIT_STEPS * MOTOR_STEP_ANGLE_DEG, 1);
  Serial.println(" degrees");
}

void stopController(const char *message)
{
  controllerState = STOPPED;
  autoReturnToSwingUp = false;
  pidIntegral = 0.0;
  clearMotorCommand();
  disableMotor();

  Serial.println();
  Serial.println(message);
  Serial.println("Motor driver disabled.");
}

// ============================================================================
// START COMMAND FUNCTIONS
// ============================================================================

bool referencesAreReady()
{
  if (!encoderZeroSet)
  {
    Serial.println("Complete calibration or send Z first.");
    return false;
  }

  if (!armCentreSet)
  {
    Serial.println("Physically centre the arm and send H first.");
    return false;
  }

  return true;
}

void startSwingUp()
{
  if (!referencesAreReady()) return;
  enterSwingUpMode();
}

void startManualUpCapture()
{
  if (!referencesAreReady()) return;

  float uprightError = calculateUprightError(pendulumAngleDeg);

  if (fabs(uprightError) > UP_CAPTURE_ENTRY_DEG)
  {
    Serial.print("Move the pendulum closer to UP. Current UP error = ");
    Serial.print(uprightError, 1);
    Serial.println(" degrees");
    return;
  }

  enterUprightCapture(false);
}

void startDownStabilization()
{
  if (!referencesAreReady()) return;

  float downError = calculateDownwardError(pendulumAngleDeg);

  if (fabs(downError) > DOWN_START_WINDOW_DEG)
  {
    Serial.print("Move the pendulum closer to DOWN. Current error = ");
    Serial.print(downError, 1);
    Serial.println(" degrees");
    return;
  }

  enterDownStabilizingMode();
}

// ============================================================================
// SWING-UP CONTROLLER
// ============================================================================

void runSwingUpController()
{
  float uprightError = calculateUprightError(pendulumAngleDeg);

  /*
    Capture immediately when entering the upright region.
    A separate CATCHING controller handles the remaining velocity.
    We deliberately do not use the old strict velocity gate here.
  */
  if (fabs(uprightError) <= UP_CAPTURE_ENTRY_DEG)
  {
    enterUprightCapture(true);
    return;
  }

  float absoluteAngle = fabs(pendulumAngleDeg);

  if (absoluteAngle >= BOTTOM_REARM_DEG)
  {
    bottomCrossingArmed = true;
  }

  bool nearBottom = absoluteAngle <= BOTTOM_WINDOW_DEG;
  bool enoughTime =
      millis() - lastSwingShiftMs >= MINIMUM_SHIFT_INTERVAL_MS;
  bool motorAtTarget =
      labs(swingTargetSteps - armPositionSteps) <= 1;

  if (bottomCrossingArmed && nearBottom && enoughTime)
  {
    bottomCrossingArmed = false;

    int requiredSide;

    if (lastPendulumDirection == 0)
    {
      requiredSide = -swingTargetSide;
    }
    else
    {
      requiredSide = lastPendulumDirection * swingPolarity;

      if (requiredSide == swingTargetSide)
      {
        requiredSide = -swingTargetSide;
      }
    }

    increaseSwingAmplitude();
    setSwingTargetSide(requiredSide);
    return;
  }

  if (!bottomCrossingArmed &&
      absoluteAngle < BOTTOM_REARM_DEG &&
      motorAtTarget &&
      millis() - lastSwingShiftMs >= STARTUP_SHIFT_INTERVAL_MS)
  {
    increaseSwingAmplitude();
    setSwingTargetSide(-swingTargetSide);
  }
}

// ============================================================================
// UPRIGHT CATCHING CONTROLLER
// ============================================================================

void runCatchingController(float elapsedSeconds)
{
  float error = calculateUprightError(pendulumAngleDeg);

  if (fabs(error) > UP_CAPTURE_ABORT_DEG)
  {
    if (autoReturnToSwingUp)
    {
      Serial.println("Capture lost. Returning to swing-up.");
      enterSwingUpMode();
    }
    else
    {
      stopController("MANUAL UP CAPTURE LOST");
    }
    return;
  }

  // Aggressive PD braking for a pendulum that arrives at UP with velocity.
  float catchOutput = upBalancePolarity *
      (CATCH_KP * error - CATCH_KD * pendulumVelocityDegS);

  float centreCorrection =
      -CATCH_ARM_CENTRE_GAIN * armPositionSteps;

  requestedMotorRateStepsS = clampFloat(
      catchOutput + centreCorrection,
      -MAX_CATCH_SPEED_STEPS_S,
      MAX_CATCH_SPEED_STEPS_S
  );

  bool sufficientlyClose =
      fabs(error) <= UP_CAPTURE_STABLE_ERROR_DEG;

  bool sufficientlySlow =
      fabs(pendulumVelocityDegS) <= UP_CAPTURE_STABLE_SPEED_DEG_S;

  if (sufficientlyClose && sufficientlySlow)
  {
    if (captureStableStartMs == 0)
    {
      captureStableStartMs = millis();
    }
    else if (millis() - captureStableStartMs >=
             UP_CAPTURE_STABLE_TIME_MS)
    {
      enterUpBalancingMode();
      return;
    }
  }
  else
  {
    captureStableStartMs = 0;
  }

  if (millis() - captureStartMs >= UP_CAPTURE_TIMEOUT_MS)
  {
    if (autoReturnToSwingUp)
    {
      Serial.println("Capture timeout. Returning to swing-up.");
      enterSwingUpMode();
    }
    else
    {
      stopController("MANUAL UP CAPTURE TIMEOUT");
    }
  }
}

// ============================================================================
// UPRIGHT PID CONTROLLER
// ============================================================================

void runUpBalanceController(float elapsedSeconds)
{
  float error = calculateUprightError(pendulumAngleDeg);

  if (fabs(error) > UP_FALL_WINDOW_DEG)
  {
    if (autoReturnToSwingUp)
    {
      Serial.println("UP position lost. Returning to swing-up.");
      enterSwingUpMode();
    }
    else
    {
      stopController("UP POSITION LOST");
    }
    return;
  }

  float controlledError = error;

  if (fabs(controlledError) < UP_DEADBAND_DEG)
  {
    controlledError = 0.0;
  }

  pidIntegral += controlledError * elapsedSeconds;
  pidIntegral = clampFloat(
      pidIntegral,
      -UP_PID_INTEGRAL_LIMIT,
      UP_PID_INTEGRAL_LIMIT
  );

  float pidOutput = upBalancePolarity *
      (UP_KP * controlledError +
       UP_KI * pidIntegral -
       UP_KD * pendulumVelocityDegS);

  float centreCorrection =
      -UP_ARM_CENTRE_GAIN * armPositionSteps;

  requestedMotorRateStepsS = clampFloat(
      pidOutput + centreCorrection,
      -MAX_UP_BALANCE_SPEED_STEPS_S,
      MAX_UP_BALANCE_SPEED_STEPS_S
  );
}

// ============================================================================
// DOWNWARD PID CONTROLLER
// ============================================================================

void runDownStabilizingController(float elapsedSeconds)
{
  float error = calculateDownwardError(pendulumAngleDeg);

  if (fabs(error) > DOWN_EXIT_WINDOW_DEG)
  {
    stopController("DOWN POSITION LOST");
    return;
  }

  float controlledError = error;

  if (fabs(controlledError) < DOWN_DEADBAND_DEG)
  {
    controlledError = 0.0;
  }

  pidIntegral += controlledError * elapsedSeconds;
  pidIntegral = clampFloat(
      pidIntegral,
      -DOWN_PID_INTEGRAL_LIMIT,
      DOWN_PID_INTEGRAL_LIMIT
  );

  float pidOutput = downBalancePolarity *
      (DOWN_KP * controlledError +
       DOWN_KI * pidIntegral -
       DOWN_KD * pendulumVelocityDegS);

  float centreCorrection =
      -DOWN_ARM_CENTRE_GAIN * armPositionSteps;

  requestedMotorRateStepsS = clampFloat(
      pidOutput + centreCorrection,
      -MAX_DOWN_BALANCE_SPEED_STEPS_S,
      MAX_DOWN_BALANCE_SPEED_STEPS_S
  );
}

// ============================================================================
// CONTROLLER UPDATE
// ============================================================================

void updateController()
{
  unsigned long currentTimeUs = micros();

  if (previousControllerUpdateUs == 0)
  {
    previousControllerUpdateUs = currentTimeUs;
    return;
  }

  unsigned long elapsedUs =
      currentTimeUs - previousControllerUpdateUs;

  if (elapsedUs < 5000)
  {
    return;
  }

  float elapsedSeconds = elapsedUs / 1000000.0;
  previousControllerUpdateUs = currentTimeUs;

  if (controllerState == SWING_UP)
  {
    runSwingUpController();
  }
  else if (controllerState == CATCHING)
  {
    runCatchingController(elapsedSeconds);
  }
  else if (controllerState == UP_BALANCING)
  {
    runUpBalanceController(elapsedSeconds);
  }
  else if (controllerState == DOWN_STABILIZING)
  {
    runDownStabilizingController(elapsedSeconds);
  }
}

// ============================================================================
// RESET / STOP
// ============================================================================

void emergencyStop()
{
  stopController("EMERGENCY STOP");
}

void resetController()
{
  pidIntegral = 0.0;
  bottomCrossingArmed = false;
  calibrationActive = false;
  autoReturnToSwingUp = false;
  captureStableStartMs = 0;

  clearMotorCommand();
  disableMotor();

  controllerState = encoderZeroSet ? READY : WAITING_FOR_ZERO;

  Serial.println("Controller reset.");
}

// ============================================================================
// PRINTING
// ============================================================================

const char *getStateName()
{
  switch (controllerState)
  {
    case WAITING_FOR_ZERO: return "WAITING_ZERO";
    case READY: return "READY";
    case DOWN_STABILIZING: return "DOWN_PID";
    case SWING_UP: return "SWING_UP";
    case CATCHING: return "CATCHING";
    case UP_BALANCING: return "UP_PID";
    case STOPPED: return "STOPPED";
    default: return "UNKNOWN";
  }
}

void printMeasurements()
{
  long rawCount = readEncoderSafely();

  long relativeCount =
      (rawCount - encoderZeroCount) * encoderDirection;

  float uprightError = calculateUprightError(pendulumAngleDeg);
  float armAngle = armPositionSteps * MOTOR_STEP_ANGLE_DEG;

  Serial.print("Raw=");
  Serial.print(rawCount);
  Serial.print(" | Relative=");
  Serial.print(relativeCount);
  Serial.print(" | CPR=");
  Serial.print(encoderCountsPerRevolution, 1);
  Serial.print(" | Pendulum=");
  Serial.print(pendulumAngleDeg, 1);
  Serial.print(" deg | UpError=");
  Serial.print(uprightError, 1);
  Serial.print(" deg | Velocity=");
  Serial.print(pendulumVelocityDegS, 1);
  Serial.print(" deg/s | Arm=");
  Serial.print(armPositionSteps);
  Serial.print(" steps (");
  Serial.print(armAngle, 1);
  Serial.print(" deg) | Target=");
  Serial.print(swingTargetSteps);
  Serial.print(" | MotorRate=");
  Serial.print(currentMotorRateStepsS, 1);
  Serial.print(" steps/s | State=");
  Serial.println(getStateName());
}

void printMenu()
{
  Serial.println();
  Serial.println("ROTARY PENDULUM COMMANDS");
  Serial.println("-------------------------");
  Serial.println("C = Start four-position calibration");
  Serial.println("N = Save next calibration position");
  Serial.println("Z = Set current DOWN position as zero");
  Serial.println("H = Set current rotary-arm position as centre");
  Serial.println("S = Swing-up and automatically catch UP");
  Serial.println("U = Manually start UP capture near upright");
  Serial.println("D = Start DOWN PID stabilization");
  Serial.println("P = Print measurements");
  Serial.println("X = Emergency stop");
  Serial.println("K = Reverse swing-up direction");
  Serial.println("V = Reverse UP capture/PID direction");
  Serial.println("B = Reverse DOWN PID direction");
  Serial.println("R = Reset controller");
  Serial.println("M = Print this menu");
}

// ============================================================================
// SERIAL COMMANDS
// ============================================================================

void processSerialCommands()
{
  while (Serial.available() > 0)
  {
    char command = Serial.read();

    switch (command)
    {
      case 'C':
      case 'c':
        startCalibration();
        break;

      case 'N':
      case 'n':
        saveCalibrationPoint();
        break;

      case 'Z':
      case 'z':
        setDownwardZero();
        break;

      case 'H':
      case 'h':
        setArmCentre();
        break;

      case 'S':
      case 's':
        if (!controllerIsActive()) startSwingUp();
        else Serial.println("Send X before starting another mode.");
        break;

      case 'U':
      case 'u':
        if (!controllerIsActive()) startManualUpCapture();
        else Serial.println("Send X before starting another mode.");
        break;

      case 'D':
      case 'd':
        if (!controllerIsActive()) startDownStabilization();
        else Serial.println("Send X before starting another mode.");
        break;

      case 'P':
      case 'p':
        printMeasurements();
        break;

      case 'X':
      case 'x':
        emergencyStop();
        break;

      case 'K':
      case 'k':
        if (controllerIsActive())
        {
          Serial.println("Send X before reversing swing polarity.");
        }
        else
        {
          swingPolarity = -swingPolarity;
          Serial.print("Swing polarity = ");
          Serial.println(swingPolarity);
        }
        break;

      case 'V':
      case 'v':
        if (controllerIsActive())
        {
          Serial.println("Send X before reversing UP polarity.");
        }
        else
        {
          upBalancePolarity = -upBalancePolarity;
          Serial.print("UP capture/PID polarity = ");
          Serial.println(upBalancePolarity);
        }
        break;

      case 'B':
      case 'b':
        if (controllerIsActive())
        {
          Serial.println("Send X before reversing DOWN polarity.");
        }
        else
        {
          downBalancePolarity = -downBalancePolarity;
          Serial.print("DOWN PID polarity = ");
          Serial.println(downBalancePolarity);
        }
        break;

      case 'R':
      case 'r':
        resetController();
        break;

      case 'M':
      case 'm':
        printMenu();
        break;

      case '\n':
      case '\r':
        break;

      default:
        Serial.println("Unknown command. Send M for menu.");
        break;
    }
  }
}

// ============================================================================
// SETUP
// ============================================================================

void setup()
{
  Serial.begin(115200);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  pinMode(ENCODER_CLK_PIN, INPUT_PULLUP);
  pinMode(ENCODER_DT_PIN, INPUT_PULLUP);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  disableMotor();

  previousEncoderState =
      (digitalRead(ENCODER_CLK_PIN) << 1) |
      digitalRead(ENCODER_DT_PIN);

  attachInterrupt(
      digitalPinToInterrupt(ENCODER_CLK_PIN),
      encoderInterrupt,
      CHANGE
  );

  attachInterrupt(
      digitalPinToInterrupt(ENCODER_DT_PIN),
      encoderInterrupt,
      CHANGE
  );

  delay(1000);

  Serial.println();
  Serial.println("ROTARY INVERTED PENDULUM CONTROLLER");
  Serial.println("Swing-up + upright catching + UP/DOWN PID");
  Serial.println("-----------------------------------------");
  Serial.print("Maximum swing side-to-side angle: ");
  Serial.print(2 * MAX_SWING_AMPLITUDE_STEPS * MOTOR_STEP_ANGLE_DEG, 1);
  Serial.println(" degrees");
  Serial.print("Soft PID arm limit: +/-");
  Serial.print(SOFT_ARM_LIMIT_STEPS * MOTOR_STEP_ANGLE_DEG, 1);
  Serial.println(" degrees");
  Serial.print("Hard total arm movement: ");
  Serial.print(2 * HARD_ARM_LIMIT_STEPS * MOTOR_STEP_ANGLE_DEG, 1);
  Serial.println(" degrees");
  Serial.print("DOWN-STABLE motor-arm limit: +/-");
  Serial.print(DOWN_ARM_LIMIT_STEPS * MOTOR_STEP_ANGLE_DEG, 1);
  Serial.println(" degrees");

  printMenu();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop()
{
  updateEncoderMeasurements();
  processSerialCommands();
  updateController();
  serviceStepperMotor();

  if (controllerIsActive() &&
      millis() - previousPrintMs >= 150)
  {
    previousPrintMs = millis();
    printMeasurements();
  }
}
