#include "PendulumController.h"
#include <math.h>

const char* PendulumController::_calibrationNames[5] = {
    "DOWN",
    "RIGHT HORIZONTAL",
    "UP",
    "LEFT HORIZONTAL",
    "DOWN AGAIN"
};

PendulumController::PendulumController()
    : _encoder(nullptr), _stepper(nullptr),
      _state(ControllerState::WAITING_FOR_ZERO),
      _encoderCPR(60.0f), _encoderDirection(1), _encoderZeroCount(0),
      _encoderZeroSet(false), _armCentreSet(false),
      _pendulumAngleDeg(0.0f), _pendulumVelocityDegS(0.0f),
      _lastPendulumDirection(0), _prevEncoderCount(0), _prevVelocityCount(0),
      _prevAngleUs(0), _prevVelocityUs(0),
      _calibrationActive(false), _calibrationIndex(0),
      _swingPolarity(1), _swingTargetSide(0), _swingAmplitudeSteps(12),
      _swingTargetSteps(0), _bottomCrossingArmed(false), _autoReturnToSwingUp(false),
      _lastSwingShiftMs(0), _captureStartMs(0), _captureStableStartMs(0),
      _pidIntegral(0.0f), _prevControllerUs(0), _prevPrintMs(0),
      _upBalancePolarity(1), _downBalancePolarity(1),
      _catchKp(9.0f), _catchKd(0.18f),
      _upKp(6.0f), _upKi(0.0f), _upKd(0.30f),
      _downKp(1.2f), _downKi(0.0f), _downKd(0.10f) {
    for (int i = 0; i < 5; i++) {
        _calibrationCounts[i] = 0;
    }
}

void PendulumController::begin(KY040& encoder, DRV8825& stepper) {
    _encoder = &encoder;
    _stepper = &stepper;
    _state = _encoderZeroSet ? ControllerState::READY : ControllerState::WAITING_FOR_ZERO;
}

float PendulumController::wrapAngleTo180(float a) {
    while (a > 180.0f)  a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

float PendulumController::calculateUprightError(float a) {
    return wrapAngleTo180(180.0f - a);
}

float PendulumController::calculateDownwardError(float a) {
    return wrapAngleTo180(0.0f - a);
}

float PendulumController::clampF(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

long PendulumController::clampL(long v, long lo, long hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

bool PendulumController::controllerIsActive() {
    return _state == ControllerState::DOWN_STABILIZING ||
           _state == ControllerState::SWING_UP ||
           _state == ControllerState::CATCHING ||
           _state == ControllerState::UP_BALANCING;
}

bool PendulumController::referencesAreReady() {
    if (!_encoderZeroSet) {
        Serial.println("Complete calibration or send Z first.");
        return false;
    }
    if (!_armCentreSet) {
        Serial.println("Physically centre the arm and send H first.");
        return false;
    }
    return true;
}

void PendulumController::resetVelocityEstimator() {
    long count = _encoder->getRawCount();
    _prevEncoderCount = count;
    _prevVelocityCount = count;
    _prevAngleUs = micros();
    _prevVelocityUs = _prevAngleUs;
    _pendulumVelocityDegS = 0.0f;
}

void PendulumController::updateEncoderMeasurements() {
    unsigned long now = micros();
    if (_prevAngleUs == 0) {
        long currentCount = _encoder->getRawCount();
        _prevAngleUs = now;
        _prevVelocityUs = now;
        _prevEncoderCount = currentCount;
        _prevVelocityCount = currentCount;
        return;
    }

    // Angle update (ANGLE_UPDATE_INTERVAL_US = 5000)
    if ((unsigned long)(now - _prevAngleUs) >= 5000UL) {
        long currentCount = _encoder->getRawCount();
        long relativeCount = (currentCount - _encoderZeroCount) * _encoderDirection;
        float unwrappedAngle = ((float)relativeCount * 360.0f) / _encoderCPR;
        _pendulumAngleDeg = wrapAngleTo180(unwrappedAngle);

        long countDiff = (currentCount - _prevEncoderCount) * _encoderDirection;
        if (countDiff > 0) {
            _lastPendulumDirection = 1;
        } else if (countDiff < 0) {
            _lastPendulumDirection = -1;
        }

        _prevEncoderCount = currentCount;
        _prevAngleUs = now;
    }

    // Velocity update (VELOCITY_UPDATE_INTERVAL_US = 25000)
    if ((unsigned long)(now - _prevVelocityUs) >= 25000UL) {
        long currentCount = _encoder->getRawCount();
        unsigned long elapsedUs = now - _prevVelocityUs;
        long countDiff = (currentCount - _prevVelocityCount) * _encoderDirection;
        float elapsedSeconds = elapsedUs / 1000000.0f;

        float measuredVelocity = ((float)countDiff * 360.0f) / (_encoderCPR * elapsedSeconds);
        // Exponential filter
        _pendulumVelocityDegS = 0.65f * _pendulumVelocityDegS + 0.35f * measuredVelocity;

        _prevVelocityCount = currentCount;
        _prevVelocityUs = now;
    }
}

void PendulumController::update() {
    updateEncoderMeasurements();

    // Run controller logic (update rate command)
    unsigned long now = micros();
    if (_prevControllerUs == 0) {
        _prevControllerUs = now;
        return;
    }

    unsigned long elapsedUs = now - _prevControllerUs;
    if (elapsedUs >= 5000UL) {
        float elapsedS = elapsedUs / 1000000.0f;
        _prevControllerUs = now;

        if (_state == ControllerState::SWING_UP) {
            runSwingUpController();
        } else if (_state == ControllerState::CATCHING) {
            runCatchingController(elapsedS);
        } else if (_state == ControllerState::UP_BALANCING) {
            runUpBalanceController(elapsedS);
        } else if (_state == ControllerState::DOWN_STABILIZING) {
            runDownStabilizingController(elapsedS);
        }
    }

    // Update motor speed/step generation
    serviceMotor();

    // Print measurements periodically
    if (controllerIsActive() && (millis() - _prevPrintMs >= 150)) {
        _prevPrintMs = millis();
        printMeasurements();
    }
}

void PendulumController::serviceMotor() {
    _stepper->updateRamp(7000.0f); // MOTOR_ACCELERATION_STEPS_S2

    if (_state == ControllerState::SWING_UP) {
        serviceSwingPositionMotor();
    } else if (_state == ControllerState::CATCHING) {
        serviceRateControlledMotor(1000.0f); // MAX_CATCH_SPEED_STEPS_S
    } else if (_state == ControllerState::UP_BALANCING) {
        serviceRateControlledMotor(850.0f);  // MAX_UP_BALANCE_SPEED_STEPS_S
    } else if (_state == ControllerState::DOWN_STABILIZING) {
        serviceRateControlledMotor(350.0f);  // MAX_DOWN_BALANCE_SPEED_STEPS_S
    }
}

void PendulumController::serviceSwingPositionMotor() {
    long distanceSteps = _swingTargetSteps - _stepper->getPositionSteps();
    if (distanceSteps == 0) {
        _stepper->clearRateCommand();
        return;
    }

    int reqDir = distanceSteps > 0 ? 1 : -1;
    long remaining = labs(distanceSteps);

    float brakingSpeed = sqrtf(2.0f * 7000.0f * remaining);
    float desiredSpeed = brakingSpeed;

    if (desiredSpeed > 700.0f) { // MAX_SWING_SPEED_STEPS_S = 700
        desiredSpeed = 700.0f;
    }
    if (remaining > 5 && desiredSpeed < 250.0f) {
        desiredSpeed = 250.0f;
    }

    float desiredRate = reqDir * desiredSpeed;
    _stepper->setRate(desiredRate);

    float curRate = _stepper->getCurrentRateStepsS();
    int actualDir = curRate > 0.0f ? 1 : -1;

    if (actualDir != reqDir && fabsf(curRate) > 10.0f) {
        _stepper->clearRateCommand();
        return;
    }

    // Hard Limit checks
    if (actualDir > 0 && _stepper->getPositionSteps() >= 75) { // HARD_ARM_LIMIT_STEPS = 75
        _stepper->clearRateCommand();
        Serial.println("POSITIVE ARM LIMIT REACHED");
        return;
    }
    if (actualDir < 0 && _stepper->getPositionSteps() <= -75) {
        _stepper->clearRateCommand();
        Serial.println("NEGATIVE ARM LIMIT REACHED");
        return;
    }

    // Step output
    _stepper->step();

    if (_stepper->getPositionSteps() == _swingTargetSteps) {
        _stepper->clearRateCommand();
        Serial.print("SWING TARGET REACHED: ");
        Serial.print(_stepper->getPositionSteps());
        Serial.print(" steps, ");
        Serial.print(_stepper->getPositionSteps() * 1.8f, 1); // MOTOR_STEP_ANGLE_DEG = 1.8
        Serial.println(" degrees");
    }
}

void PendulumController::serviceRateControlledMotor(float maxSpeed) {
    float desiredRate = clampF(_stepper->getRequestedRateStepsS(), -maxSpeed, maxSpeed);

    long activeSoftLimitSteps = (_state == ControllerState::DOWN_STABILIZING) ? 66 : 65; // DOWN_ARM_LIMIT_STEPS=66, SOFT_ARM_LIMIT_STEPS=65

    // Block outward movement past soft limit
    if (_stepper->getPositionSteps() >= activeSoftLimitSteps && desiredRate > 0.0f) {
        desiredRate = 0.0f;
    }
    if (_stepper->getPositionSteps() <= -activeSoftLimitSteps && desiredRate < 0.0f) {
        desiredRate = 0.0f;
    }

    // Prevent acceleration ramp carry-through past soft limits
    if (_stepper->getPositionSteps() >= activeSoftLimitSteps && _stepper->getCurrentRateStepsS() > 0.0f) {
        _stepper->clearRateCommand();
    }
    if (_stepper->getPositionSteps() <= -activeSoftLimitSteps && _stepper->getCurrentRateStepsS() < 0.0f) {
        _stepper->clearRateCommand();
    }

    _stepper->setRate(desiredRate);

    // Hard limits check
    int direction = _stepper->getCurrentRateStepsS() > 0.0f ? 1 : -1;
    if (direction > 0 && _stepper->getPositionSteps() >= 75) {
        _stepper->clearRateCommand();
        Serial.println("POSITIVE HARD ARM LIMIT REACHED");
        return;
    }
    if (direction < 0 && _stepper->getPositionSteps() <= -75) {
        _stepper->clearRateCommand();
        Serial.println("NEGATIVE HARD ARM LIMIT REACHED");
        return;
    }

    // Output pulses
    _stepper->step();
}

void PendulumController::runSwingUpController() {
    float uprightError = calculateUprightError(_pendulumAngleDeg);
    if (fabsf(uprightError) <= 36.0f) { // UP_CAPTURE_ENTRY_DEG = 36.0
        enterUprightCapture(true);
        return;
    }

    float absAngle = fabsf(_pendulumAngleDeg);
    if (absAngle >= 45.0f) { // BOTTOM_REARM_DEG = 45.0
        _bottomCrossingArmed = true;
    }

    bool nearBottom = absAngle <= 30.0f; // BOTTOM_WINDOW_DEG = 30.0
    bool enoughTime = (millis() - _lastSwingShiftMs >= 120UL); // MINIMUM_SHIFT_INTERVAL_MS = 120
    bool motorAtTarget = labs(_swingTargetSteps - _stepper->getPositionSteps()) <= 1;

    if (_bottomCrossingArmed && nearBottom && enoughTime) {
        _bottomCrossingArmed = false;
        int reqSide;
        if (_lastPendulumDirection == 0) {
            reqSide = -_swingTargetSide;
        } else {
            reqSide = _lastPendulumDirection * _swingPolarity;
            if (reqSide == _swingTargetSide) {
                reqSide = -_swingTargetSide;
            }
        }
        increaseSwingAmplitude();
        setSwingTargetSide(reqSide);
        return;
    }

    if (!_bottomCrossingArmed && absAngle < 45.0f && motorAtTarget &&
        (millis() - _lastSwingShiftMs >= 500UL)) { // STARTUP_SHIFT_INTERVAL_MS = 500
        increaseSwingAmplitude();
        setSwingTargetSide(-_swingTargetSide);
    }
}

void PendulumController::runCatchingController(float elapsedS) {
    float error = calculateUprightError(_pendulumAngleDeg);
    if (fabsf(error) > 72.0f) { // UP_CAPTURE_ABORT_DEG = 72.0
        if (_autoReturnToSwingUp) {
            Serial.println("Capture lost. Returning to swing-up.");
            enterSwingUpMode();
        } else {
            stopController("MANUAL UP CAPTURE LOST");
        }
        return;
    }

    float catchOutput = _upBalancePolarity * (_catchKp * error - _catchKd * _pendulumVelocityDegS);
    float centreCorrection = -0.5f * _stepper->getPositionSteps(); // CATCH_ARM_CENTRE_GAIN = 0.5

    _stepper->setRate(catchOutput + centreCorrection);

    bool sufficientlyClose = fabsf(error) <= 12.0f; // UP_CAPTURE_STABLE_ERROR_DEG = 12.0
    bool sufficientlySlow  = fabsf(_pendulumVelocityDegS) <= 180.0f; // UP_CAPTURE_STABLE_SPEED_DEG_S = 180.0

    if (sufficientlyClose && sufficientlySlow) {
        if (_captureStableStartMs == 0) {
            _captureStableStartMs = millis();
        } else if (millis() - _captureStableStartMs >= 80UL) { // UP_CAPTURE_STABLE_TIME_MS = 80
            enterUpBalancingMode();
            return;
        }
    } else {
        _captureStableStartMs = 0;
    }

    if (millis() - _captureStartMs >= 1800UL) { // UP_CAPTURE_TIMEOUT_MS = 1800
        if (_autoReturnToSwingUp) {
            Serial.println("Capture timeout. Returning to swing-up.");
            enterSwingUpMode();
        } else {
            stopController("MANUAL UP CAPTURE TIMEOUT");
        }
    }
}

void PendulumController::runUpBalanceController(float elapsedS) {
    float error = calculateUprightError(_pendulumAngleDeg);
    if (fabsf(error) > 48.0f) { // UP_FALL_WINDOW_DEG = 48.0
        if (_autoReturnToSwingUp) {
            Serial.println("UP position lost. Returning to swing-up.");
            enterSwingUpMode();
        } else {
            stopController("UP POSITION LOST");
        }
        return;
    }

    float controlledError = error;
    if (fabsf(controlledError) < 3.0f) { // UP_DEADBAND_DEG = 3.0
        controlledError = 0.0f;
    }

    _pidIntegral += controlledError * elapsedS;
    _pidIntegral = clampF(_pidIntegral, -25.0f, 25.0f); // UP_PID_INTEGRAL_LIMIT = 25.0

    float pidOutput = _upBalancePolarity * (_upKp * controlledError + _upKi * _pidIntegral - _upKd * _pendulumVelocityDegS);
    float centreCorrection = -1.2f * _stepper->getPositionSteps(); // UP_ARM_CENTRE_GAIN = 1.2

    _stepper->setRate(pidOutput + centreCorrection);
}

void PendulumController::runDownStabilizingController(float elapsedS) {
    float error = calculateDownwardError(_pendulumAngleDeg);
    if (fabsf(error) > 60.0f) { // DOWN_EXIT_WINDOW_DEG = 60.0
        stopController("DOWN POSITION LOST");
        return;
    }

    float controlledError = error;
    if (fabsf(controlledError) < 3.0f) { // DOWN_DEADBAND_DEG = 3.0
        controlledError = 0.0f;
    }

    _pidIntegral += controlledError * elapsedS;
    _pidIntegral = clampF(_pidIntegral, -20.0f, 20.0f); // DOWN_PID_INTEGRAL_LIMIT = 20.0

    float pidOutput = _downBalancePolarity * (_downKp * controlledError + _downKi * _pidIntegral - _downKd * _pendulumVelocityDegS);
    float centreCorrection = -1.5f * _stepper->getPositionSteps(); // DOWN_ARM_CENTRE_GAIN = 1.5

    _stepper->setRate(pidOutput + centreCorrection);
}

void PendulumController::enterSwingUpMode() {
    _state = ControllerState::SWING_UP;
    _autoReturnToSwingUp = true;
    _pidIntegral = 0.0f;
    _bottomCrossingArmed = false;
    _swingAmplitudeSteps = 12; // INITIAL_SWING_AMPLITUDE_STEPS
    _captureStableStartMs = 0;

    _stepper->enable();
    _stepper->clearRateCommand();

    Serial.println();
    Serial.println("SWING-UP MODE STARTED");
    Serial.println("The controller will automatically enter CATCHING near UP.");

    setSwingTargetSide(_swingPolarity);
}

void PendulumController::enterUprightCapture(bool returnToSwingUp) {
    _state = ControllerState::CATCHING;
    _autoReturnToSwingUp = returnToSwingUp;
    _pidIntegral = 0.0f;
    _captureStartMs = millis();
    _captureStableStartMs = 0;

    _stepper->clearRateCommand();
    _stepper->enable();

    Serial.println();
    Serial.println("UPRIGHT REGION ENTERED");
    Serial.println("CATCHING CONTROLLER STARTED");
}

void PendulumController::enterUpBalancingMode() {
    _state = ControllerState::UP_BALANCING;
    _pidIntegral = 0.0f;
    _captureStableStartMs = 0;
    _stepper->clearRateCommand();
    _stepper->enable();

    Serial.println();
    Serial.println("UPRIGHT CAPTURED");
    Serial.println("UP PID BALANCING STARTED");
}

void PendulumController::enterDownStabilizingMode() {
    _state = ControllerState::DOWN_STABILIZING;
    _autoReturnToSwingUp = false;
    _pidIntegral = 0.0f;
    _stepper->clearRateCommand();
    _stepper->enable();

    Serial.println();
    Serial.println("DOWN PID STABILIZATION STARTED");
    Serial.print("DOWN motor-arm limit: +/-");
    Serial.print(66);
    Serial.print(" steps = +/-");
    Serial.print(66 * 1.8f, 1);
    Serial.println(" degrees");
}

void PendulumController::stopController(const char* msg) {
    _state = ControllerState::STOPPED;
    _autoReturnToSwingUp = false;
    _pidIntegral = 0.0f;
    _stepper->clearRateCommand();
    _stepper->disable();

    Serial.println();
    Serial.println(msg);
    Serial.println("Motor driver disabled.");
}

void PendulumController::setSwingTargetSide(int side) {
    _swingTargetSide = side >= 0 ? 1 : -1;
    long reqTarget = (long)_swingTargetSide * _swingAmplitudeSteps;

    _swingTargetSteps = clampL(reqTarget, -45, 45); // MAX_SWING_AMPLITUDE_STEPS = 45

    _stepper->clearRateCommand();
    _lastSwingShiftMs = millis();

    Serial.print("New swing target: ");
    Serial.print(_swingTargetSide > 0 ? "RIGHT" : "LEFT");
    Serial.print(" | Target=");
    Serial.print(_swingTargetSteps);
    Serial.print(" steps | Angle=");
    Serial.print(_swingTargetSteps * 1.8f, 1);
    Serial.println(" degrees");
}

void PendulumController::increaseSwingAmplitude() {
    _swingAmplitudeSteps += 3; // SWING_AMPLITUDE_INCREMENT_STEPS = 3
    if (_swingAmplitudeSteps > 45) {
        _swingAmplitudeSteps = 45;
    }
}

void PendulumController::startCalibration() {
    _stepper->clearRateCommand();
    _stepper->disable();

    _calibrationActive = true;
    _calibrationIndex = 0;
    _state = ControllerState::STOPPED;

    Serial.println();
    Serial.println("FOUR-POSITION CALIBRATION STARTED");
    Serial.println("Rotate in the SAME direction for all positions.");
    Serial.print("Place pendulum at ");
    Serial.print(_calibrationNames[0]);
    Serial.println(" and send N.");
}

void PendulumController::saveCalibrationPoint() {
    if (!_calibrationActive) {
        Serial.println("Send C before using N.");
        return;
    }

    _calibrationCounts[_calibrationIndex] = _encoder->getRawCount();

    Serial.print("Saved ");
    Serial.print(_calibrationNames[_calibrationIndex]);
    Serial.print(" at raw count ");
    Serial.println(_calibrationCounts[_calibrationIndex]);

    _calibrationIndex++;
    if (_calibrationIndex >= 5) {
        finishCalibration();
        return;
    }

    Serial.print("Move to ");
    Serial.print(_calibrationNames[_calibrationIndex]);
    Serial.println(" in the SAME direction and send N.");
}

void PendulumController::finishCalibration() {
    long quarterCounts[4];
    for (int i = 0; i < 4; i++) {
        quarterCounts[i] = labs(_calibrationCounts[i+1] - _calibrationCounts[i]);
    }

    long totalQuarter = quarterCounts[0] + quarterCounts[1] + quarterCounts[2] + quarterCounts[3];
    float avgQuarter = totalQuarter / 4.0f;

    if (avgQuarter < 2.0f) {
        Serial.println("CALIBRATION FAILED: encoder movement was too small.");
        _calibrationActive = false;
        return;
    }

    _encoderCPR = avgQuarter * 4.0f;
    long completeTurnDiff = _calibrationCounts[4] - _calibrationCounts[0];
    _encoderDirection = completeTurnDiff >= 0 ? 1 : -1;
    _encoderZeroCount = _calibrationCounts[4];

    _pendulumAngleDeg = 0.0f;
    _encoderZeroSet = true;
    _calibrationActive = false;
    _state = ControllerState::READY;

    resetVelocityEstimator();

    Serial.println();
    Serial.println("CALIBRATION COMPLETED");
    for (int i = 0; i < 4; i++) {
        Serial.print(_calibrationNames[i]);
        Serial.print(" to ");
        Serial.print(_calibrationNames[i+1]);
        Serial.print(" counts: ");
        Serial.println(quarterCounts[i]);
    }
    Serial.print("Counts per revolution: ");
    Serial.println(_encoderCPR, 1);
    Serial.print("Encoder direction: ");
    Serial.println(_encoderDirection);
    Serial.println("Final DOWN position is now 0 degrees.");
}

void PendulumController::setDownwardZero() {
    if (controllerIsActive()) {
        Serial.println("Send X before setting zero.");
        return;
    }
    if (_calibrationActive) {
        Serial.println("Complete calibration first.");
        return;
    }

    _encoderZeroCount = _encoder->getRawCount();
    _pendulumAngleDeg = 0.0f;
    _encoderZeroSet = true;
    _state = ControllerState::READY;

    resetVelocityEstimator();
    Serial.println("Current pendulum position set as DOWN = 0 degrees.");
}

void PendulumController::setArmCentre() {
    if (controllerIsActive()) {
        Serial.println("Send X before setting the arm centre.");
        return;
    }

    _stepper->setPositionSteps(0);
    _swingTargetSteps = 0;
    _armCentreSet = true;
    _stepper->clearRateCommand();

    Serial.println("Current rotary-arm position set as centre.");
    Serial.print("Soft arm limit: +/-");
    Serial.print(65 * 1.8f, 1);
    Serial.println(" degrees");
    Serial.print("Hard arm limit: +/-");
    Serial.print(75 * 1.8f, 1);
    Serial.println(" degrees");
}

void PendulumController::startSwingUp() {
    if (!referencesAreReady()) return;
    enterSwingUpMode();
}

void PendulumController::startManualUpCapture() {
    if (!referencesAreReady()) return;
    float error = calculateUprightError(_pendulumAngleDeg);
    if (fabsf(error) > 36.0f) {
        Serial.print("Move the pendulum closer to UP. Current error = ");
        Serial.print(error, 1);
        Serial.println(" degrees");
        return;
    }
    enterUprightCapture(false);
}

void PendulumController::startDownStabilization() {
    if (!referencesAreReady()) return;
    float error = calculateDownwardError(_pendulumAngleDeg);
    if (fabsf(error) > 30.0f) {
        Serial.print("Move the pendulum closer to DOWN. Current error = ");
        Serial.print(error, 1);
        Serial.println(" degrees");
        return;
    }
    enterDownStabilizingMode();
}

void PendulumController::emergencyStop() {
    stopController("EMERGENCY STOP");
}

void PendulumController::resetController() {
    _pidIntegral = 0.0f;
    _bottomCrossingArmed = false;
    _calibrationActive = false;
    _autoReturnToSwingUp = false;
    _captureStableStartMs = 0;

    _stepper->clearRateCommand();
    _stepper->disable();

    _state = _encoderZeroSet ? ControllerState::READY : ControllerState::WAITING_FOR_ZERO;
    Serial.println("Controller reset.");
}

long PendulumController::getArmPositionSteps() const {
    return _stepper ? _stepper->getPositionSteps() : 0;
}

float PendulumController::getMotorRateStepsS() const {
    return _stepper ? _stepper->getCurrentRateStepsS() : 0.0f;
}

const char* PendulumController::getStateName() const {
    switch (_state) {
        case ControllerState::WAITING_FOR_ZERO: return "WAITING_ZERO";
        case ControllerState::READY: return "READY";
        case ControllerState::DOWN_STABILIZING: return "DOWN_PID";
        case ControllerState::SWING_UP: return "SWING_UP";
        case ControllerState::CATCHING: return "CATCHING";
        case ControllerState::UP_BALANCING: return "UP_PID";
        case ControllerState::STOPPED: return "STOPPED";
        default: return "UNKNOWN";
    }
}

void PendulumController::printMeasurements() {
    long rawCount = _encoder->getRawCount();
    long relativeCount = (rawCount - _encoderZeroCount) * _encoderDirection;
    float upError = calculateUprightError(_pendulumAngleDeg);
    float armAngle = _stepper->getPositionSteps() * 1.8f;

    Serial.print("Raw=");
    Serial.print(rawCount);
    Serial.print(" | Relative=");
    Serial.print(relativeCount);
    Serial.print(" | CPR=");
    Serial.print(_encoderCPR, 1);
    Serial.print(" | Pendulum=");
    Serial.print(_pendulumAngleDeg, 1);
    Serial.print(" deg | UpError=");
    Serial.print(upError, 1);
    Serial.print(" deg | Velocity=");
    Serial.print(_pendulumVelocityDegS, 1);
    Serial.print(" deg/s | Arm=");
    Serial.print(_stepper->getPositionSteps());
    Serial.print(" steps (");
    Serial.print(armAngle, 1);
    Serial.print(" deg) | Target=");
    Serial.print(_swingTargetSteps);
    Serial.print(" | MotorRate=");
    Serial.print(_stepper->getCurrentRateStepsS(), 1);
    Serial.print(" steps/s | State=");
    Serial.println(getStateName());
}

void PendulumController::printMenu() {
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

void PendulumController::processCommand(char command) {
    switch (command) {
        case 'C': case 'c': startCalibration(); break;
        case 'N': case 'n': saveCalibrationPoint(); break;
        case 'Z': case 'z': setDownwardZero(); break;
        case 'H': case 'h': setArmCentre(); break;
        case 'S': case 's':
            if (!controllerIsActive()) startSwingUp();
            else Serial.println("Send X before starting another mode.");
            break;
        case 'U': case 'u':
            if (!controllerIsActive()) startManualUpCapture();
            else Serial.println("Send X before starting another mode.");
            break;
        case 'D': case 'd':
            if (!controllerIsActive()) startDownStabilization();
            else Serial.println("Send X before starting another mode.");
            break;
        case 'P': case 'p': printMeasurements(); break;
        case 'X': case 'x': emergencyStop(); break;
        case 'K': case 'k':
            if (controllerIsActive()) {
                Serial.println("Send X before reversing swing polarity.");
            } else {
                _swingPolarity = -_swingPolarity;
                Serial.print("Swing polarity = ");
                Serial.println(_swingPolarity);
            }
            break;
        case 'V': case 'v':
            if (controllerIsActive()) {
                Serial.println("Send X before reversing UP polarity.");
            } else {
                _upBalancePolarity = -_upBalancePolarity;
                Serial.print("UP capture/PID polarity = ");
                Serial.println(_upBalancePolarity);
            }
            break;
        case 'B': case 'b':
            if (controllerIsActive()) {
                Serial.println("Send X before reversing DOWN polarity.");
            } else {
                _downBalancePolarity = -_downBalancePolarity;
                Serial.print("DOWN PID polarity = ");
                Serial.println(_downBalancePolarity);
            }
            break;
        case 'R': case 'r': resetController(); break;
        case 'M': case 'm': printMenu(); break;
        default:
            break;
    }
}
