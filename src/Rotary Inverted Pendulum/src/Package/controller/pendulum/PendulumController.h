#pragma once
#include <Arduino.h>
#include "../../devices/encoder/KY040.h"
#include "../../devices/stepper/DRV8825.h"

// ============================================================================
// Controller State Machine
// ============================================================================
enum class ControllerState {
    WAITING_FOR_ZERO,
    READY,
    DOWN_STABILIZING,
    SWING_UP,
    CATCHING,
    UP_BALANCING,
    STOPPED
};

// ============================================================================
// PendulumController
// Encapsulates the full swing-up + catching + UP/DOWN PID state machine.
// Mirrors the verified monolithic ESP32 code.
// ============================================================================
class PendulumController {
public:
    PendulumController();
    void begin(KY040& encoder, DRV8825& stepper);

    // Call from loop() - runs encoder measurement, controller, motor service
    void update();

    // Process one serial command character
    void processCommand(char cmd);

    // ---- State queries ----
    ControllerState getState()     const { return _state; }
    const char*     getStateName() const;

    // ---- Measurements ----
    float getPendulumAngleDeg()    const { return _pendulumAngleDeg; }
    float getPendulumVelocityDegS() const { return _pendulumVelocityDegS; }
    long  getArmPositionSteps()    const;
    float getMotorRateStepsS()     const;
    float getEncoderCPR()          const { return _encoderCPR; }
    bool  isEncoderZeroSet()       const { return _encoderZeroSet; }
    bool  isArmCentreSet()         const { return _armCentreSet; }

    // ---- Polarity flags ----
    int getSwingPolarity()    const { return _swingPolarity; }
    int getUpPolarity()       const { return _upBalancePolarity; }
    int getDownPolarity()     const { return _downBalancePolarity; }

    // ---- Tuning (used by WebService) ----
    void setCatchGains(float kp, float kd)           { _catchKp=kp; _catchKd=kd; }
    void setUpGains(float kp, float ki, float kd)    { _upKp=kp; _upKi=ki; _upKd=kd; }
    void setDownGains(float kp, float ki, float kd)  { _downKp=kp; _downKi=ki; _downKd=kd; }

    float getCatchKp()  const { return _catchKp; }
    float getCatchKd()  const { return _catchKd; }
    float getUpKp()     const { return _upKp; }
    float getUpKi()     const { return _upKi; }
    float getUpKd()     const { return _upKd; }
    float getDownKp()   const { return _downKp; }
    float getDownKi()   const { return _downKi; }
    float getDownKd()   const { return _downKd; }

private:
    KY040*   _encoder;
    DRV8825* _stepper;
    ControllerState _state;

    // ---- Encoder state ----
    float _encoderCPR;
    int   _encoderDirection;
    long  _encoderZeroCount;
    bool  _encoderZeroSet;
    bool  _armCentreSet;
    float _pendulumAngleDeg;
    float _pendulumVelocityDegS;
    int   _lastPendulumDirection;
    long  _prevEncoderCount;
    long  _prevVelocityCount;
    unsigned long _prevAngleUs;
    unsigned long _prevVelocityUs;

    // ---- Calibration ----
    bool _calibrationActive;
    int  _calibrationIndex;
    long _calibrationCounts[5];
    static const char* _calibrationNames[5];

    // ---- Swing-up ----
    int           _swingPolarity;
    int           _swingTargetSide;
    int           _swingAmplitudeSteps;
    long          _swingTargetSteps;
    bool          _bottomCrossingArmed;
    bool          _autoReturnToSwingUp;
    unsigned long _lastSwingShiftMs;

    // ---- Capturing ----
    unsigned long _captureStartMs;
    unsigned long _captureStableStartMs;

    // ---- PID state ----
    float _pidIntegral;

    // ---- Timing ----
    unsigned long _prevControllerUs;
    unsigned long _prevPrintMs;

    // ---- Polarity ----
    int _upBalancePolarity;
    int _downBalancePolarity;

    // ---- Tunable gains ----
    float _catchKp, _catchKd;
    float _upKp, _upKi, _upKd;
    float _downKp, _downKi, _downKd;

    // ============================================================
    // Private helpers
    // ============================================================
    // Math
    float wrapAngleTo180(float a);
    float calculateUprightError(float a);
    float calculateDownwardError(float a);
    float clampF(float v, float lo, float hi);
    long  clampL(long  v, long  lo, long  hi);

    // Checks
    bool controllerIsActive();
    bool referencesAreReady();

    // Encoder
    void updateEncoderMeasurements();
    void resetVelocityEstimator();

    // Motor service
    void serviceMotor();
    void serviceSwingPositionMotor();
    void serviceRateControlledMotor(float maxSpeed);

    // State entry
    void enterSwingUpMode();
    void enterUprightCapture(bool returnToSwingUp);
    void enterUpBalancingMode();
    void enterDownStabilizingMode();
    void stopController(const char* msg);

    // Controller loops
    void runSwingUpController();
    void runCatchingController(float elapsedS);
    void runUpBalanceController(float elapsedS);
    void runDownStabilizingController(float elapsedS);

    // Calibration
    void startCalibration();
    void saveCalibrationPoint();
    void finishCalibration();

    // Setup commands
    void setDownwardZero();
    void setArmCentre();
    void startSwingUp();
    void startManualUpCapture();
    void startDownStabilization();
    void emergencyStop();
    void resetController();

    // Swing helpers
    void setSwingTargetSide(int side);
    void increaseSwingAmplitude();

    // Printing
    void printMeasurements();
    void printMenu();
};
