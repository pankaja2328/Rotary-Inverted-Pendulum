#include "SerialInterface.h"
#include "Config.h"
#include "EncoderTracker.h"
#include "StepperFollower.h"

void printHelp() {
  Serial.println();
  Serial.println("Rotary Inverted Pendulum - Dual Mode Controller");
  Serial.println("---------------------------------------------------");
  Serial.println("E        : Enable HANGING Tracking mode");
  Serial.println("B        : Enable INVERTED Balancing mode");
  Serial.println("D        : Disable tracking");
  Serial.println("Z        : Calibrate current position as 0 degrees");
  Serial.println("HP<val>  : Set Hanging Kp, e.g. HP120.0");
  Serial.println("HI<val>  : Set Hanging Ki");
  Serial.println("HK<val>  : Set Hanging Kd");
  Serial.println("IP<val>  : Set Inverted Kp, e.g. IP45.0");
  Serial.println("II<val>  : Set Inverted Ki");
  Serial.println("IK<val>  : Set Inverted Kd");
  Serial.println("S        : Show current gains");
  Serial.println("H        : Show commands");
  Serial.println();
}

void printGains() {
  Serial.println("--- HANGING GAINS ---");
  Serial.print("Kp:"); Serial.print(HANGING_KP, 3);
  Serial.print("  Ki:"); Serial.print(HANGING_KI, 3);
  Serial.print("  Kd:"); Serial.println(HANGING_KD, 3);
  
  Serial.println("--- INVERTED GAINS ---");
  Serial.print("Kp:"); Serial.print(INVERTED_KP, 3);
  Serial.print("  Ki:"); Serial.print(INVERTED_KI, 3);
  Serial.print("  Kd:"); Serial.println(INVERTED_KD, 3);
}

void processSerialCommands() {
  while (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) continue;

    char command = toupper(input[0]);
    char subCommand = (input.length() > 1) ? toupper(input[1]) : '\0';

    if (command == 'E') {
      enableTracking(MODE_HANGING);
    } else if (command == 'B') {
      enableTracking(MODE_INVERTED);
    } else if (command == 'D') {
      disableTracking();
    } else if (command == 'Z') {
      zeroPosition();
    } else if (command == 'H' && subCommand == '\0') {
      printHelp();
    } else if (command == 'S') {
      printGains();
    } else if (command == 'H' && subCommand == 'P') {
      HANGING_KP = input.substring(2).toFloat();
      Serial.print("Hanging Kp set to "); Serial.println(HANGING_KP, 3);
    } else if (command == 'H' && subCommand == 'I') {
      HANGING_KI = input.substring(2).toFloat();
      Serial.print("Hanging Ki set to "); Serial.println(HANGING_KI, 3);
    } else if (command == 'H' && subCommand == 'K') {
      HANGING_KD = input.substring(2).toFloat();
      Serial.print("Hanging Kd set to "); Serial.println(HANGING_KD, 3);
    } else if (command == 'I' && subCommand == 'P') {
      INVERTED_KP = input.substring(2).toFloat();
      Serial.print("Inverted Kp set to "); Serial.println(INVERTED_KP, 3);
    } else if (command == 'I' && subCommand == 'I') {
      INVERTED_KI = input.substring(2).toFloat();
      Serial.print("Inverted Ki set to "); Serial.println(INVERTED_KI, 3);
    } else if (command == 'I' && subCommand == 'K') {
      INVERTED_KD = input.substring(2).toFloat();
      Serial.print("Inverted Kd set to "); Serial.println(INVERTED_KD, 3);
    }
  }
}

void printData() {
  float motorAngle = motorStepPosition * 360.0 / MOTOR_STEPS_PER_REV;
  long currentCount = readEncoderCount();

  Serial.print("EncCount:");      Serial.print(currentCount);         Serial.print('\t');
  Serial.print("PendAngle_deg:"); Serial.print(pendulumAngleDeg, 2); Serial.print('\t');
  Serial.print("Error_deg:");     Serial.print(pendulumErrorDeg, 2); Serial.print('\t');
  Serial.print("MotorAngle_deg:"); Serial.print(motorAngle, 1);       Serial.print('\t');
  Serial.print("Rate_sps:");      Serial.print(currentStepRate, 0);   Serial.print('\t');
  Serial.print("Dir:");           Serial.print(currentDirection);     Serial.print('\t');
  Serial.print("Tracking:");      Serial.println(trackingEnabled ? 1 : 0);
}
