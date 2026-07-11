#include "Package/devices/stepper/StepperMotor.h"
#include "Package/services/webserver/WebService.h"
#include "Package/services/wifi/WiFiService.h"
#include <Arduino.h>

const char *AP_SSID = "Stepper_Motor_Test";
const char *AP_PASSWORD = "12345678";

// DRV8825 stepper driver
const int STEP_PIN = 15;
const int DIR_PIN = 2;
const int EN_PIN = 4;

// Instances
StepperMotor stepper(STEP_PIN, DIR_PIN, EN_PIN);
WiFiService wifi(AP_SSID, AP_PASSWORD);
WebService web(stepper);

void setup() {
  Serial.begin(115200);

  stepper.begin();
  Serial.println(
      "[OK] Stepper motor driver initialized (coils initially disabled).");

  wifi.begin();
  Serial.printf("[WiFi] SSID: %s\n", AP_SSID);
  Serial.printf("[WiFi] Connect to: http://%s\n", wifi.getIPAddress().toString().c_str());

  web.begin();
  Serial.println("[HTTP] Web server started on port 80.");
}

void loop() {
  web.handle();

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    if (stepper.isEnabled()) {
      Serial.printf("Mode: %d | Pos: %ld | Speed: %.1f steps/s\n",
                    (int)stepper.getMode(), stepper.getCurrentPosition(),
                    stepper.getCurrentSpeed());
    } else {
      Serial.println("Driver status: DISABLED");
    }
  }
}
