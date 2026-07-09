#include <Arduino.h>
#include "Package/devices/stepper/StepperMotor.h"
#include "Package/services/wifi/WiFiService.h"
#include "Package/services/webserver/WebService.h"

// WiFi Credentials for AP mode
const char* AP_SSID     = "Stepper_Motor_Test";
const char* AP_PASSWORD = "12345678";

// GPIO Pin definitions for A4988 / DRV8825 stepper driver
const int STEP_PIN = 14;
const int DIR_PIN  = 12;
const int EN_PIN   = 13;

// Instances
StepperMotor stepper(STEP_PIN, DIR_PIN, EN_PIN);
WiFiService wifi(AP_SSID, AP_PASSWORD);
WebService web(stepper);

void setup() {
  Serial.begin(115200);

  // Initialize Stepper Motor Driver
  stepper.begin();
  Serial.println("[OK] Stepper motor driver initialized (coils initially disabled).");

  // Initialize WiFi AP
  wifi.begin();
  Serial.printf("[WiFi] SSID: %s\n", AP_SSID);
  Serial.printf("[WiFi] Connect to: http://%s\n", wifi.getIPAddress());

  // Initialize WebServer
  web.begin();
  Serial.println("[HTTP] Web server started on port 80.");
}

void loop() {
  // Update web server to handle incoming requests
  web.handle();

  // NOTE: stepper.update() is no longer called here.
  // It is invoked automatically by the esp_timer hardware callback
  // configured in StepperMotor::begin() at a fixed 200 µs interval (5 kHz).

  // Print current status to Serial monitor at 5Hz
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    if (stepper.isEnabled()) {
      Serial.printf("Mode: %d | Pos: %ld | Speed: %.1f steps/s\n",
                    (int)stepper.getMode(),
                    stepper.getCurrentPosition(),
                    stepper.getCurrentSpeed());
    } else {
      Serial.println("Driver status: DISABLED");
    }
  }
}
