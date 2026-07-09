#include <Arduino.h>
#include "Package/devices/mpu6050/MPU6050.h"
#include "Package/services/wifi/WiFiService.h"
#include "Package/services/webserver/WebService.h"

// WiFi Credentials
const char* AP_SSID     = "MPU6050_Sensor";
const char* AP_PASSWORD = "12345678";

// Instances
MPU6050 mpu(21, 22);
WiFiService wifi(AP_SSID, AP_PASSWORD);
WebService web(mpu);

void setup() {
  Serial.begin(115200);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("[ERR] MPU6050 not found! System halted.");
    while (1) delay(1000);
  }
  Serial.println("[OK] MPU6050 initialized.");

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

  // Update MPU6050 reading and filter calculations
  mpu.update();

  // Print angles to Serial monitor at 10Hz
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.printf("Pitch: %.2f | Roll: %.2f | Yaw: %.2f\n", 
                  mpu.getPitch(), mpu.getRoll(), mpu.getYaw());
  }
}
