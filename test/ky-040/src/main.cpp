#include "Package/devices/encoder/KY040.h"
#include "Package/services/webserver/WebService.h"
#include "Package/services/wifi/WiFiService.h"
#include <Arduino.h>

const int CLK_PIN = 18;
const int DT_PIN = 19;
const int SW_PIN = 21;

const char *AP_SSID = "KY040_Encoder_Test";
const char *AP_PASSWORD = "12345678";

// Instances
KY040 encoder(CLK_PIN, DT_PIN, SW_PIN);
WiFiService wifi(AP_SSID, AP_PASSWORD);
WebService web(encoder);

void setup() {
  Serial.begin(115200);

  encoder.begin();
  Serial.println("[OK] KY-040 Rotary Encoder Initialized.");

  wifi.begin();
  Serial.printf("[WiFi] SSID: %s\n", AP_SSID);
  Serial.printf("[WiFi] Connect to: http://%s\n", wifi.getIPAddress());
  web.begin();
  Serial.println("[HTTP] Web server started on port 80.");
}

void loop() {
  web.handle();

  if (encoder.isButtonPressed()) {
    Serial.println("Button Pressed! Resetting position to 0.");
    encoder.setPosition(0);
  }

  delay(10);
}
