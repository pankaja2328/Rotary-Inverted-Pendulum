#include <Arduino.h>
#include "Package/devices/encoder/KY040.h"
#include "Package/devices/stepper/DRV8825.h"
#include "Package/controller/pendulum/PendulumController.h"
#include "Package/services/wifi/WiFiService.h"
#include "Package/services/webserver/WebService.h"

// WiFi configurations
const char *AP_SSID = "Rotary_Inverted_Pendulum";
const char *AP_PASSWORD = "12345678";

// Pin configurations matching verified working code
const int ENCODER_CLK_PIN = 32;
const int ENCODER_DT_PIN  = 33;

const int STEP_PIN   = 26;
const int DIR_PIN    = 27;
const int ENABLE_PIN = 25;

// Instances
KY040 encoder(ENCODER_CLK_PIN, ENCODER_DT_PIN);
DRV8825 stepper(STEP_PIN, DIR_PIN, ENABLE_PIN);
PendulumController controller;

WiFiService wifi(AP_SSID, AP_PASSWORD);
WebService web(controller);

void processSerialCommands() {
    while (Serial.available() > 0) {
        char command = Serial.read();
        if (command == '\n' || command == '\r') {
            continue;
        }
        controller.processCommand(command);
    }
}

void setup() {
    Serial.begin(115200);

    encoder.begin();
    stepper.begin();
    controller.begin(encoder, stepper);

    wifi.begin();
    Serial.printf("[WiFi] SSID: %s\n", AP_SSID);
    Serial.printf("[WiFi] Connect to: http://%s\n", wifi.getIPAddress());

    web.begin();
    Serial.println("[HTTP] Web server started on port 80.");

    Serial.println();
    Serial.println("ROTARY INVERTED PENDULUM CONTROLLER");
    Serial.println("Swing-up + upright catching + UP/DOWN PID");
    Serial.println("-----------------------------------------");
    Serial.print("Maximum swing side-to-side angle: ");
    Serial.print(2 * 45 * 1.8f, 1);
    Serial.println(" degrees");
    Serial.print("Soft PID arm limit: +/-");
    Serial.print(65 * 1.8f, 1);
    Serial.println(" degrees");
    Serial.print("Hard total arm movement: ");
    Serial.print(2 * 75 * 1.8f, 1);
    Serial.println(" degrees");
    Serial.print("DOWN-STABLE motor-arm limit: +/-");
    Serial.print(66 * 1.8f, 1);
    Serial.println(" degrees");

    controller.processCommand('M'); // Print menu
}

void loop() {
    controller.update();
    web.handle();
    processSerialCommands();
}
