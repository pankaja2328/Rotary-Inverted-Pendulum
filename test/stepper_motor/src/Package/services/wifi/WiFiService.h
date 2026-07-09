#pragma once

/**
 * @file  WiFiService.h
 * @brief Manages the WiFi Access Point for ESP32.
 */
class WiFiService {
public:
  WiFiService(const char* ssid, const char* password);
  void begin();
  const char* getIPAddress() const;

private:
  const char* _ssid;
  const char* _password;
};
