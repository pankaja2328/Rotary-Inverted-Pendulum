#include "WiFiService.h"
#include <WiFi.h>

WiFiService::WiFiService(const char* ssid, const char* password)
  : _ssid(ssid), _password(password) {}

void WiFiService::begin() {
  WiFi.softAP(_ssid, _password);
}

const char* WiFiService::getIPAddress() const {
  static char ipStr[16];
  strncpy(ipStr, WiFi.softAPIP().toString().c_str(), sizeof(ipStr));
  return ipStr;
}
