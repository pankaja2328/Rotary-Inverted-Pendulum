#pragma once

class WiFiService {
public:
  WiFiService(const char* ssid, const char* password);
  void begin();
  const char* getIPAddress() const;

private:
  const char* _ssid;
  const char* _password;
};
