#include "WebService.h"
#include "html/index_html.h"

WebService::WebService(MPU6050& mpu) : _server(80), _mpu(mpu) {}

void WebService::begin() {
  _server.on("/", [this]() { this->_handleRoot(); });
  _server.on("/data", [this]() { this->_handleData(); });
  _server.onNotFound([this]() { this->_handleNotFound(); });
  _server.begin();
}

void WebService::handle() {
  _server.handleClient();
}

void WebService::_handleRoot() {
  _server.send_P(200, "text/html", HTML_PAGE);
}

void WebService::_handleData() {
  char json[80];
  snprintf(json, sizeof(json),
    "{\"pitch\":%.2f,\"roll\":%.2f,\"yaw\":%.2f}",
    _mpu.getPitch(), _mpu.getRoll(), _mpu.getYaw());
  _server.send(200, "application/json", json);
}

void WebService::_handleNotFound() {
  _server.send(404, "text/plain", "404 Not Found");
}
