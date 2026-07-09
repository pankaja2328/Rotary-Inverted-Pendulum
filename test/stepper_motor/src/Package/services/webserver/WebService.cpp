#include "WebService.h"
#include "html/index_html.h"

WebService::WebService(StepperMotor& stepper) : _server(80), _stepper(stepper) {}

void WebService::begin() {
  _server.on("/", [this]() { this->_handleRoot(); });
  _server.on("/data", [this]() { this->_handleData(); });
  _server.on("/control", [this]() { this->_handleControl(); });
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
  char json[192];
  snprintf(json, sizeof(json),
    "{\"pos\":%ld,\"speed\":%.2f,\"en\":%d,\"mode\":%d,\"minPulse\":%.1f,\"startingPulse\":%.1f,\"accel\":%.1f}",
    _stepper.getCurrentPosition(),
    _stepper.getCurrentSpeed(),
    _stepper.isEnabled() ? 1 : 0,
    (int)_stepper.getMode(),
    _stepper.getMinPulse(),
    _stepper.getStartingPulse(),
    _stepper.getAcceleration());
  _server.send(200, "application/json", json);
}

void WebService::_handleControl() {
  if (_server.hasArg("en")) {
    bool en = _server.arg("en").toInt() > 0;
    _stepper.enable(en);
  }
  
  if (_server.hasArg("mode")) {
    int modeVal = _server.arg("mode").toInt();
    if (modeVal >= 0 && modeVal <= 2) {
      _stepper.setMode((StepperMotor::Mode)modeVal);
    }
  }

  if (_server.hasArg("pos")) {
    long pos = _server.arg("pos").toInt();
    _stepper.setTargetPosition(pos);
  }

  if (_server.hasArg("speed")) {
    float speed = _server.arg("speed").toFloat();
    _stepper.setTargetSpeed(speed);
  }

  if (_server.hasArg("accel")) {
    float accel = _server.arg("accel").toFloat();
    _stepper.setAcceleration(accel);
  }

  if (_server.hasArg("minPulse")) {
    float mp = _server.arg("minPulse").toFloat();
    _stepper.setMinPulse(mp);
  }

  if (_server.hasArg("startingPulse")) {
    float sp = _server.arg("startingPulse").toFloat();
    _stepper.setStartingPulse(sp);
  }

  if (_server.hasArg("sweepMin") && _server.hasArg("sweepMax")) {
    long sMin = _server.arg("sweepMin").toInt();
    long sMax = _server.arg("sweepMax").toInt();
    _stepper.setSweepLimits(sMin, sMax);
  }

  if (_server.hasArg("stop")) {
    _stepper.stop();
    _stepper.enable(false);
  }

  _server.send(200, "text/plain", "OK");
}

void WebService::_handleNotFound() {
  _server.send(404, "text/plain", "404 Not Found");
}
