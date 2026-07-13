#pragma once
#include <Arduino.h>
#include <WebServer.h>
#include "../../controller/pendulum/PendulumController.h"

class WebService {
public:
  WebService(PendulumController& controller);
  void begin();
  void handle();

private:
  PendulumController& _controller;
  WebServer _server;

  void handleRoot();
  void handleControl();
  void handleStatus();
};
