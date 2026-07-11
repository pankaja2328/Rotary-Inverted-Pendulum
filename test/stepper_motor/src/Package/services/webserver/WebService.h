#pragma once
#include <Arduino.h>
#include <WebServer.h>
#include "../../devices/stepper/StepperMotor.h"

class WebService {
public:
  WebService(StepperMotor& stepper);
  void begin();
  void handle();

private:
  StepperMotor& _stepper;
  WebServer _server;

  void handleRoot();
  void handleControl();
  void handleStatus();
};
