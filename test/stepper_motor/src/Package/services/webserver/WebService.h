#pragma once
#include <WebServer.h>
#include "../../devices/stepper/StepperMotor.h"

/**
 * @file  WebService.h
 * @brief Serves the dashboard page, JSON status API, and receives control endpoints.
 */
class WebService {
public:
  WebService(StepperMotor& stepper);
  void begin();
  void handle();

private:
  WebServer _server;
  StepperMotor& _stepper;

  void _handleRoot();
  void _handleData();
  void _handleControl();
  void _handleNotFound();
};
