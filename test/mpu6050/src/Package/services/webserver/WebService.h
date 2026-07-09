#pragma once
#include <WebServer.h>
#include "../../devices/mpu6050/MPU6050.h"

/**
 * @file  WebService.h
 * @brief Serves the 3D visualization page and JSON data API.
 */
class WebService {
public:
  WebService(MPU6050& mpu);
  void begin();
  void handle();

private:
  WebServer _server;
  MPU6050& _mpu;

  void _handleRoot();
  void _handleData();
  void _handleNotFound();
};
