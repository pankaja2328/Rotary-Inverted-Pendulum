#pragma once
#include <WebServer.h>
#include "../../devices/encoder/KY040.h"

class WebService {
public:
  WebService(KY040& encoder);
  void begin();
  void handle();

private:
  WebServer _server;
  KY040& _encoder;

  void _handleRoot();
  void _handleData();
  void _handleNotFound();
};
