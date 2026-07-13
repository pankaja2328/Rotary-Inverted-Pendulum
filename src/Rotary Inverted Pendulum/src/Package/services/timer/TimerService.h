#pragma once
#include <Arduino.h>
#include <esp_timer.h>

/**
 * @file  TimerService.h
 * @brief Wraps ESP32 esp_timer for precise, periodic hardware timer callbacks.
 */
class TimerService {
public:
  using Callback = void (*)(void* arg);

  TimerService();
  ~TimerService();

  bool begin(Callback cb, void* arg, uint64_t interval_us, const char* name = "TimerService");
  void stop();
  bool isRunning() const { return _running; }

private:
  esp_timer_handle_t _handle;
  bool _running;
};
