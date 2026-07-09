#pragma once
#include <Arduino.h>
#include <esp_timer.h>

/**
 * @file  TimerService.h
 * @brief Wraps ESP32 esp_timer for precise, periodic hardware timer callbacks.
 *
 * ESP32 esp_timer callbacks run from a dedicated high-priority FreeRTOS task,
 * not from main loop, providing much more reliable and deterministic timing
 * than polling from loop().
 */
class TimerService {
public:
  using Callback = void (*)(void* arg);

  TimerService();
  ~TimerService();

  /**
   * @brief  Create and start a periodic timer.
   * @param  cb          Callback function to invoke on each tick.
   * @param  arg         User argument forwarded to the callback (e.g. `this` pointer).
   * @param  interval_us Period in microseconds.
   * @param  name        Optional descriptive name (for esp_timer diagnostics).
   * @return true on success.
   */
  bool begin(Callback cb, void* arg, uint64_t interval_us, const char* name = "TimerService");

  /**
   * @brief Stop and delete the timer.
   */
  void stop();

  bool isRunning() const { return _running; }

private:
  esp_timer_handle_t _handle;
  bool _running;
};
