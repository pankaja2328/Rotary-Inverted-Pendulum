#include "TimerService.h"

TimerService::TimerService() : _handle(nullptr), _running(false) {}

TimerService::~TimerService() {
  stop();
}

bool TimerService::begin(Callback cb, void* arg, uint64_t interval_us, const char* name) {
  if (_running) {
    stop();
  }

  esp_timer_create_args_t cfg = {};
  cfg.callback        = cb;
  cfg.arg             = arg;
  cfg.dispatch_method = ESP_TIMER_TASK;   // Runs from high-priority timer task (not ISR)
  cfg.name            = name;

  esp_err_t err = esp_timer_create(&cfg, &_handle);
  if (err != ESP_OK) {
    Serial.printf("[TimerService] esp_timer_create failed: %s\n", esp_err_to_name(err));
    return false;
  }

  err = esp_timer_start_periodic(_handle, interval_us);
  if (err != ESP_OK) {
    Serial.printf("[TimerService] esp_timer_start_periodic failed: %s\n", esp_err_to_name(err));
    esp_timer_delete(_handle);
    _handle = nullptr;
    return false;
  }

  _running = true;
  Serial.printf("[TimerService] '%s' started at %llu us interval.\n", name, interval_us);
  return true;
}

void TimerService::stop() {
  if (_handle != nullptr) {
    if (_running) {
      esp_timer_stop(_handle);
    }
    esp_timer_delete(_handle);
    _handle  = nullptr;
    _running = false;
  }
}
