#ifndef KY040_H
#define KY040_H

#include <Arduino.h>

class KY040 {
public:
    KY040(int clkPin, int dtPin, int swPin = -1);
    void begin();
    long getPosition() const;
    long getDelta();
    void setPosition(long position);
    bool isButtonPressed();
    unsigned long getInterruptCount() const;

private:
    int _clkPin;
    int _dtPin;
    int _swPin;

    volatile long _position;
    long _lastPosition;
    
    volatile uint8_t _lastPinState;
    volatile bool _buttonPressed;
    volatile unsigned long _lastButtonPressTime;
    volatile unsigned long _interruptCount;

    static void IRAM_ATTR isr(void* arg);
    void handleInterrupt();

    static void IRAM_ATTR buttonIsr(void* arg);
    void handleButtonInterrupt();
};

#endif // KY040_H
