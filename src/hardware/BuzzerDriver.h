#ifndef BUZZER_DRIVER_H
#define BUZZER_DRIVER_H

#include <Arduino.h>

class BuzzerDriver {
public:
    BuzzerDriver();

    bool begin(uint8_t pin = 39, uint8_t pwmChannel = 0, uint16_t frequency = 2000);

    void tone(uint16_t frequency = 0);

    void noTone();

    void beep(uint16_t duration_ms = 100);

    void beepPattern(uint8_t count, uint16_t beep_duration_ms = 100, uint16_t pause_duration_ms = 100);

    void calibrationWarning();

    void systemReady();

    void errorAlert();

    void modeChange();

    bool isActive() const { return _active; }

private:
    uint8_t _pin;
    uint8_t _pwmChannel;
    uint16_t _defaultFrequency;
    bool _active;
};

#endif
