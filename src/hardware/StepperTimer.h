#ifndef STEPPER_TIMER_H
#define STEPPER_TIMER_H

#include <Arduino.h>
#include "../config/HardwareConfig.h"

class StepperTimer {
public:
    enum Axis {
        PAN = 0,    // Pan ekseni (yatay) - Timer 0
        TILT = 1    // Tilt ekseni (dikey) - Timer 1
    };

    StepperTimer();

    bool begin(Axis axis, uint8_t stepPin, uint8_t dirPin);

    void setFrequency(float stepsPerSecond);

    void setDirection(bool clockwise);

    void enable();

    void disable();

    int32_t getStepCount() const;

    void resetStepCount();

    uint32_t getISROverruns() const;

private:
    hw_timer_t *_timer;
    uint8_t _stepPin;
    uint8_t _dirPin;
    Axis _axis;

    volatile int32_t _stepCount;
    volatile uint32_t _isrOverruns;
    volatile bool _currentDirection;

    static void IRAM_ATTR timerISR_Pan();
    static void IRAM_ATTR timerISR_Tilt();

    static StepperTimer *_instances[2];
};

#endif
