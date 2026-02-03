#ifndef ENCODER_FILTER_H
#define ENCODER_FILTER_H

#include <Arduino.h>
#include "../utils/MathUtils.h"

class EncoderFilter {
public:
    EncoderFilter();

    bool begin(uint8_t windowSize = 5);

    void addSample(float angle, uint32_t timestamp_us);

    float getFiltered() const;

    float getVelocity() const;

    void reset();

    bool isReady() const { return _sampleCount >= _windowSize; }

    float getRaw() const { return _lastRaw; }

private:
    static constexpr uint8_t MAX_WINDOW_SIZE = 10;

    float _samples[MAX_WINDOW_SIZE];
    uint32_t _timestamps[MAX_WINDOW_SIZE];

    uint8_t _head;
    uint8_t _windowSize;
    uint8_t _sampleCount;

    float _lastRaw;
    float _lastFiltered;
    float _velocity;

    float _lastUnwrapped;
    int32_t _wrapCount;

    float computeAverage();
    float unwrapAngle(float angle);
};

#endif
