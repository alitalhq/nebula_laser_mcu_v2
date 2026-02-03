#include "EncoderFilter.h"

EncoderFilter::EncoderFilter()
    : _head(0)
    , _windowSize(5)//ortalama alınacak örnek sayısı 5 ideal
    , _sampleCount(0)
    , _lastRaw(0)
    , _lastFiltered(0)
    , _velocity(0)
    , _lastUnwrapped(0)
    , _wrapCount(0)
{
    for (uint8_t i = 0; i < MAX_WINDOW_SIZE; i++) {
        _samples[i] = 0;
        _timestamps[i] = 0;
    }
}


bool EncoderFilter::begin(uint8_t windowSize) {
    if (windowSize == 0 || windowSize > MAX_WINDOW_SIZE) {
        return false;
    }

    _windowSize = windowSize;
    reset();
    return true;
}

void EncoderFilter::addSample(float angle, uint32_t timestamp_us) {//buffera yeni encoder okuması ekler (0-360)
    if (!MathUtils::isValid(angle)) {
        return;
    }

    _lastRaw = angle;

    float unwrapped = unwrapAngle(angle);

    _samples[_head] = unwrapped;
    _timestamps[_head] = timestamp_us;

    if (_sampleCount >= _windowSize) {
        uint8_t tail = (_head + 1) % _windowSize;

        float angleDelta = _samples[_head] - _samples[tail];
        uint32_t timeDelta = _timestamps[_head] - _timestamps[tail];

        if (timeDelta > 0) {
            _velocity = (angleDelta * 1000000.0f) / timeDelta;
        }
    } else {
        _sampleCount++;
    }

    _head = (_head + 1) % _windowSize;

    _lastFiltered = computeAverage();
}

float EncoderFilter::getFiltered() const {
    float wrapped = fmodf(_lastFiltered, 360.0f);
    if (wrapped < 0.0f) wrapped += 360.0f;
    return wrapped;
}

float EncoderFilter::getVelocity() const {//tahmini açısal hız döndürür
    return _velocity;
}

void EncoderFilter::reset() {//sıfırlama
    _head = 0;
    _sampleCount = 0;
    _lastRaw = 0;
    _lastFiltered = 0;
    _velocity = 0;
    _lastUnwrapped = 0;
    _wrapCount = 0;

    for (uint8_t i = 0; i < MAX_WINDOW_SIZE; i++) {
        _samples[i] = 0;
        _timestamps[i] = 0;
    }
}

float EncoderFilter::computeAverage() {//bufferdaki verilerin ortalamasını alır
    if (_sampleCount == 0) return 0.0f;

    float sum = 0.0f;
    uint8_t count = (_sampleCount < _windowSize) ? _sampleCount : _windowSize;

    for (uint8_t i = 0; i < count; i++) {
        sum += _samples[i];
    }

    return sum / count;
}

float EncoderFilter::unwrapAngle(float angle) {// 0 360 derecelerini geçince oluşan açı atlamalarını algılar ve düzenler
    if (_sampleCount > 0) {
        float delta = angle - _lastRaw;

        if (delta > 180.0f) {
            _wrapCount--;
        } else if (delta < -180.0f) {
            _wrapCount++;
        }
    }

    float unwrapped = angle + (_wrapCount * 360.0f);
    _lastUnwrapped = unwrapped;

    return unwrapped;
}
