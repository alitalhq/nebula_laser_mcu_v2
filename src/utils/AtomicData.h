#ifndef ATOMIC_DATA_H
#define ATOMIC_DATA_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
// FreeRTOS ortamında task'ler ve ISR'ler arasında veri paylaşımını
// güvenli hale getirmek için kullanılır. Yarış durumlarını önler;
// küçük veriler için kritik bölge, yapılar için mutex tabanlı koruma sağlar.

/*
 * Kullanim:
 *   AtomicFloat position(0.0f);
 *   position.store(45.0f);  // Guvenli yazma
 *   float val = position.load();  // Guvenli okuma
 */
class AtomicFloat {
public:

    AtomicFloat(float initial = 0.0f) : _value(initial) {
        _mux = portMUX_INITIALIZER_UNLOCKED;
    }

    void store(float value) {
        portENTER_CRITICAL(&_mux);
        _value = value;
        portEXIT_CRITICAL(&_mux);
    }

    float load() const {
        portENTER_CRITICAL(&_mux);
        float val = _value;
        portEXIT_CRITICAL(&_mux);
        return val;
    }

    float add(float delta) {
        portENTER_CRITICAL(&_mux);
        _value += delta;
        float result = _value;
        portEXIT_CRITICAL(&_mux);
        return result;
    }

private:

    volatile float _value;

    mutable portMUX_TYPE _mux;
};


/*
 * Kullanim:
 *   struct MyData { float x, y; };
 *   AtomicStruct<MyData> data;
 *   MyData d; d.x = 1; d.y = 2;
 *   data.store(d);
 *   data.load(d);
 */
template<typename T>
class AtomicStruct {
public:
    AtomicStruct() {
        _mutex = xSemaphoreCreateMutex();
    }

    ~AtomicStruct() {
        if (_mutex) {
            vSemaphoreDelete(_mutex);
        }
    }

    bool store(const T &value, TickType_t timeout = portMAX_DELAY) {
        if (xSemaphoreTake(_mutex, timeout) == pdTRUE) {
            _value = value;
            xSemaphoreGive(_mutex);
            return true;
        }
        return false;
    }


    bool load(T &value, TickType_t timeout = portMAX_DELAY) const {
        if (xSemaphoreTake(_mutex, timeout) == pdTRUE) {
            value = _value;
            xSemaphoreGive(_mutex);
            return true;
        }
        return false;
    }

private:
    T _value;

    mutable SemaphoreHandle_t _mutex;
};

#endif
