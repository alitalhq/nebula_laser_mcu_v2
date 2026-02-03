#ifndef SENSOR_HEALTH_H
#define SENSOR_HEALTH_H

#include <Arduino.h>

class SensorHealth {
public:
    enum Sensor {
        BODY_IMU = 0,       // Gövde IMU sensörü
        HEAD_IMU,           // Kafa IMU sensörü
        PAN_ENCODER,        // Pan ekseni encoder'ı
        TILT_ENCODER,       // Tilt ekseni encoder'ı
        MULTIPLEXER,        // I2C multiplexer (eski tasarımdan, şu an kullanılmıyor)
        SENSOR_COUNT        // Toplam sensör sayısı (enum sonlandırıcı)
    };

    SensorHealth();

    void recordSuccess(Sensor s);

    void recordFailure(Sensor s);

    bool isSensorHealthy(Sensor s) const;

    float getSensorReliability(Sensor s) const;

    bool shouldEnterSafeMode() const;

    void reset();

    void resetSensor(Sensor s);

    const char* getSensorName(Sensor s) const;

private:
    struct SensorStatus {
        uint32_t successCount;
        uint32_t failureCount;
        uint32_t consecutiveFailures;
        bool healthy;

        SensorStatus() : successCount(0), failureCount(0),
                        consecutiveFailures(0), healthy(true) {}
    };

    SensorStatus _sensors[SENSOR_COUNT];

    static constexpr uint32_t FAILURE_THRESHOLD = 10;

    static constexpr float MIN_RELIABILITY = 0.80f;
};

#endif
