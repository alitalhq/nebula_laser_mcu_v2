#include "SensorHealth.h"

SensorHealth::SensorHealth() {
    reset();
}

void SensorHealth::recordSuccess(Sensor s) {
    if (s >= SENSOR_COUNT) return;

    _sensors[s].successCount++;
    _sensors[s].consecutiveFailures = 0;
    _sensors[s].healthy = true;
}

void SensorHealth::recordFailure(Sensor s) {
    if (s >= SENSOR_COUNT) return;

    _sensors[s].failureCount++;
    _sensors[s].consecutiveFailures++;

    if (_sensors[s].consecutiveFailures >= FAILURE_THRESHOLD) {
        _sensors[s].healthy = false;
        Serial.printf("SENSOR SAGLIGI: %s SAGLIKSIZ olarak isaretlendi\n", getSensorName(s));
    }
}

bool SensorHealth::isSensorHealthy(Sensor s) const {
    if (s >= SENSOR_COUNT) return false;
    return _sensors[s].healthy;
}

float SensorHealth::getSensorReliability(Sensor s) const {
    if (s >= SENSOR_COUNT) return 0.0f;

    uint32_t total = _sensors[s].successCount + _sensors[s].failureCount;

    if (total == 0) return 1.0f;

    return (float)_sensors[s].successCount / total;
}

bool SensorHealth::shouldEnterSafeMode() const {

    if (!isSensorHealthy(PAN_ENCODER) || !isSensorHealthy(TILT_ENCODER)) {
        return true;
    }

    if (!isSensorHealthy(BODY_IMU) && !isSensorHealthy(HEAD_IMU)) {
        return true;
    }

    if (!isSensorHealthy(MULTIPLEXER)) {
        return true;
    }

    return false;
}

void SensorHealth::reset() {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        resetSensor((Sensor)i);
    }
}

void SensorHealth::resetSensor(Sensor s) {
    if (s >= SENSOR_COUNT) return;

    _sensors[s].successCount = 0;
    _sensors[s].failureCount = 0;
    _sensors[s].consecutiveFailures = 0;
    _sensors[s].healthy = true;
}

const char* SensorHealth::getSensorName(Sensor s) const {
    switch (s) {
        case BODY_IMU:     return "Govde IMU";
        case HEAD_IMU:     return "Kafa IMU";
        case PAN_ENCODER:  return "Pan Encoder";
        case TILT_ENCODER: return "Tilt Encoder";
        case MULTIPLEXER:  return "I2C Multiplexer";
        default:           return "Bilinmeyen";
    }
}
