#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include <Arduino.h>
#include "IMUDriver.h"
#include "../utils/MathUtils.h"

class IMUFusion {
public:
    struct Orientation {
        float roll;   // X ekseni etrafında dönüş (derece) - yan yatma
        float pitch;  // Y ekseni etrafında dönüş (derece) - öne/arkaya eğim
        float yaw;    // Z ekseni etrafında dönüş (derece) - yönelim
        uint32_t timestamp_us;  // Zaman damgası

        Orientation() : roll(0), pitch(0), yaw(0), timestamp_us(0) {}
    };

    IMUFusion();

    bool begin(float alpha = 0.98f);

    void update(const IMUDriver::IMUData &imu, float dt);

    Orientation getOrientation() const;

    void getAngularVelocity(float &wx, float &wy, float &wz) const;

    void reset(const Orientation *initial = nullptr);

    void setAlpha(float alpha);

    float getAlpha() const { return _alpha; }

    bool hasConverged() const { return _sampleCount > 100; }

private:
    float _roll, _pitch, _yaw;
    float _lastGyroX, _lastGyroY, _lastGyroZ;

    float _alpha;

    uint32_t _sampleCount;

    float _accelXFiltered, _accelYFiltered, _accelZFiltered;

    float accelToRoll(float ax, float ay, float az);
    float accelToPitch(float ax, float ay, float az);

    void filterAccel(float ax, float ay, float az);
};

#endif
