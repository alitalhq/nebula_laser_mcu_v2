#include "IMUFusion.h"
#include <math.h>

//IMU sensör füzyonu için tamamlayıcı filtre
//açı = α × (açı + gyro × dt) + (1 - α) × accel_açısı

IMUFusion::IMUFusion()
    : _roll(0), _pitch(0), _yaw(0)
    , _lastGyroX(0), _lastGyroY(0), _lastGyroZ(0)
    , _alpha(0.98f)
    , _sampleCount(0)
    , _accelXFiltered(0), _accelYFiltered(0), _accelZFiltered(0)
{
}

bool IMUFusion::begin(float alpha) {
    _alpha = alpha;
    reset();
    return true;
}

void IMUFusion::update(const IMUDriver::IMUData &imu, float dt) {//filtre güncellemesi, imudan her veri okunduğunda filtre güncellenir
    if (!imu.valid || dt <= 0 || dt > 0.1f) {
        return;
    }

    _lastGyroX = imu.gyro_x;
    _lastGyroY = imu.gyro_y;
    _lastGyroZ = imu.gyro_z;

    float rollGyro  = _roll  + imu.gyro_x * dt;
    float pitchGyro = _pitch + imu.gyro_y * dt;
    float yawGyro   = _yaw   + imu.gyro_z * dt;

    filterAccel(imu.accel_x, imu.accel_y, imu.accel_z);

    float rollAccel  = accelToRoll(_accelXFiltered, _accelYFiltered, _accelZFiltered);
    float pitchAccel = accelToPitch(_accelXFiltered, _accelYFiltered, _accelZFiltered);

    _roll  = _alpha * rollGyro  + (1.0f - _alpha) * rollAccel;
    _pitch = _alpha * pitchGyro + (1.0f - _alpha) * pitchAccel;
    _yaw   = yawGyro;

    _roll  = MathUtils::wrapAngle180(_roll);
    _pitch = MathUtils::wrapAngle180(_pitch);
    _yaw   = MathUtils::wrapAngle360(_yaw);

    _sampleCount++;
}

IMUFusion::Orientation IMUFusion::getOrientation() const {
    Orientation ori;
    ori.roll = _roll;
    ori.pitch = _pitch;
    ori.yaw = _yaw;
    ori.timestamp_us = micros();
    return ori;
}

void IMUFusion::getAngularVelocity(float &wx, float &wy, float &wz) const {
    wx = _lastGyroX;
    wy = _lastGyroY;
    wz = _lastGyroZ;
}

void IMUFusion::reset(const Orientation *initial) {
    if (initial) {
        _roll = initial->roll;
        _pitch = initial->pitch;
        _yaw = initial->yaw;
    } else {
        _roll = 0.0f;
        _pitch = 0.0f;
        _yaw = 0.0f;
    }

    _lastGyroX = 0.0f;
    _lastGyroY = 0.0f;
    _lastGyroZ = 0.0f;

    _accelXFiltered = 0.0f;
    _accelYFiltered = 0.0f;
    _accelZFiltered = 9.81f;  // Hareketsiz başladığını varsay (yerçekimi Z ekseninde)

    _sampleCount = 0;
}

void IMUFusion::setAlpha(float alpha) {
    _alpha = MathUtils::clamp(alpha, 0.9f, 0.999f);
}

float IMUFusion::accelToRoll(float ax, float ay, float az) {//ivmeden roll (x ekseni) hesabı
    if (fabsf(az) < 0.1f && fabsf(ay) < 0.1f) {
        return 0.0f;
    }

    return MathUtils::radToDeg(atan2f(ay, az));
}

float IMUFusion::accelToPitch(float ax, float ay, float az) { // pitch y ekseni hesabı
    float sqrtYZ = sqrtf(ay * ay + az * az);

    if (sqrtYZ < 0.1f) {
        return 0.0f;
    }

    return MathUtils::radToDeg(atan2f(-ax, sqrtYZ));
}

void IMUFusion::filterAccel(float ax, float ay, float az) {// ivemölçer filtreleme (üstel hareketli ortalama)
    const float accelAlpha = 0.1f;

    if (_sampleCount == 0) {
        _accelXFiltered = ax;
        _accelYFiltered = ay;
        _accelZFiltered = az;
    } else {
        _accelXFiltered = accelAlpha * ax + (1.0f - accelAlpha) * _accelXFiltered;
        _accelYFiltered = accelAlpha * ay + (1.0f - accelAlpha) * _accelYFiltered;
        _accelZFiltered = accelAlpha * az + (1.0f - accelAlpha) * _accelZFiltered;
    }
}
