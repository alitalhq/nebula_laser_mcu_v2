#ifndef SENSOR_CALIBRATION_H
#define SENSOR_CALIBRATION_H

#include <Arduino.h>
#include <Preferences.h>  // ESP32 NVS (Non-Volatile Storage)

class SensorCalibration {
public:

    struct CalibrationData {

        float body_gyro_bias_x;
        float body_gyro_bias_y;
        float body_gyro_bias_z;

        float head_gyro_bias_x;
        float head_gyro_bias_y;
        float head_gyro_bias_z;

        float pan_encoder_zero;
        float tilt_encoder_zero;

        float pan_min;
        float pan_max;
        float tilt_min;
        float tilt_max;

        uint32_t timestamp;
        uint8_t version;

        uint32_t crc;

        CalibrationData() {
            body_gyro_bias_x = 0; body_gyro_bias_y = 0; body_gyro_bias_z = 0;
            head_gyro_bias_x = 0; head_gyro_bias_y = 0; head_gyro_bias_z = 0;
            pan_encoder_zero = 0; tilt_encoder_zero = 0;
            pan_min = -30; pan_max = 30;
            tilt_min = -20; tilt_max = 20;
            timestamp = 0;
            version = 1;
            crc = 0;
        }
    };

    SensorCalibration();

    bool loadFromNVS();

    bool saveToNVS(const CalibrationData &data);

    bool isValid() const { return _valid; }

    CalibrationData getData() const { return _data; }

    void setData(const CalibrationData &data);

    bool eraseCalibration();

    void printCalibration() const;

private:
    CalibrationData _data;
    bool _valid;

    Preferences _prefs;

    uint32_t calculateCRC(const CalibrationData &data) const;
    bool verifyCRC(const CalibrationData &data) const;
};

#endif
