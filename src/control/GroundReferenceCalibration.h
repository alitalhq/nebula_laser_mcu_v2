#ifndef GROUND_REFERENCE_CALIBRATION_H
#define GROUND_REFERENCE_CALIBRATION_H

#include <Arduino.h>
#include "../hardware/IMUDriver.h"
#include "../sensors/IMUFusion.h"

class GroundReferenceCalibration {
public:
    struct GroundReference {
        float ax, ay, az;  // Baslangictaki yercekimi vektoru (m/s²)
        float pitch;       // Duzken pitch acisi (yaklasik 0° olmali)
        float roll;        // Duzken roll acisi (yaklasik 0° olmali)
        bool calibrated;   // Kalibrasyon tamamlandi mi?
    };

    GroundReferenceCalibration();

    bool calibrate(IMUDriver &bodyIMU, uint16_t samples = 1000);

    void computeGroundLockAngles(
        const IMUFusion::Orientation &currentOri,
        float &targetPan,
        float &targetTilt
    );

    bool isCalibrated() const { return _reference.calibrated; }

    GroundReference getReference() const { return _reference; }

    void printCalibration() const;

private:
    GroundReference _reference;  // Kaydedilmis yer referansi
};

#endif
