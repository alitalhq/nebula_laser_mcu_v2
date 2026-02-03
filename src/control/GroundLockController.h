#ifndef GROUND_LOCK_CONTROLLER_H
#define GROUND_LOCK_CONTROLLER_H

#include <Arduino.h>
#include "../sensors/IMUFusion.h"
#include "GroundReferenceCalibration.h"

class GroundLockController {
public:
    GroundLockController();

    bool begin();

    void setGroundReference(const GroundReferenceCalibration &calib) {
        _groundRef = calib;
    }

    void computeGroundLockTarget(
        const IMUFusion::Orientation &bodyOri,
        float &targetPan,
        float &targetTilt
    );

    void setEnabled(bool enabled) { _enabled = enabled; }

    bool isEnabled() const { return _enabled; }

private:
    bool _enabled;
    GroundReferenceCalibration _groundRef;
};

#endif
