#ifndef STABILIZATION_CONTROLLER_H
#define STABILIZATION_CONTROLLER_H

#include <Arduino.h>
#include "../sensors/IMUFusion.h"
#include "../utils/AtomicData.h"
#include "../config/ControlConfig.h"

class StabilizationController {
public:

    StabilizationController();

    bool begin(const StabilizationConfig &cfg);

    void update(
        const IMUFusion::Orientation &bodyOri,
        const IMUFusion::Orientation &headOri,
        float encoderVelPan,
        float encoderVelTilt,
        float dt
    );

    float getVelocityCommandPan() const;

    float getVelocityCommandTilt() const;

    void getIMUDelta(float &deltaPan, float &deltaTilt) const;

    void setConfig(const StabilizationConfig &cfg);

    void reset();

private:
    StabilizationConfig _cfg;

    float _lastBodyYaw;
    float _lastBodyPitch;
    float _lastHeadYaw;
    float _lastHeadPitch;

    float _deltaIMUPan;
    float _deltaIMUTilt;

    float _filteredGyroBodyPan;
    float _filteredGyroBodyTilt;
    float _filteredGyroHeadPan;
    float _filteredGyroHeadTilt;

    AtomicFloat _velCmdPan;
    AtomicFloat _velCmdTilt;

    float computeFeedForward(float gyroBody, float gyroHead, float kFeedforward);

    float computeDamping(float encoderVel, float kDamping);

    float lowPassFilter(float current, float newSample, float alpha);
};

#endif
