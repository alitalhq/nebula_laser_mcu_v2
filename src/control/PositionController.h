#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include <Arduino.h>
#include "../utils/AtomicData.h"
#include "../config/ControlConfig.h"
#include "../utils/MathUtils.h"

class PositionController {
public:
    PositionController();

    bool begin(const PositionConfig &cfg);

    void update(
        float worldTargetPan,
        float worldTargetTilt,
        float currentWorldPan,
        float currentWorldTilt,
        float dt
    );

    float getVelocityCommandPan() const;

    float getVelocityCommandTilt() const;

    void getError(float &errorPan, float &errorTilt) const;

    void resetIntegral();

    void setConfig(const PositionConfig &cfg);


    void reset();

private:
    PositionConfig _cfg;

    float _integralPan;
    float _integralTilt;
    float _lastErrorPan;
    float _lastErrorTilt;

    float _errorPan;
    float _errorTilt;

    AtomicFloat _velCmdPan;
    AtomicFloat _velCmdTilt;


    float computePI(float error, float &integral, float kp, float ki, float iMax, float dt);

    bool isInDeadZone(float error) const;
};

#endif
