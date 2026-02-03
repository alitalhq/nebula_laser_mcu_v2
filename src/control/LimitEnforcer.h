#ifndef LIMIT_ENFORCER_H
#define LIMIT_ENFORCER_H

#include <Arduino.h>
#include "../config/ControlConfig.h"
#include "../utils/MathUtils.h"

class LimitEnforcer {
public:

    LimitEnforcer();

    bool begin(const LimitConfig &cfg);

    void enforce(
        float currentEncoderPan,
        float currentEncoderTilt,
        float &velocityPan,
        float &velocityTilt,
        float &worldTargetPan,
        float &worldTargetTilt,
        float deltaIMUPan,
        float deltaIMUTilt,
        float dt
    );

    bool isAtLimit(bool pan, bool positive) const;

    void setConfig(const LimitConfig &cfg);

    void reset();

private:
    LimitConfig _cfg;

    bool _panAtMin, _panAtMax;
    bool _tiltAtMin, _tiltAtMax;

    float computeScaleFactor(
        float current,
        float limitValue,
        float margin,
        bool approaching
    );

    bool isApproachingLimit(
        float velocity,
        bool positiveLimit
    );
};

#endif
