#ifndef COMMAND_COMBINER_H
#define COMMAND_COMBINER_H

#include <Arduino.h>
#include "../config/ControlConfig.h"
#include "../utils/MathUtils.h"

class CommandCombiner {
public:
    CommandCombiner();

    bool begin(const CombinerConfig &cfg);

    void update(
        float posVelPan,
        float posVelTilt,
        float stabVelPan,
        float stabVelTilt,
        float ffVelPan,
        float ffVelTilt,
        float dt
    );

    float getFinalVelocityPan() const { return _finalVelPan; }

    float getFinalVelocityTilt() const { return _finalVelTilt; }

    void setConfig(const CombinerConfig &cfg);

    void reset();

private:
    CombinerConfig _cfg;

    float _prevVelPan;
    float _prevVelTilt;

    float _finalVelPan;
    float _finalVelTilt;

    float limitAcceleration(float desired, float previous, float maxAccel, float dt);
};

#endif
