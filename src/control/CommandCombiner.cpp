#include "CommandCombiner.h"//StabilizationController + PositionController + ROS2'den gelen komutları birleştirip asıl komutu üretir.

CommandCombiner::CommandCombiner()
    : _prevVelPan(0), _prevVelTilt(0)
    , _finalVelPan(0), _finalVelTilt(0)
{
}

bool CommandCombiner::begin(const CombinerConfig &cfg) {
    _cfg = cfg;
    reset();
    return true;
}

void CommandCombiner::update(
    float posVelPan,
    float posVelTilt,
    float stabVelPan,
    float stabVelTilt,
    float ffVelPan,
    float ffVelTilt,
    float dt
)
{
    if (dt <= 0 || dt > 0.02f) {
        return;
    }

    // Ham hiz = pozisyon + stabilizasyon + (ileri besleme × kazanc)
    float rawVelPan  = posVelPan + stabVelPan + (ffVelPan * _cfg.feedforward_gain);
    float rawVelTilt = posVelTilt + stabVelTilt + (ffVelTilt * _cfg.feedforward_gain);

    // Motorun maksimum hizini asmamak icin
    rawVelPan  = MathUtils::clamp(rawVelPan,  -_cfg.max_velocity_pan,  _cfg.max_velocity_pan);
    rawVelTilt = MathUtils::clamp(rawVelTilt, -_cfg.max_velocity_tilt, _cfg.max_velocity_tilt);

    // Yumusak gecisler saglar, adim kaybini onler
    _finalVelPan = limitAcceleration(
        rawVelPan,
        _prevVelPan,
        _cfg.max_acceleration_pan,
        dt
    );

    _finalVelTilt = limitAcceleration(
        rawVelTilt,
        _prevVelTilt,
        _cfg.max_acceleration_tilt,
        dt
    );

    _prevVelPan = _finalVelPan;
    _prevVelTilt = _finalVelTilt;
}

void CommandCombiner::setConfig(const CombinerConfig &cfg) {
    _cfg = cfg;
}

void CommandCombiner::reset() {
    _prevVelPan = 0.0f;
    _prevVelTilt = 0.0f;
    _finalVelPan = 0.0f;
    _finalVelTilt = 0.0f;
}

float CommandCombiner::limitAcceleration(float desired, float previous, float maxAccel, float dt) {
    // Bu zaman adiminda izin verilen maksimum hiz degisimi
    float maxDeltaV = maxAccel * dt;

    float deltaV = desired - previous;

    // Ivme limitine kilitle
    if (deltaV > maxDeltaV) {
        return previous + maxDeltaV;
    } else if (deltaV < -maxDeltaV) {
        return previous - maxDeltaV;
    } else {
        return desired;
    }
}
