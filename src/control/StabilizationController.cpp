#include "StabilizationController.h"//dron titremelerini hesaplar ve telafi için komut üretir.
#include "../utils/MathUtils.h"

StabilizationController::StabilizationController()
    : _deltaIMUPan(0), _deltaIMUTilt(0)
    , _filteredGyroBodyPan(0), _filteredGyroBodyTilt(0)
{

}

bool StabilizationController::begin(const StabilizationConfig &cfg) {
    _cfg = cfg;
    reset();
    return true;
}

void StabilizationController::update(
    const IMUFusion::Orientation &bodyOri,
    const IMUFusion::Orientation &headOri,
    float encoderVelPan,
    float encoderVelTilt,
    float bodyGyroPan,
    float bodyGyroTilt,
    float dt
)
{
    if (dt <= 0 || dt > 0.01f) {
        return;
    }

    _deltaIMUPan  = MathUtils::angularError(bodyOri.yaw, headOri.yaw);
    _deltaIMUTilt = MathUtils::angularError(bodyOri.pitch, headOri.pitch);

    const float gyroAlpha = 0.3f;

    // Pan feedforward: body roll rate direkt (işaret: ters yön)
    _filteredGyroBodyPan  = lowPassFilter(_filteredGyroBodyPan,  bodyGyroPan,  gyroAlpha);
    // Tilt feedforward: body pitch rate direkt (işaret: aynı yön — tilt mekaniği)
    _filteredGyroBodyTilt = lowPassFilter(_filteredGyroBodyTilt, bodyGyroTilt, gyroAlpha);

    float ffPan  = -_cfg.k_feedforward_pan  * _filteredGyroBodyPan;
    float ffTilt = -_cfg.k_feedforward_tilt * _filteredGyroBodyTilt;

    float dampingPan  = computeDamping(encoderVelPan,  _cfg.k_damping_pan);
    float dampingTilt = computeDamping(encoderVelTilt, _cfg.k_damping_tilt);

    _velCmdPan.store(ffPan  + dampingPan);
    _velCmdTilt.store(ffTilt + dampingTilt);
}

float StabilizationController::getVelocityCommandPan() const {
    return _velCmdPan.load();
}

float StabilizationController::getVelocityCommandTilt() const {
    return _velCmdTilt.load();
}

void StabilizationController::getIMUDelta(float &deltaPan, float &deltaTilt) const {
    deltaPan = _deltaIMUPan;
    deltaTilt = _deltaIMUTilt;
}

void StabilizationController::setConfig(const StabilizationConfig &cfg) {
    _cfg = cfg;
}

void StabilizationController::reset() {
    _deltaIMUPan = 0;
    _deltaIMUTilt = 0;
    _filteredGyroBodyPan = 0;
    _filteredGyroBodyTilt = 0;
    _velCmdPan.store(0);
    _velCmdTilt.store(0);
}

float StabilizationController::computeFeedForward(float gyroBody, float gyroHead, float kFeedforward) {
    float disturbanceVel = gyroBody - gyroHead;
    return kFeedforward * disturbanceVel;
}

float StabilizationController::computeDamping(float encoderVel, float kDamping) {
    return -kDamping * encoderVel;
}

float StabilizationController::lowPassFilter(float current, float newSample, float alpha) {
    return alpha * newSample + (1.0f - alpha) * current;
}
