#include "StabilizationController.h"//dron titremelerini hesaplar ve telafi için komut üretir.
#include "../utils/MathUtils.h"

StabilizationController::StabilizationController()
    : _lastBodyYaw(0), _lastBodyPitch(0)
    , _lastHeadYaw(0), _lastHeadPitch(0)
    , _deltaIMUPan(0), _deltaIMUTilt(0)
    , _filteredGyroBodyPan(0), _filteredGyroBodyTilt(0)
    , _filteredGyroHeadPan(0), _filteredGyroHeadTilt(0)
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
    float dt
)
{

    if (dt <= 0 || dt > 0.01f) {
        return;
    }

    _deltaIMUPan  = MathUtils::angularError(bodyOri.yaw, headOri.yaw);
    _deltaIMUTilt = MathUtils::angularError(bodyOri.pitch, headOri.pitch);

    float gyroBodyPan  = MathUtils::angularError(bodyOri.yaw, _lastBodyYaw) / dt;
    float gyroBodyTilt = MathUtils::angularError(bodyOri.pitch, _lastBodyPitch) / dt;
    float gyroHeadPan  = MathUtils::angularError(headOri.yaw, _lastHeadYaw) / dt;
    float gyroHeadTilt = MathUtils::angularError(headOri.pitch, _lastHeadPitch) / dt;

    _lastBodyYaw = bodyOri.yaw;
    _lastBodyPitch = bodyOri.pitch;
    _lastHeadYaw = headOri.yaw;
    _lastHeadPitch = headOri.pitch;

    const float gyroAlpha = 0.3f;
    _filteredGyroBodyPan  = lowPassFilter(_filteredGyroBodyPan, gyroBodyPan, gyroAlpha);
    _filteredGyroBodyTilt = lowPassFilter(_filteredGyroBodyTilt, gyroBodyTilt, gyroAlpha);
    _filteredGyroHeadPan  = lowPassFilter(_filteredGyroHeadPan, gyroHeadPan, gyroAlpha);
    _filteredGyroHeadTilt = lowPassFilter(_filteredGyroHeadTilt, gyroHeadTilt, gyroAlpha);

    float ffPan  = computeFeedForward(_filteredGyroBodyPan, _filteredGyroHeadPan,
                                      _cfg.k_feedforward_pan);
    float ffTilt = computeFeedForward(_filteredGyroBodyTilt, _filteredGyroHeadTilt,
                                      _cfg.k_feedforward_tilt);

    float dampingPan  = computeDamping(encoderVelPan, _cfg.k_damping_pan);
    float dampingTilt = computeDamping(encoderVelTilt, _cfg.k_damping_tilt);

    float velCmdPan  = ffPan + dampingPan;
    float velCmdTilt = ffTilt + dampingTilt;

    _velCmdPan.store(velCmdPan);
    _velCmdTilt.store(velCmdTilt);
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
    _lastBodyYaw = 0;
    _lastBodyPitch = 0;
    _lastHeadYaw = 0;
    _lastHeadPitch = 0;
    _deltaIMUPan = 0;
    _deltaIMUTilt = 0;
    _filteredGyroBodyPan = 0;
    _filteredGyroBodyTilt = 0;
    _filteredGyroHeadPan = 0;
    _filteredGyroHeadTilt = 0;
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
