#include "PositionController.h"//dünya referansına göre hedef sapmalarını hesaplar. hedefi açı değil vektör olarak görür.

PositionController::PositionController()
    : _integralPan(0), _integralTilt(0)
    , _lastErrorPan(0), _lastErrorTilt(0)
    , _filteredDerivPan(0), _filteredDerivTilt(0)
    , _errorPan(0), _errorTilt(0)
{
}

bool PositionController::begin(const PositionConfig &cfg) {
    _cfg = cfg;
    reset();
    return true;
}

void PositionController::update(
    float worldTargetPan,
    float worldTargetTilt,
    float currentWorldPan,
    float currentWorldTilt,
    float dt
)
{

    if (dt <= 0 || dt > 0.02f) {
        return;
    }

    _errorPan  = MathUtils::angularError(worldTargetPan, currentWorldPan);
    _errorTilt = MathUtils::angularError(worldTargetTilt, currentWorldTilt);

    float effectiveErrorPan  = (fabsf(_errorPan)  < _cfg.deadzone_pan)  ? 0.0f : _errorPan;
    float effectiveErrorTilt = (fabsf(_errorTilt) < _cfg.deadzone_tilt) ? 0.0f : _errorTilt;

    if (effectiveErrorPan == 0.0f) {
        _integralPan = 0.0f;
        _filteredDerivPan = 0.0f;
    }
    if (effectiveErrorTilt == 0.0f) {
        _integralTilt = 0.0f;
        _filteredDerivTilt = 0.0f;
    }

    float velCmdPan = computePID(
        effectiveErrorPan,
        _integralPan, _lastErrorPan, _filteredDerivPan,
        _cfg.kp_pan, _cfg.ki_pan, _cfg.kd_pan, _cfg.i_max_pan,
        dt
    );

    float velCmdTilt = computePID(
        effectiveErrorTilt,
        _integralTilt, _lastErrorTilt, _filteredDerivTilt,
        _cfg.kp_tilt, _cfg.ki_tilt, _cfg.kd_tilt, _cfg.i_max_tilt,
        dt
    );

    _velCmdPan.store(velCmdPan);
    _velCmdTilt.store(velCmdTilt);
}

float PositionController::getVelocityCommandPan() const {
    return _velCmdPan.load();
}

float PositionController::getVelocityCommandTilt() const {
    return _velCmdTilt.load();
}

void PositionController::getError(float &errorPan, float &errorTilt) const {
    errorPan = _errorPan;
    errorTilt = _errorTilt;
}

void PositionController::resetIntegral() {
    _integralPan = 0.0f;
    _integralTilt = 0.0f;
}

void PositionController::setConfig(const PositionConfig &cfg) {
    _cfg = cfg;
}

void PositionController::setDeadzone(float pan, float tilt) {
    _cfg.deadzone_pan  = pan;
    _cfg.deadzone_tilt = tilt;
}

void PositionController::reset() {
    _integralPan = 0.0f;
    _integralTilt = 0.0f;
    _lastErrorPan = 0.0f;
    _lastErrorTilt = 0.0f;
    _filteredDerivPan = 0.0f;
    _filteredDerivTilt = 0.0f;
    _errorPan = 0.0f;
    _errorTilt = 0.0f;
    _velCmdPan.store(0.0f);
    _velCmdTilt.store(0.0f);
}

float PositionController::computePID(
    float error,
    float &integral,
    float &lastError,
    float &filteredDeriv,
    float kp, float ki, float kd,
    float iMax,
    float dt
)
{
    float pTerm = kp * error;

    integral += ki * error * dt;
    integral = MathUtils::clamp(integral, -iMax, iMax);

    // Türev: gürültü amplifikasyonunu önlemek için düşük geçiren filtreden geçirilir
    float rawDeriv = (error - lastError) / dt;
    const float derivAlpha = 0.15f;
    filteredDeriv = derivAlpha * rawDeriv + (1.0f - derivAlpha) * filteredDeriv;
    float dTerm = kd * filteredDeriv;

    lastError = error;

    return pTerm + integral + dTerm;
}

