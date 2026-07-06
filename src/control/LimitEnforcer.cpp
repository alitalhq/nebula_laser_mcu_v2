#include "LimitEnforcer.h"

LimitEnforcer::LimitEnforcer()
    : _panAtMin(false), _panAtMax(false)
    , _tiltAtMin(false), _tiltAtMax(false)
{
}

bool LimitEnforcer::begin(const LimitConfig &cfg) {
    _cfg = cfg;
    reset();
    return true;
}

void LimitEnforcer::enforce(
    float currentEncoderPan,
    float currentEncoderTilt,
    float &velocityPan,
    float &velocityTilt,
    float &worldTargetPan,
    float &worldTargetTilt,
    float deltaIMUPan,
    float deltaIMUTilt,
    float dt
)
{
    if (!_cfg.enforce_limits) {
        return;
    }

    // Home'dan açısal sapma — dairesel, wrap-around güvenli
    // Pozitif = saat yönünde, negatif = saat yönü tersi
    float devPan  = MathUtils::angularError(currentEncoderPan,  _cfg.pan_offset);
    float devTilt = MathUtils::angularError(currentEncoderTilt, _cfg.tilt_offset);

    // ── PAN ──────────────────────────────────────────────────────────────

    _panAtMin = (devPan <= -_cfg.pan_range);
    _panAtMax = (devPan >=  _cfg.pan_range);

    if (_panAtMin) {
        velocityPan = MathUtils::clamp(velocityPan, 0.0f, 1000.0f);   // sadece uzaklasmaya izin
    } else if (_panAtMax) {
        velocityPan = MathUtils::clamp(velocityPan, -1000.0f, 0.0f);  // sadece uzaklasmaya izin
    } else {
        float distToMin = devPan + _cfg.pan_range;   // 0 = min limitte, artar uzaklaştıkça
        float distToMax = _cfg.pan_range - devPan;   // 0 = max limitte, artar uzaklaştıkça

        if (distToMin < _cfg.soft_margin && velocityPan < 0) {
            float scale = MathUtils::clamp(distToMin / _cfg.soft_margin, 0.0f, 1.0f);
            velocityPan *= scale;
            if (velocityPan < 0) velocityPan = 0.0f;
        }
        if (distToMax < _cfg.soft_margin && velocityPan > 0) {
            float scale = MathUtils::clamp(distToMax / _cfg.soft_margin, 0.0f, 1.0f);
            velocityPan *= scale;
            if (velocityPan > 0) velocityPan = 0.0f;
        }
    }

    // ── TILT ─────────────────────────────────────────────────────────────

    _tiltAtMin = (devTilt <= -_cfg.tilt_range);
    _tiltAtMax = (devTilt >=  _cfg.tilt_range);

    if (_tiltAtMin) {
        velocityTilt = MathUtils::clamp(velocityTilt, 0.0f, 1000.0f);   // sadece uzaklasmaya izin
    } else if (_tiltAtMax) {
        velocityTilt = MathUtils::clamp(velocityTilt, -1000.0f, 0.0f);  // sadece uzaklasmaya izin
    } else {
        float distToMin = devTilt + _cfg.tilt_range;
        float distToMax = _cfg.tilt_range - devTilt;

        if (distToMin < _cfg.soft_margin && velocityTilt < 0) {
            float scale = MathUtils::clamp(distToMin / _cfg.soft_margin, 0.0f, 1.0f);
            velocityTilt *= scale;
            if (velocityTilt < 0) velocityTilt = 0.0f;
        }
        if (distToMax < _cfg.soft_margin && velocityTilt > 0) {
            float scale = MathUtils::clamp(distToMax / _cfg.soft_margin, 0.0f, 1.0f);
            velocityTilt *= scale;
            if (velocityTilt > 0) velocityTilt = 0.0f;
        }
    }
}

bool LimitEnforcer::isAtLimit(bool pan, bool positive) const {
    if (pan) return positive ? _panAtMax : _panAtMin;
    else     return positive ? _tiltAtMax : _tiltAtMin;
}

void LimitEnforcer::setConfig(const LimitConfig &cfg) {
    _cfg = cfg;
}

void LimitEnforcer::reset() {
    _panAtMin = _panAtMax = false;
    _tiltAtMin = _tiltAtMax = false;
}
