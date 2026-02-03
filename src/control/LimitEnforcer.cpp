#include "LimitEnforcer.h"//hedef açıları gimbalın hareket sınırları içerisinde tutar
//güvenli bölgede tam yetki
//yumuşak limitte kontrol kademe kademe azaltılır
//sert limitte zorunlu durdurma

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

    //PAN EKSENİ İÇİN LİMİT KONTROLLERİ

    _panAtMin = false;
    _panAtMax = false;

    // Yumusak limit bolgesinde mi kontrol et (minimum limite yaklasma)
    if (currentEncoderPan <= _cfg.pan_min + _cfg.soft_margin) {
        // Minimum limite yaklasiliyor

        // Olcek faktoru hesapla (1.0 → 0.0)
        float scale = computeScaleFactor(
            currentEncoderPan,
            _cfg.pan_min,
            _cfg.soft_margin,
            velocityPan < 0  // Negatif hiz = limite yaklasma
        );

        // Pozisyon kontrolunu olceklendir
        velocityPan *= scale;

        // Limite dogru hareketi engelle
        if (velocityPan < 0) {
            velocityPan = 0;
        }
    }

    // Yumusak limit bolgesinde mi kontrol et (maksimum limite yaklasma)
    if (currentEncoderPan >= _cfg.pan_max - _cfg.soft_margin) {
        // Maksimum limite yaklasiliyor

        float scale = computeScaleFactor(
            currentEncoderPan,
            _cfg.pan_max,
            _cfg.soft_margin,
            velocityPan > 0  // Pozitif hiz = limite yaklasma
        );

        velocityPan *= scale;

        // Limite dogru hareketi engelle
        if (velocityPan > 0) {
            velocityPan = 0;
        }
    }

    // Sert limit kontrolu (minimum)
    if (currentEncoderPan <= _cfg.pan_min) {
        _panAtMin = true;  // Pan minimum limite ulasti
        // Sadece pozitif harekete (limitten uzaklasma) izin ver
        velocityPan = MathUtils::clamp(velocityPan, 0, 1000);

        // Dunya hedefini ulasilabilir degere ayarla
        // IMU deltasi eklenir cunku dunya referansi encoder + IMU
        worldTargetPan = _cfg.pan_min + deltaIMUPan;
    }

    // Sert limit kontrolu (maksimum)
    if (currentEncoderPan >= _cfg.pan_max) {
        _panAtMax = true;  // Pan maksimum limite ulasti
        // Sadece negatif harekete (limitten uzaklasma) izin ver
        velocityPan = MathUtils::clamp(velocityPan, -1000, 0);

        worldTargetPan = _cfg.pan_max + deltaIMUPan;
    }

    //TILT EKSENİ İÇİN LİMİT KONTROLLERİ

    _tiltAtMin = false;
    _tiltAtMax = false;

    if (currentEncoderTilt <= _cfg.tilt_min + _cfg.soft_margin) {
        float scale = computeScaleFactor(
            currentEncoderTilt,
            _cfg.tilt_min,
            _cfg.soft_margin,
            velocityTilt < 0
        );

        velocityTilt *= scale;

        if (velocityTilt < 0) {
            velocityTilt = 0;
        }
    }

    if (currentEncoderTilt >= _cfg.tilt_max - _cfg.soft_margin) {
        float scale = computeScaleFactor(
            currentEncoderTilt,
            _cfg.tilt_max,
            _cfg.soft_margin,
            velocityTilt > 0
        );

        velocityTilt *= scale;

        if (velocityTilt > 0) {
            velocityTilt = 0;
        }
    }

    if (currentEncoderTilt <= _cfg.tilt_min) {
        _tiltAtMin = true;
        velocityTilt = MathUtils::clamp(velocityTilt, 0, 1000);
        worldTargetTilt = _cfg.tilt_min + deltaIMUTilt;
    }

    if (currentEncoderTilt >= _cfg.tilt_max) {
        _tiltAtMax = true;
        velocityTilt = MathUtils::clamp(velocityTilt, -1000, 0);
        worldTargetTilt = _cfg.tilt_max + deltaIMUTilt;
    }
}

bool LimitEnforcer::isAtLimit(bool pan, bool positive) const {
    if (pan) {
        return positive ? _panAtMax : _panAtMin;
    } else {
        return positive ? _tiltAtMax : _tiltAtMin;
    }
}

void LimitEnforcer::setConfig(const LimitConfig &cfg) {
    _cfg = cfg;
}

void LimitEnforcer::reset() {
    _panAtMin = false;
    _panAtMax = false;
    _tiltAtMin = false;
    _tiltAtMax = false;
}

float LimitEnforcer::computeScaleFactor(float current, float limitValue, float margin, bool approaching) {
    if (!approaching) {
        return 1.0f;
    }

    float distanceToLimit = fabsf(current - limitValue);

    float scale = distanceToLimit / margin;
    return MathUtils::clamp(scale, 0.0f, 1.0f);
}

bool LimitEnforcer::isApproachingLimit(float velocity, bool positiveLimit) {
    if (positiveLimit) {
        return velocity > 0;
    } else {
        return velocity < 0;
    }
}
