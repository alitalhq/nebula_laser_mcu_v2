#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <Arduino.h>
#include "SoftI2C.h"

class EncoderDriver {
public:
    enum MagnetStatus {
        MAGNET_NOT_DETECTED = 0,    // Mıknatıs algılanmadı
        MAGNET_TOO_WEAK     = 1,    // Mıknatıs çok zayıf (uzak veya küçük)
        MAGNET_GOOD         = 2,    // Mıknatıs uygun (doğru mesafe ve güç)
        MAGNET_TOO_STRONG   = 3     // Mıknatıs çok güçlü (çok yakın)
    };

    EncoderDriver();

    bool begin(SoftI2C &wire, uint8_t address = 0x36);

    bool readRawAngle(uint16_t &raw);

    float readAngleDegrees();

    bool detectMagnet();

    MagnetStatus getMagnetStatus();

    uint8_t getAGC();

    uint16_t getMagnitude();

    uint32_t getReadFailures() const { return _readFailures; }

    void resetFailureCount() { _readFailures = 0; }

private:
    SoftI2C *_wire;
    uint8_t _address;
    uint32_t _readFailures;

    // I2C okuma yardımcıları
    bool readRegister(uint8_t reg, uint8_t &value);
    bool readRegister16(uint8_t reg, uint16_t &value);

    // AS5600 Register adresleri
    static constexpr uint8_t REG_RAW_ANGLE_H = 0x0C;  // Ham açı yüksek bayt
    static constexpr uint8_t REG_RAW_ANGLE_L = 0x0D;  // Ham açı düşük bayt
    static constexpr uint8_t REG_STATUS      = 0x0B;  // Durum register'ı
    static constexpr uint8_t REG_AGC         = 0x1A;  // Otomatik kazanç kontrolü
    static constexpr uint8_t REG_MAGNITUDE_H = 0x1B;  // Büyüklük yüksek bayt
    static constexpr uint8_t REG_MAGNITUDE_L = 0x1C;  // Büyüklük düşük bayt
};

#endif
