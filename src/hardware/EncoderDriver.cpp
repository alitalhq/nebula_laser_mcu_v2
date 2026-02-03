#include "EncoderDriver.h"

EncoderDriver::EncoderDriver()
    : _wire(nullptr)
    , _address(0x36)
    , _readFailures(0)
{
}

bool EncoderDriver::begin(SoftI2C &wire, uint8_t address) {//başlangıçta i2c bağlantısı ve mıknatıs durumu kontrol edilir
    _wire = &wire;
    _address = address;

    uint8_t status;
    if (!readRegister(REG_STATUS, status)) {
        Serial.printf("Encoder @ 0x%02X: Yanit yok\n", _address);
        return false;
    }

    MagnetStatus magStatus = getMagnetStatus();
    if (magStatus == MAGNET_GOOD) {
        Serial.printf("Encoder @ 0x%02X: Baslatildi (miknatis iyi)\n", _address);
    } else {
        Serial.printf(
            "Encoder @ 0x%02X: UYARI - Miknatis durumu = %d (2=iyi)\n",
            _address, magStatus
        );
    }

    return true;
}

bool EncoderDriver::readRawAngle(uint16_t &raw) {//açının ham değerini okur
    if (!readRegister16(REG_RAW_ANGLE_H, raw)) {
        _readFailures++;
        return false;
    }

    raw &= 0x0FFF;

    return true;
}

float EncoderDriver::readAngleDegrees() {//açı derece cinsinden okunur
    uint16_t raw;
    if (!readRawAngle(raw)) {
        return -1.0f;
    }

    // 360 derece / 4096 adım = 0.087890625 derece/adım
    return raw * 0.087890625f;
}


bool EncoderDriver::detectMagnet() {// mıknatısın algılanıp algılanmadığını döner
    uint8_t status;
    if (!readRegister(REG_STATUS, status)) {
        return false;
    }
    return (status & 0x20) != 0;
}

EncoderDriver::MagnetStatus EncoderDriver::getMagnetStatus() {// detaylı mıknatıs kontrolü
    uint8_t status;
    if (!readRegister(REG_STATUS, status)) {
        return MAGNET_NOT_DETECTED;
    }

    // Bit 4-3: ML (Çok Zayıf) ve MH (Çok Güçlü) bayrakları
    uint8_t magBits = (status >> 3) & 0x03;

    // Önce mıknatıs algılandı mı kontrol et
    if ((status & 0x20) == 0) {
        return MAGNET_NOT_DETECTED;
    }

    // Manyetik alan gücünü yorumla
    switch (magBits) {
        case 0: return MAGNET_NOT_DETECTED;  // MD bayrağı set değilse
        case 1: return MAGNET_TOO_WEAK;      // ML set (çok zayıf)
        case 2: return MAGNET_GOOD;          // Normal aralıkta
        case 3: return MAGNET_TOO_STRONG;    // MH set (çok güçlü)
        default: return MAGNET_NOT_DETECTED;
    }
}

uint8_t EncoderDriver::getAGC() {//Düşük değer = güçlü mıknatıs, yüksek değer = zayıf mıknatıs.İdeal aralık: 64-192.
    uint8_t agc;
    if (!readRegister(REG_AGC, agc)) {
        return 0;
    }
    return agc;
}

uint16_t EncoderDriver::getMagnitude() {// manyetik alan büyüklüğü
    uint16_t mag;
    if (!readRegister16(REG_MAGNITUDE_H, mag)) {
        return 0;
    }
    return mag & 0x0FFF;
}

bool EncoderDriver::readRegister(uint8_t reg, uint8_t &value) {//i2c ile register okuma
    _wire->beginTransmission(_address);
    _wire->write(reg);
    if (!_wire->endTransmission()) {
        return false;
    }
    
    if (_wire->requestFrom(_address, (uint8_t)1) != 1) {
        return false;
    }

    value = _wire->read();
    return true;
}

bool EncoderDriver::readRegister16(uint8_t reg, uint16_t &value) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    if (!_wire->endTransmission()) {
        return false;
    }

    // 2 bayt oku
    if (_wire->requestFrom(_address, (uint8_t)2) != 2) {
        return false;
    }

    uint8_t high = _wire->read();  // Yüksek bayt
    uint8_t low  = _wire->read();  // Düşük bayt
    value = (high << 8) | low;

    return true;
}
