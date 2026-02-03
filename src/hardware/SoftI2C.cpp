#include "SoftI2C.h"

SoftI2C::SoftI2C(uint8_t sda, uint8_t scl)
    : _sda(sda), _scl(scl), _frequency(100000), _rxIndex(0), _rxLength(0)
{
}

bool SoftI2C::begin(uint32_t frequency) {
    _frequency = frequency;


    _delayUs = (1000000 / frequency) / 2;

    pinMode(_sda, OUTPUT_OPEN_DRAIN);
    pinMode(_scl, OUTPUT_OPEN_DRAIN);

    sdaHigh();
    sclHigh();
    delayMicroseconds(10);

    return true;
}

void SoftI2C::sdaLow() {
    digitalWrite(_sda, LOW);
}

void SoftI2C::sdaHigh() {
    digitalWrite(_sda, HIGH);
}

void SoftI2C::sclLow() {
    digitalWrite(_scl, LOW);
}

void SoftI2C::sclHigh() {
    digitalWrite(_scl, HIGH);

    uint32_t timeout = micros() + 1000;
    while (digitalRead(_scl) == LOW && micros() < timeout);
}

bool SoftI2C::readSDA() {
    return digitalRead(_sda);
}

void SoftI2C::delayHalf() {
    delayMicroseconds(_delayUs);
}

void SoftI2C::startCondition() {
    sdaHigh();
    sclHigh();
    delayHalf();
    sdaLow();      // SCL yüksekken SDA düş = START
    delayHalf();
    sclLow();      // Veri yolunu sürmeye hazırlan
}

void SoftI2C::stopCondition() {
    sdaLow();
    delayHalf();
    sclHigh();
    delayHalf();
    sdaHigh();     // SCL yüksekken SDA yüksel = STOP
    delayHalf();
}

bool SoftI2C::writeByte(uint8_t data) {
    // 8 biti sırayla gönder (MSB önce)
    for (uint8_t i = 0; i < 8; i++) {
        if (data & 0x80) {
            sdaHigh();
        } else {
            sdaLow();
        }
        delayHalf();
        sclHigh();     // Yükselen kenar: slave veriyi örnekler
        delayHalf();
        sclLow();
        data <<= 1;    // Sonraki bit için kaydır
    }

    sdaHigh();         // SDA'yı serbest bırak (slave sürebilsin)
    delayHalf();
    sclHigh();
    delayHalf();
    bool ack = !readSDA();  // LOW = ACK, HIGH = NACK
    sclLow();

    return ack;
}

uint8_t SoftI2C::readByte(bool ack) {
    uint8_t data = 0;
    sdaHigh();         // SDA'yı serbest bırak (slave sürebilsin)

    // 8 biti sırayla oku (MSB önce)
    for (uint8_t i = 0; i < 8; i++) {
        delayHalf();
        sclHigh();     // Yükselen kenar: veriyi örnekle
        delayHalf();
        data <<= 1;
        if (readSDA()) {
            data |= 1;
        }
        sclLow();
    }

    // ACK veya NACK gönder
    if (ack) {
        sdaLow();      // ACK = daha fazla bayt isteniyor
    } else {
        sdaHigh();     // NACK = bu son bayt
    }
    delayHalf();
    sclHigh();
    delayHalf();
    sclLow();
    sdaHigh();         // SDA'yı serbest bırak

    return data;
}

bool SoftI2C::beginTransmission(uint8_t address) {
    startCondition();
    // Yazma adresi: 7-bit adres sola kaydır, R/W biti = 0 (yazma)
    return writeByte((address << 1) | 0);
}

bool SoftI2C::write(uint8_t data) {
    return writeByte(data);
}

bool SoftI2C::endTransmission() {
    stopCondition();
    return true;
}

uint8_t SoftI2C::requestFrom(uint8_t address, uint8_t count) {
    _rxLength = 0;
    _rxIndex = 0;

    startCondition();
    // Okuma adresi: 7-bit adres sola kaydır, R/W biti = 1 (okuma)
    if (!writeByte((address << 1) | 1)) {
        stopCondition();
        return 0;  // Cihaz yanıt vermedi
    }

    // İstenen sayıda bayt oku
    for (uint8_t i = 0; i < count; i++) {
        // Son bayt hariç tümü için ACK gönder
        bool ack = (i < count - 1);
        _rxBuffer[_rxLength++] = readByte(ack);
    }

    stopCondition();
    return _rxLength;
}

uint8_t SoftI2C::read() {
    if (_rxIndex < _rxLength) {
        return _rxBuffer[_rxIndex++];
    }
    return 0;
}

uint8_t SoftI2C::available() {
    return _rxLength - _rxIndex;
}
