#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include <Arduino.h>

class SoftI2C {
public:
    SoftI2C(uint8_t sda, uint8_t scl);

    bool begin(uint32_t frequency = 100000);

    bool beginTransmission(uint8_t address);

    bool write(uint8_t data);

    bool endTransmission();

    uint8_t requestFrom(uint8_t address, uint8_t count);

    uint8_t read();

    uint8_t available();

private:
    uint8_t _sda;               // SDA pin numarası
    uint8_t _scl;               // SCL pin numarası

    uint32_t _frequency;        // Yapılandırılmış frekans

    uint32_t _delayUs;          // Yarım periyot gecikme (mikrosaniye)

    uint8_t _rxBuffer[32];      // Alınan veriler için tampon

    uint8_t _rxIndex;           // Okuma indeksi

    uint8_t _rxLength;          // Tamponda bulunan veri uzunluğu

    void sdaLow();              // SDA'yı düşük çek
    void sdaHigh();             // SDA'yı serbest bırak (pull-up ile yüksek)
    void sclLow();              // SCL'yi düşük çek
    void sclHigh();             // SCL'yi serbest bırak (clock stretching için bekler)
    bool readSDA();             // SDA durumunu oku

    void startCondition();      // START koşulu (SDA düşerken SCL yüksek)
    void stopCondition();       // STOP koşulu (SDA yükselirken SCL yüksek)
    bool writeByte(uint8_t data);   // 8 bit + ACK oku
    uint8_t readByte(bool ack);     // 8 bit oku + ACK/NACK gönder

    void delayHalf();           // Yarım saat periyodu bekle
};

#endif
