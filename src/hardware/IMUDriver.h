#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <Arduino.h>
#include <Wire.h>

class IMUDriver {
public:
    struct IMUData {
        float accel_x, accel_y, accel_z;    // İvme değerleri (m/s²)
        float gyro_x, gyro_y, gyro_z;        // Açısal hız değerleri (derece/s)
        uint32_t timestamp_us;               // Okuma zaman damgası (mikrosaniye)
        bool valid;                          // Okuma başarılı mı?
        IMUData() : accel_x(0), accel_y(0), accel_z(0),// varsayılan yapıcı metod
                    gyro_x(0), gyro_y(0), gyro_z(0),
                    timestamp_us(0), valid(false) {}
    };

    IMUDriver();

    bool begin(TwoWire &wire, uint8_t address = 0x68);

    bool configure(uint8_t accelRange, uint16_t gyroRange, uint16_t odr);

    bool read(IMUData &data);

    bool calibrateGyro(uint16_t samples = 1000);

    void setGyroBias(float bias_x, float bias_y, float bias_z);

    void getGyroBias(float &bias_x, float &bias_y, float &bias_z) const;

    uint8_t getChipID();

    uint32_t getReadFailures() const { return _readFailures; }

    void resetFailureCount() { _readFailures = 0; }

private:
    TwoWire *_wire;
    uint8_t _address;

    float _accelScale;       // Ham değerden m/s²'ye dönüşüm katsayısı
    float _gyroScale;        // Ham değerden derece/s'ye dönüşüm katsayısı

    float _gyroBiasX, _gyroBiasY, _gyroBiasZ;
    float _accelBiasX, _accelBiasY, _accelBiasZ;

    uint32_t _readFailures;

    // I2C yardımcı fonksiyonları
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t &value);
    bool readRegisters(uint8_t reg, uint8_t *buffer, uint8_t count);

    // Ölçek ayarlama fonksiyonları
    void setAccelScale(uint8_t range);
    void setGyroScale(uint16_t range);

    // BMI160 Register adresleri
    static constexpr uint8_t REG_CHIP_ID       = 0x00;  // Çip kimlik register'ı
    static constexpr uint8_t REG_DATA_0        = 0x04;  // Veri başlangıç register'ı
    static constexpr uint8_t REG_STATUS        = 0x1B;  // Durum register'ı
    static constexpr uint8_t REG_ACC_CONF      = 0x40;  // İvmeölçer yapılandırma
    static constexpr uint8_t REG_ACC_RANGE     = 0x41;  // İvmeölçer aralığı
    static constexpr uint8_t REG_GYR_CONF      = 0x42;  // Jiroskop yapılandırma
    static constexpr uint8_t REG_GYR_RANGE     = 0x43;  // Jiroskop aralığı
    static constexpr uint8_t REG_CMD           = 0x7E;  // Komut register'ı

    // BMI160 Komutları
    static constexpr uint8_t CMD_ACC_PM_NORMAL = 0x11;  // İvmeölçer normal güç modu
    static constexpr uint8_t CMD_GYR_PM_NORMAL = 0x15;  // Jiroskop normal güç modu
    static constexpr uint8_t CMD_SOFT_RESET    = 0xB6;  // Yazılımsal sıfırlama
    static constexpr uint8_t CHIP_ID_BMI160    = 0xD1;  // Beklenen çip kimliği
};

#endif
