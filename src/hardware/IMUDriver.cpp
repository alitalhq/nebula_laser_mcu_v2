#include "IMUDriver.h"
#include "../utils/MathUtils.h"

IMUDriver::IMUDriver()
    : _wire(nullptr)
    , _address(0x68)
    , _accelScale(1.0f)
    , _gyroScale(1.0f)
    , _gyroBiasX(0), _gyroBiasY(0), _gyroBiasZ(0)
    , _accelBiasX(0), _accelBiasY(0), _accelBiasZ(0)
    , _readFailures(0)
{
}

bool IMUDriver::begin(TwoWire &wire, uint8_t address) {//IMU'yu başlatır
    _wire = &wire;
    _address = address;

    uint8_t chipID = getChipID();//çip kontrolü
    if (chipID != CHIP_ID_BMI160) {
        Serial.printf("IMU @ 0x%02X: Yanlis cip ID 0x%02X (beklenen 0x%02X)\n",
                      _address, chipID, CHIP_ID_BMI160);
        return false;
    }

    writeRegister(REG_CMD, CMD_SOFT_RESET);//yazılımsal sıfırlama sensör sıfırlandı
    delay(100);

    writeRegister(REG_CMD, CMD_ACC_PM_NORMAL);//ivmeölçer aktifleşti
    delay(10);

    writeRegister(REG_CMD, CMD_GYR_PM_NORMAL);//jiroskop aktifleşti
    delay(100);

    Serial.printf("IMU @ 0x%02X: Baslatildi\n", _address);
    return true;
}

bool IMUDriver::configure(uint8_t accelRange, uint16_t gyroRange, uint16_t odr) {
    // İvmeölçer aralığını yapılandır
    if (!writeRegister(REG_ACC_RANGE, accelRange)) {
        Serial.println("IMU: Ivmeolcer aralik yapilandirmasi basarisiz");
        return false;
    }
    setAccelScale(accelRange);  // Dönüşüm katsayısını güncelle

    // Jiroskop aralığını yapılandır
    if (!writeRegister(REG_GYR_RANGE, gyroRange)) {
        Serial.println("IMU: Jiroskop aralik yapilandirmasi basarisiz");
        return false;
    }
    setGyroScale(gyroRange);  // Dönüşüm katsayısını güncelle

    // Çıkış veri hızını (ODR) yapılandır
    // İstenen Hz değerini register bitine çevir
    uint8_t odr_bits = 0;
    if (odr >= 1600) odr_bits = 0x0C;      // 1600 Hz
    else if (odr >= 800) odr_bits = 0x0B;  // 800 Hz
    else if (odr >= 400) odr_bits = 0x0A;  // 400 Hz
    else if (odr >= 200) odr_bits = 0x09;  // 200 Hz
    else odr_bits = 0x08;                   // 100 Hz

    // İvmeölçer ODR ve modunu ayarla
    // 0x20 biti normal bant genişliği modunu seçer
    if (!writeRegister(REG_ACC_CONF, odr_bits | 0x20)) {
        Serial.println("IMU: Ivmeolcer ODR yapilandirmasi basarisiz");
        return false;
    }

    // Jiroskop ODR ve modunu ayarla
    if (!writeRegister(REG_GYR_CONF, odr_bits | 0x20)) {
        Serial.println("IMU: Jiroskop ODR yapilandirmasi basarisiz");
        return false;
    }

    Serial.printf("IMU @ 0x%02X: Yapilandirildi (ODR=%d Hz)\n", _address, odr);
    return true;
}

bool IMUDriver::read(IMUData &data) {//IMU verilerini okur
    uint8_t buffer[12];

    // Veri düzeni: [gyro_x_l, gyro_x_h, gyro_y_l, gyro_y_h, gyro_z_l, gyro_z_h,
    //               accel_x_l, accel_x_h, accel_y_l, accel_y_h, accel_z_l, accel_z_h]
    if (!readRegisters(REG_DATA_0, buffer, 12)) {
        _readFailures++;
        data.valid = false;
        return false;
    }

    int16_t gyro_x_raw = (int16_t)(buffer[1] << 8 | buffer[0]);
    int16_t gyro_y_raw = (int16_t)(buffer[3] << 8 | buffer[2]);
    int16_t gyro_z_raw = (int16_t)(buffer[5] << 8 | buffer[4]);

    int16_t accel_x_raw = (int16_t)(buffer[7] << 8 | buffer[6]);
    int16_t accel_y_raw = (int16_t)(buffer[9] << 8 | buffer[8]);
    int16_t accel_z_raw = (int16_t)(buffer[11] << 8 | buffer[10]);

    // Fiziksel birimlere dönüştür ve sapma düzeltmesi uygula
    // Jiroskop: derece/saniye
    data.gyro_x = (gyro_x_raw * _gyroScale) - _gyroBiasX;
    data.gyro_y = (gyro_y_raw * _gyroScale) - _gyroBiasY;
    data.gyro_z = (gyro_z_raw * _gyroScale) - _gyroBiasZ;

    // İvmeölçer: m/s²
    data.accel_x = (accel_x_raw * _accelScale) - _accelBiasX;
    data.accel_y = (accel_y_raw * _accelScale) - _accelBiasY;
    data.accel_z = (accel_z_raw * _accelScale) - _accelBiasZ;

    // Zaman damgası ve geçerlilik bayrağı
    data.timestamp_us = micros();
    data.valid = true;

    return true;
}

bool IMUDriver::calibrateGyro(uint16_t samples) {//Jiroskop kalibrasyonu
    Serial.printf("Jiroskop kalibre ediliyor @ 0x%02X (%d ornek)...\n", _address, samples);

    float sum_x = 0, sum_y = 0, sum_z = 0;
    uint16_t valid_samples = 0;

    for (uint16_t i = 0; i < samples; i++) {
        IMUData data;
        if (read(data)) {
            sum_x += data.gyro_x + _gyroBiasX;
            sum_y += data.gyro_y + _gyroBiasY;
            sum_z += data.gyro_z + _gyroBiasZ;
            valid_samples++;
        }
        delay(1);

        if ((i + 1) % 100 == 0) {
            Serial.print(".");
        }
    }
    Serial.println();

    if (valid_samples < samples / 2) {
        Serial.println("Jiroskop kalibrasyonu basarisiz: cok fazla okuma hatasi");
        return false;
    }

    // Ortalama sapmayı hesapla
    _gyroBiasX = sum_x / valid_samples;
    _gyroBiasY = sum_y / valid_samples;
    _gyroBiasZ = sum_z / valid_samples;

    Serial.printf("Jiroskop sapmasi: [%.3f, %.3f, %.3f] derece/s\n",
                  _gyroBiasX, _gyroBiasY, _gyroBiasZ);

    return true;
}

void IMUDriver::setGyroBias(float bias_x, float bias_y, float bias_z) {//NVS'den alınan kalibrasyon değerlerini uygular
    _gyroBiasX = bias_x;
    _gyroBiasY = bias_y;
    _gyroBiasZ = bias_z;
}

void IMUDriver::getGyroBias(float &bias_x, float &bias_y, float &bias_z) const {//NVS kalibrasyon verilerini getirir
    bias_x = _gyroBiasX;
    bias_y = _gyroBiasY;
    bias_z = _gyroBiasZ;
}

uint8_t IMUDriver::getChipID() {//Çip kimliği okuma, doğrulama için
    uint8_t id = 0;
    readRegister(REG_CHIP_ID, id);
    return id;
}

bool IMUDriver::writeRegister(uint8_t reg, uint8_t value) {//I2C iletişim fonksiyonları
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->write(value);
    return (_wire->endTransmission() == 0);
}

bool IMUDriver::readRegister(uint8_t reg, uint8_t &value) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) {
        return false;
    }

    if (_wire->requestFrom(_address, (uint8_t)1) != 1) {
        return false;
    }

    value = _wire->read();
    return true;
}

bool IMUDriver::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t count) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) {
        return false;
    }

    if (_wire->requestFrom(_address, count) != count) {
        return false;
    }

    for (uint8_t i = 0; i < count; i++) {
        buffer[i] = _wire->read();
    }

    return true;
}

void IMUDriver::setAccelScale(uint8_t range) {//hamdan gerçek değerleri üretir
    // BMI160 ivmeölçer aralıkları: 0x03=±2g, 0x05=±4g, 0x08=±8g, 0x0C=±16g
    // Ölçek = (aralık × yerçekimi) / 16-bit maksimum değer
    switch (range) {
        case 0x03: _accelScale = 2.0f * 9.81f / 32768.0f; break;   // ±2g
        case 0x05: _accelScale = 4.0f * 9.81f / 32768.0f; break;   // ±4g
        case 0x08: _accelScale = 8.0f * 9.81f / 32768.0f; break;   // ±8g
        case 0x0C: _accelScale = 16.0f * 9.81f / 32768.0f; break;  // ±16g
        default:   _accelScale = 4.0f * 9.81f / 32768.0f; break;   // Varsayılan ±4g
    }
}

void IMUDriver::setGyroScale(uint16_t range) {
    // BMI160 jiroskop aralıkları: 0x00=±2000°/s, 0x01=±1000°/s, 0x02=±500°/s,
    //                             0x03=±250°/s, 0x04=±125°/s
    // Ölçek = aralık / 16-bit maksimum değer
    switch (range) {
        case 0x00: _gyroScale = 2000.0f / 32768.0f; break;  // ±2000°/s
        case 0x01: _gyroScale = 1000.0f / 32768.0f; break;  // ±1000°/s
        case 0x02: _gyroScale = 500.0f / 32768.0f; break;   // ±500°/s
        case 0x03: _gyroScale = 250.0f / 32768.0f; break;   // ±250°/s
        case 0x04: _gyroScale = 125.0f / 32768.0f; break;   // ±125°/s
        default:   _gyroScale = 500.0f / 32768.0f; break;   // Varsayılan ±500°/s
    }
}
