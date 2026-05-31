#include "SensorCalibration.h"

static uint32_t crc32_byte(uint32_t crc, uint8_t byte) {
    crc ^= byte;
    for (int i = 0; i < 8; i++) {
        // Polinom: 0xEDB88320 (IEEE 802.3 CRC-32)
        crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320 : 0);
    }
    return crc;
}

SensorCalibration::SensorCalibration()
    : _valid(false)
{
}

bool SensorCalibration::loadFromNVS() {//NVS'den klibrasyon verilerini okur
    if (!_prefs.begin("gimbal", true)) {
        Serial.println("NVS namespace acilamadi");
        return false;
    }

    if (!_prefs.isKey("calib")) {
        _prefs.end();
        Serial.println("NVS'de kalibrasyon bulunamadi");
        return false;
    }

    size_t dataSize = _prefs.getBytesLength("calib");
    if (dataSize != sizeof(CalibrationData)) {
        _prefs.end();
        Serial.printf("Kalibrasyon boyutu uyusmazligi: %d vs %d\n", dataSize, sizeof(CalibrationData));
        return false;
    }

    _prefs.getBytes("calib", &_data, sizeof(CalibrationData));
    _prefs.end();

    if (!verifyCRC(_data)) {
        Serial.println("Kalibrasyon CRC kontrolu basarisiz - veri bozuk!");
        return false;
    }

    if (_data.version != 1) {
        Serial.printf("Kalibrasyon surum uyusmazligi: %d (beklenen 1)\n", _data.version);
        return false;
    }

    _valid = true;
    Serial.println("Kalibrasyon NVS'den basariyla yuklendi");
    return true;
}

bool SensorCalibration::saveToNVS(const CalibrationData &data) {//NVS'ye kaydetme
    CalibrationData saveData = data;
    saveData.timestamp = millis();
    saveData.version = 1;

    saveData.crc = calculateCRC(saveData);

    if (!_prefs.begin("gimbal", false)) {
        Serial.println("NVS yazma icin acilamadi");
        return false;
    }

    size_t written = _prefs.putBytes("calib", &saveData, sizeof(CalibrationData));
    _prefs.end();

    if (written != sizeof(CalibrationData)) {
        Serial.println("Kalibrasyon NVS'ye yazilamadi");
        return false;
    }

    _data = saveData;
    _valid = true;

    Serial.println("Kalibrasyon NVS'ye basariyla kaydedildi");
    return true;
}

void SensorCalibration::setData(const CalibrationData &data) {//bellekte kalibrasyon verisini günceller, NVS kaydetmeden
    _data = data;
    _valid = true;
}

bool SensorCalibration::eraseCalibration() {//kalibrasyon verilerini temizler
    if (!_prefs.begin("gimbal", false)) {
        return false;
    }

    bool success = _prefs.remove("calib");
    _prefs.end();

    if (success) {
        _valid = false;
        Serial.println("Kalibrasyon NVS'den silindi");
    }

    return success;
}

void SensorCalibration::printCalibration() const {//kalibrasyon verilerini yazar
    if (!_valid) {
        Serial.println("Gecerli kalibrasyon yuklenmedi");
        return;
    }

    Serial.println("\n========== KALIBRASYON VERILERI ==========");
    Serial.printf("Surum: %d\n", _data.version);
    Serial.printf("Zaman Damgasi: %u ms\n", _data.timestamp);

    Serial.println("\nGovde IMU Jiroskop Sapmasi (derece/s):");
    Serial.printf("  X: %.4f\n", _data.body_gyro_bias_x);
    Serial.printf("  Y: %.4f\n", _data.body_gyro_bias_y);
    Serial.printf("  Z: %.4f\n", _data.body_gyro_bias_z);

    Serial.println("\nKafa IMU Jiroskop Sapmasi (derece/s):");
    Serial.printf("  X: %.4f\n", _data.head_gyro_bias_x);
    Serial.printf("  Y: %.4f\n", _data.head_gyro_bias_y);
    Serial.printf("  Z: %.4f\n", _data.head_gyro_bias_z);

    Serial.println("\nEncoder Sifir Konumlari (derece):");
    Serial.printf("  Pan:  %.2f\n", _data.pan_encoder_zero);
    Serial.printf("  Tilt: %.2f\n", _data.tilt_encoder_zero);

    Serial.println("\nMekanik Limitler (derece):");
    Serial.printf("  Pan:  [%.2f, %.2f]\n", _data.pan_min, _data.pan_max);
    Serial.printf("  Tilt: [%.2f, %.2f]\n", _data.tilt_min, _data.tilt_max);

    Serial.printf("\nCRC: 0x%08X\n", _data.crc);
    Serial.println("==========================================\n");
}

uint32_t SensorCalibration::calculateCRC(const CalibrationData &data) const {
    uint32_t crc = 0xFFFFFFFF;

    const uint8_t *bytes = (const uint8_t *)&data;
    size_t len = sizeof(CalibrationData) - sizeof(uint32_t);

    for (size_t i = 0; i < len; i++) {
        crc = crc32_byte(crc, bytes[i]);
    }

    return ~crc;
}

bool SensorCalibration::verifyCRC(const CalibrationData &data) const {
    uint32_t computed = calculateCRC(data);
    return (computed == data.crc);
}
