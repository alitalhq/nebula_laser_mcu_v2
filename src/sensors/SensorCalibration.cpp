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
        DBG_PRINTLN("NVS namespace acilamadi");
        return false;
    }

    if (!_prefs.isKey("calib")) {
        _prefs.end();
        DBG_PRINTLN("NVS'de kalibrasyon bulunamadi");
        return false;
    }

    size_t dataSize = _prefs.getBytesLength("calib");
    if (dataSize != sizeof(CalibrationData)) {
        _prefs.end();
        DBG_PRINTF("Kalibrasyon boyutu uyusmazligi: %d vs %d\n", dataSize, sizeof(CalibrationData));
        return false;
    }

    _prefs.getBytes("calib", &_data, sizeof(CalibrationData));
    _prefs.end();

    if (!verifyCRC(_data)) {
        DBG_PRINTLN("Kalibrasyon CRC kontrolu basarisiz - veri bozuk!");
        return false;
    }

    if (_data.version != 1) {
        DBG_PRINTF("Kalibrasyon surum uyusmazligi: %d (beklenen 1)\n", _data.version);
        return false;
    }

    _valid = true;
    DBG_PRINTLN("Kalibrasyon NVS'den basariyla yuklendi");
    return true;
}

bool SensorCalibration::saveToNVS(const CalibrationData &data) {//NVS'ye kaydetme
    CalibrationData saveData = data;
    saveData.timestamp = millis();
    saveData.version = 1;

    saveData.crc = calculateCRC(saveData);

    if (!_prefs.begin("gimbal", false)) {
        DBG_PRINTLN("NVS yazma icin acilamadi");
        return false;
    }

    size_t written = _prefs.putBytes("calib", &saveData, sizeof(CalibrationData));
    _prefs.end();

    if (written != sizeof(CalibrationData)) {
        DBG_PRINTLN("Kalibrasyon NVS'ye yazilamadi");
        return false;
    }

    _data = saveData;
    _valid = true;

    DBG_PRINTLN("Kalibrasyon NVS'ye basariyla kaydedildi");
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
        DBG_PRINTLN("Kalibrasyon NVS'den silindi");
    }

    return success;
}

void SensorCalibration::printCalibration() const {//kalibrasyon verilerini yazar
    if (!_valid) {
        DBG_PRINTLN("Gecerli kalibrasyon yuklenmedi");
        return;
    }

    DBG_PRINTLN("\n========== KALIBRASYON VERILERI ==========");
    DBG_PRINTF("Surum: %d\n", _data.version);
    DBG_PRINTF("Zaman Damgasi: %u ms\n", _data.timestamp);

    DBG_PRINTLN("\nGovde IMU Jiroskop Sapmasi (derece/s):");
    DBG_PRINTF("  X: %.4f\n", _data.body_gyro_bias_x);
    DBG_PRINTF("  Y: %.4f\n", _data.body_gyro_bias_y);
    DBG_PRINTF("  Z: %.4f\n", _data.body_gyro_bias_z);

    DBG_PRINTLN("\nKafa IMU Jiroskop Sapmasi (derece/s):");
    DBG_PRINTF("  X: %.4f\n", _data.head_gyro_bias_x);
    DBG_PRINTF("  Y: %.4f\n", _data.head_gyro_bias_y);
    DBG_PRINTF("  Z: %.4f\n", _data.head_gyro_bias_z);

    DBG_PRINTLN("\nEncoder Sifir Konumlari (derece):");
    DBG_PRINTF("  Pan:  %.2f\n", _data.pan_encoder_zero);
    DBG_PRINTF("  Tilt: %.2f\n", _data.tilt_encoder_zero);

    DBG_PRINTLN("\nMekanik Limitler (derece):");
    DBG_PRINTF("  Pan:  [%.2f, %.2f]\n", _data.pan_min, _data.pan_max);
    DBG_PRINTF("  Tilt: [%.2f, %.2f]\n", _data.tilt_min, _data.tilt_max);

    DBG_PRINTF("\nCRC: 0x%08X\n", _data.crc);
    DBG_PRINTLN("==========================================\n");
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
