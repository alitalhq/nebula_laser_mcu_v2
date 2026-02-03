#include "GroundReferenceCalibration.h"//yere göre referans alarak kalibre eder
#include <math.h>

GroundReferenceCalibration::GroundReferenceCalibration()
{
    _reference.ax = 0.0f;
    _reference.ay = 0.0f;
    _reference.az = -9.81f;  // Varsayilan: dik asagi
    _reference.pitch = 0.0f;
    _reference.roll = 0.0f;
    _reference.calibrated = false;
}

bool GroundReferenceCalibration::calibrate(IMUDriver &bodyIMU, uint16_t samples) {
    Serial.println("\n========== YER REFERANSI KALIBRASYONU ==========");
    Serial.println("Bu kalibrasyon 'gercek asagi' yonunu belirler");
    Serial.println("");
    Serial.println("GEREKSINIMLER:");
    Serial.println("  1. Drone'u DUZ ZEMINE yerlestirin");
    Serial.println("  2. Drone HAREKETSIZ olmali (titresim yok)");
    Serial.println("  3. Gimbal HERHANGI bir acida olabilir (otomatik ayarlanir)");
    Serial.println("");
    Serial.println("Kalibrasyon 3 saniye icinde basliyor...");
    delay(3000);

    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    uint16_t valid_samples = 0;

    Serial.printf("%d ornek toplaniyor", samples);

    for (uint16_t i = 0; i < samples; i++) {
        IMUDriver::IMUData data;

        if (!bodyIMU.read(data)) {
            continue;
        }

        sum_ax += data.accel_x;
        sum_ay += data.accel_y;
        sum_az += data.accel_z;
        valid_samples++;

        if ((i + 1) % 100 == 0) {
            Serial.print(".");
        }

        delay(1);
    }
    Serial.println(" Tamamlandi!");

    if (valid_samples < samples / 2) {
        Serial.println("HATA: Cok fazla basarisiz okuma");
        return false;
    }

    _reference.ax = sum_ax / valid_samples;
    _reference.ay = sum_ay / valid_samples;
    _reference.az = sum_az / valid_samples;

    _reference.pitch = atan2(_reference.ax,
                             sqrt(_reference.ay * _reference.ay +
                                  _reference.az * _reference.az)) * 180.0f / PI;

    _reference.roll = atan2(_reference.ay,
                            sqrt(_reference.ax * _reference.ax +
                                 _reference.az * _reference.az)) * 180.0f / PI;

    _reference.calibrated = true;

    Serial.println("\n[OK] Yer referansi kalibrasyonu BASARILI");
    printCalibration();
    Serial.println("==========================================\n");

    return true;
}

void GroundReferenceCalibration::computeGroundLockAngles(
    const IMUFusion::Orientation &currentOri,
    float &targetPan,
    float &targetTilt
)
{
    if (!_reference.calibrated) {
        targetPan = 0.0f;
        targetTilt = 90.0f;
        return;
    }

    float pitchDeviation = currentOri.pitch - _reference.pitch;
    float rollDeviation = currentOri.roll - _reference.roll;

    targetTilt = 90.0f - pitchDeviation;

    targetPan = 0.0f;
}

void GroundReferenceCalibration::printCalibration() const {

    float magnitude = sqrt(_reference.ax * _reference.ax +
                          _reference.ay * _reference.ay +
                          _reference.az * _reference.az);

    Serial.println("Kalibrasyon Sonuclari:");
    Serial.printf("  Yercekimi vektoru: [%.3f, %.3f, %.3f] m/s2\n",
                  _reference.ax, _reference.ay, _reference.az);
    Serial.printf("  Buyukluk: %.3f m/s2 (nominal: 9.81)\n", magnitude);
    Serial.printf("  Referans pitch: %.2f derece\n", _reference.pitch);
    Serial.printf("  Referans roll: %.2f derece\n", _reference.roll);

    if (fabs(magnitude - 9.81f) > 2.0f) {
        Serial.println("  [!] UYARI: Yercekimi buyuklugu anormal - sensor hatali olabilir");
    }

    if (fabs(_reference.pitch) > 5.0f) {
        Serial.println("  [!] UYARI: Pitch > 5 derece - drone duz olmayabilir");
    }

    if (fabs(_reference.roll) > 5.0f) {
        Serial.println("  [!] UYARI: Roll > 5 derece - drone duz olmayabilir");
    }
    
    if (fabs(_reference.pitch) < 2.0f && fabs(_reference.roll) < 2.0f) {
        Serial.println("  [OK] Drone kalibrasyon sirasinda duz idi");
    }
}
