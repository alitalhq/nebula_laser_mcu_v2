#include <Arduino.h>
#include <Wire.h>
#include "esp_task_wdt.h"
#include "config/HardwareConfig.h"
#include "config/ControlConfig.h"
#include "control/GroundReferenceCalibration.h"
#include "hardware/IMUDriver.h"
#include "hardware/BuzzerDriver.h"
#include "hardware/EncoderDriver.h"
#include "hardware/SoftI2C.h"
#include "hardware/StepperTimer.h"
#include "hardware/TMC2209Driver.h"
#include "communication/TargetManager.h"
#include "sensors/SensorHealth.h"
#include "sensors/SensorCalibration.h"
#include "tasks/Tasks.h"

IMUDriver g_bodyIMU;
IMUDriver g_headIMU;
EncoderDriver g_panEncoder;
EncoderDriver g_tiltEncoder;
SoftI2C g_softI2C_pan(SOFT_I2C_PAN_SDA, SOFT_I2C_PAN_SCL);   // Pan encoder için yazılımsal I2C
SoftI2C g_softI2C_tilt(SOFT_I2C_TILT_SDA, SOFT_I2C_TILT_SCL); // Tilt encoder için yazılımsal I2C
StepperTimer g_panMotor;
StepperTimer g_tiltMotor;
TMC2209Driver g_panTMC;
TMC2209Driver g_tiltTMC;
TargetManager g_targetMgr; // Hedef konum ve mod yöneticisi
SensorHealth g_sensorHealth;   // Sensör sağlık durumu izleyicisi
GroundReferenceCalibration g_groundRef;  // Yerçekimi referans kalibrasyonu
BuzzerDriver g_buzzer;

bool initializeHardware() { //donanımları başlatır

    DBG_PRINTLN("I2C veri yollari baslatiliyor..."); //i2c yollarını kuruyoruz

        // 4 adet i2c veri yolu bulunur bunlardan ikisi donanımsal (imu için)
    // diğer ikisi ise yazılımsal (encoder için) 

    // Wire.begin() öncesi bus temizle — crash/reset sonrası stuck SDA'yı serbest bırakır
    auto clearI2CBus = [](uint8_t sda, uint8_t scl) {
        pinMode(scl, OUTPUT); pinMode(sda, OUTPUT);
        digitalWrite(sda, HIGH);
        for (int i = 0; i < 9; i++) {
            digitalWrite(scl, HIGH); delayMicroseconds(5);
            digitalWrite(scl, LOW);  delayMicroseconds(5);
        }
        digitalWrite(sda, LOW); delayMicroseconds(5);
        digitalWrite(scl, HIGH); delayMicroseconds(5);
        digitalWrite(sda, HIGH); delayMicroseconds(5);
    };
    clearI2CBus(I2C0_SDA, I2C0_SCL);
    Wire.begin(I2C0_SDA, I2C0_SCL, HARDWARE_I2C_SPEED);
    Wire.setTimeout(I2C_TIMEOUT_MS);
    DBG_PRINTLN("I2C0 (Wire) @ 400kHz - Govde IMU icin");

    clearI2CBus(I2C1_SDA, I2C1_SCL);
    Wire1.begin(I2C1_SDA, I2C1_SCL, HARDWARE_I2C_SPEED);
    Wire1.setTimeout(I2C_TIMEOUT_MS);
    DBG_PRINTLN("I2C1 (Wire1) @ 400kHz - Kafa IMU icin");

    g_softI2C_pan.begin(SOFTWARE_I2C_SPEED);
    DBG_PRINTLN("SoftI2C Pan @ 100kHz - Pan Encoder icin");

    g_softI2C_tilt.begin(SOFTWARE_I2C_SPEED);
    DBG_PRINTLN("SoftI2C Tilt @ 100kHz - Tilt Encoder icin");

    /////////////////////////////////////////////////////////////////////////////////////////

    DBG_PRINTLN("Buzzer baslatiliyor...");//sesli uyarı için buzzer
    if (!g_buzzer.begin(BUZZER_PIN, BUZZER_PWM_CH, BUZZER_FREQ)) {
        DBG_PRINTLN("UYARI: Buzzer baslatma basarisiz");
    } else {
        DBG_PRINTLN("Buzzer TAMAM");
    }
    /////////////////////////////////////////////////////////////////////////////////////////

    DBG_PRINTLN("GPIO yapilandiriliyor...");// gpio pinleri ayarlanıyor

    pinMode(MOTOR_ENABLE_PIN, OUTPUT); //motor enable pini
    digitalWrite(MOTOR_ENABLE_PIN, HIGH);// HIGH olursa devre dışı

    pinMode(LASER_PIN, OUTPUT);//lazer ledi
    digitalWrite(LASER_PIN, LOW);

    pinMode(STATUS_LED_PIN, OUTPUT);//durum ledi (kartun üzerinde)
    digitalWrite(STATUS_LED_PIN, LOW);

    DBG_PRINTLN("GPIO yapilandirildi");

    /////////////////////////////////////////////////////////////////////////////////////////

    DBG_PRINTLN("TMC2209 suruculer baslatiliyor...");

    // PAN TMC2209 - HardwareSerial1 (UART2 - GPIO47/48)
    g_panTMC.begin(Serial2, TMC_PAN_RX_PIN, TMC_PAN_TX_PIN, "PAN");
    if (!g_panTMC.isUartOk()) {
        DBG_PRINTLN("UYARI: Pan TMC2209 UART basarisiz - standalone modda devam ediliyor");
        DBG_PRINTLN("  STEP/DIR calisir, UART yapilandirmasi (mikrostep/akim) uygulanmadi");
    } else {
        g_panTMC.printStatus();
        DBG_PRINTLN("Pan TMC2209 UART TAMAM");
    }

    // TILT TMC2209 - HardwareSerial1 (UART1 - GPIO45/46)
    g_tiltTMC.begin(Serial1, TMC_TILT_RX_PIN, TMC_TILT_TX_PIN, "TILT");
    if (!g_tiltTMC.isUartOk()) {
        DBG_PRINTLN("UYARI: Tilt TMC2209 UART basarisiz - standalone modda devam ediliyor");
    } else {
        g_tiltTMC.printStatus();
        DBG_PRINTLN("Tilt TMC2209 UART TAMAM");
    }

    /////////////////////////////////////////////////////////////////////////////////////////

    DBG_PRINTLN("Govde IMU baslatiliyor...");//IMU
    if (!g_bodyIMU.begin(Wire, I2C0_SDA, I2C0_SCL, BMI160_ADDR)) {
        DBG_PRINTLN("HATA: Govde IMU baslatma basarisiz");
        return false;
    }
    if (!g_bodyIMU.configure(BMI160_ACCEL_RANGE, BMI160_GYRO_RANGE, BMI160_ODR)) {
        DBG_PRINTLN("HATA: Govde IMU yapilandirma basarisiz");
        return false;
    }
    DBG_PRINTLN("Govde IMU TAMAM (I2C0, 0x68)");

    /////////////////////////////////////////////////////////////////////////////////////////

    DBG_PRINTLN("Kafa IMU baslatiliyor..."); //IMU
    if (!g_headIMU.begin(Wire1, I2C1_SDA, I2C1_SCL, BMI160_ADDR)) {
        DBG_PRINTLN("HATA: Kafa IMU baslatma basarisiz");
        return false;
    }
    if (!g_headIMU.configure(BMI160_ACCEL_RANGE, BMI160_GYRO_RANGE, BMI160_ODR)) {
        DBG_PRINTLN("HATA: Kafa IMU yapilandirma basarisiz");
        return false;
    }
    DBG_PRINTLN("Kafa IMU TAMAM (I2C1, 0x68)");

    /////////////////////////////////////////////////////////////////////////////////////////

    DBG_PRINTLN("Pan Encoder baslatiliyor...");//Encoder
    if (!g_panEncoder.begin(g_softI2C_pan, AS5600_ADDR)) {
        DBG_PRINTLN("KRITIK HATA: Pan encoder bulunamadi!");
        return false;
    }

    // Mıknatıs durumunu kontrol et - doğru çalışma için önemli
    EncoderDriver::MagnetStatus panMagnet = g_panEncoder.getMagnetStatus();
    if (panMagnet == EncoderDriver::MAGNET_GOOD) {
        DBG_PRINTLN("Pan Encoder TAMAM (SoftI2C, 0x36, miknatis iyi)");
    } else {
        DBG_PRINTF("Pan Encoder TAMAM ama miknatis durumu: %d (2=iyi)\n", panMagnet);
    }

    /////////////////////////////////////////////////////////////////////////////////////////

    DBG_PRINTLN("Tilt Encoder baslatiliyor...");//Encoder
    if (!g_tiltEncoder.begin(g_softI2C_tilt, AS5600_ADDR, true)) {
        DBG_PRINTLN("KRITIK HATA: Tilt encoder bulunamadi!");
        return false;
    }

    EncoderDriver::MagnetStatus tiltMagnet = g_tiltEncoder.getMagnetStatus();
    if (tiltMagnet == EncoderDriver::MAGNET_GOOD) {
        DBG_PRINTLN("Tilt Encoder TAMAM (SoftI2C, 0x36, miknatis iyi)");
    } else {
        DBG_PRINTF("Tilt Encoder TAMAM ama miknatis durumu: %d (2=iyi)\n", tiltMagnet);
    }

    /////////////////////////////////////////////////////////////////////////////////////////

    DBG_PRINTLN("Step motorlar baslatiliyor...");//Step motorlar

    if (!g_panMotor.begin(StepperTimer::PAN, PAN_STEP_PIN, PAN_DIR_PIN)) {
        DBG_PRINTLN("KRITIK HATA: Pan motor zamanlayicisi baslatma basarisiz!");
        return false;
    }
    DBG_PRINTLN("Pan Motor TAMAM");

    if (!g_tiltMotor.begin(StepperTimer::TILT, TILT_STEP_PIN, TILT_DIR_PIN)) {
        DBG_PRINTLN("KRITIK HATA: Tilt motor zamanlayicisi baslatma basarisiz!");
        return false;
    }
    DBG_PRINTLN("Tilt Motor TAMAM");

    DBG_PRINTLN("========== DONANIM TAMAM ==========\n");
    return true;
}

bool calibrateGyros() {//imuları kalibre edip NSVye kaydeder
    DBG_PRINTLN("\n========== JIROSKOP KALIBRASYONU ==========");
    DBG_PRINTLN("Gimbal 10 saniye boyunca HAREKETSIZ kalmalidir");
    DBG_PRINTLN("Kalibrasyon sirasinda gimbal'i HAREKET ETTIRMEYIN");
    DBG_PRINTLN("");
    DBG_PRINTLN("Kalibrasyon 3 saniye icinde basliyor...");

    delay(3000);

    g_buzzer.calibrationWarning();//kalibrasyon boyunca ses çıkarır


    DBG_PRINTLN("\nGovde IMU jiroskopu kalibre ediliyor...");
    if (!g_bodyIMU.calibrateGyro(1000)) { //Govde IMU'dan 1000 örnek ile kalibre ediliyor
        g_buzzer.noTone();
        g_buzzer.errorAlert();//Kalibre edilemezse hata sesi çıkartıyor
        DBG_PRINTLN("HATA: Govde IMU kalibrasyonu basarisiz");
        return false;
    }

    DBG_PRINTLN("Kafa IMU jiroskopu kalibre ediliyor...");
    if (!g_headIMU.calibrateGyro(1000)) { // Kafa IMU kalibrasyonu
        g_buzzer.noTone();
        g_buzzer.errorAlert();
        DBG_PRINTLN("HATA: Kafa IMU kalibrasyonu basarisiz");
        return false;
    }

    g_buzzer.noTone();
    delay(200);

    DBG_PRINTLN("\nKalibrasyon NVS'ye kaydediliyor..."); // kalibre verileri Non-Volatile Storage'e kaydediliyor
    SensorCalibration calib;                                // bu sayede cihaz yeniden başladığında kalibrasyon verileri korunur
    SensorCalibration::CalibrationData data;

    if (calib.loadFromNVS()) {
        data = calib.getData();//NVS'de veri varsa yükler
        DBG_PRINTLN("Mevcut mekanik limitler korunuyor");
    } else {
        data.pan_min = DEFAULT_PAN_MIN; //yoksa configteki default sınırları alır bu değerleri güncellemeyi unutma
        data.pan_max = DEFAULT_PAN_MAX;
        data.tilt_min = DEFAULT_TILT_MIN;
        data.tilt_max = DEFAULT_TILT_MAX;
        DBG_PRINTLN("Varsayilan mekanik limitler kullaniliyor");
    }

    // Jiroskop sapma değerlerini güncelle NSVdeki verilere göre
    g_bodyIMU.getGyroBias(data.body_gyro_bias_x, data.body_gyro_bias_y, data.body_gyro_bias_z);
    g_headIMU.getGyroBias(data.head_gyro_bias_x, data.head_gyro_bias_y, data.head_gyro_bias_z);

    if (calib.saveToNVS(data)) {//kaydet
        DBG_PRINTLN("Jiroskop kalibrasyonu NVS'ye kaydedildi");
    } else {
        DBG_PRINTLN("Kalibrasyon NVS'ye kaydedilemedi");
        DBG_PRINTLN("Kalibrasyon yeniden baslatmada kaybolacak");
    }

    DBG_PRINTLN("========== JIROSKOP KALIBRASYONU TAMAMLANDI ==========\n");
    return true;
}

bool loadCalibration() {//kalibrasyonu uygulama
    DBG_PRINTLN("\n========== KALIBRASYON YUKLENIYOR ==========");

    SensorCalibration calib;

    // NVS'den kalibrasyon verilerini almaya çalışır
    if (!calib.loadFromNVS()) {
        DBG_PRINTLN("NVS'de kalibrasyon bulunamadi");
        DBG_PRINTLN("Varsayilan degerler kullaniliyor");
        DBG_PRINTLN("Optimum performans icin jiroskop kalibrasyonu yapin");

        // Varsayılan kalibrasyon oluştur
        SensorCalibration::CalibrationData data;
        data.pan_min = DEFAULT_PAN_MIN;
        data.pan_max = DEFAULT_PAN_MAX;
        data.tilt_min = DEFAULT_TILT_MIN;
        data.tilt_max = DEFAULT_TILT_MAX;

        // Sıfır sapma değerleri (kalibrasyon yapılmamış)
        data.body_gyro_bias_x = 0;
        data.body_gyro_bias_y = 0;
        data.body_gyro_bias_z = 0;
        data.head_gyro_bias_x = 0;
        data.head_gyro_bias_y = 0;
        data.head_gyro_bias_z = 0;

        calib.setData(data);
    } else {
        DBG_PRINTLN("Kalibrasyon NVS'den yuklendi");
    }

    // Kalibrasyon değerlerini ekrana yazdırır
    calib.printCalibration();

    // Kalibrasyonu donanımlara yükler
    SensorCalibration::CalibrationData data = calib.getData();

    // IMUlara jiroskop sapma değerlerini verir
    g_bodyIMU.setGyroBias(data.body_gyro_bias_x, data.body_gyro_bias_y, data.body_gyro_bias_z);
    g_headIMU.setGyroBias(data.head_gyro_bias_x, data.head_gyro_bias_y, data.head_gyro_bias_z);

    // limitleri günceller
    g_limitConfig.pan_min = data.pan_min;
    g_limitConfig.pan_max = data.pan_max;
    g_limitConfig.tilt_min = data.tilt_min;
    g_limitConfig.tilt_max = data.tilt_max;
    // Limitler, encoder'ın seviyeleme sıfırına göre yeniden tanımlanana kadar devre dışı.
    g_limitConfig.enforce_limits = false;

    DBG_PRINTLN("========== KALIBRASYON UYGULANDI ==========\n");
    return true;
}

void initializeControllers() {
    DBG_PRINTLN("\n========== KONTROLCULER BASLATILIYOR ==========");

    float initialPan = g_panEncoder.readAngleDegrees(); // başlangıç encoder değerleri okunur
    float initialTilt = g_tiltEncoder.readAngleDegrees();

    // encoder okuma hatası kontrolü
    if (initialPan < 0 || initialTilt < 0) {
        DBG_PRINTLN("HATA: Baslangic encoder konumlari okunamadi");
        DBG_PRINTLN("HATA: Yedek olarak 0° kullaniliyor");
        initialPan = 0;
        initialTilt = 0;
    }

    DBG_PRINTF("Baslangic encoder konumu:\n");
    DBG_PRINTF("  Pan:  %.2f°\n", initialPan);
    DBG_PRINTF("  Tilt: %.2f°\n", initialTilt);

    // Hedef yöneticisini mevcut konumla başlat
    if (!g_targetMgr.begin(initialPan, initialTilt)) {
        DBG_PRINTLN("HATA: TargetManager baslatma basarisiz");
    } else {
        DBG_PRINTLN("TargetManager baslatildi");
    }

    //sensorler için mutex oluşturuyoruz. mutex çoklu işlerde aynı ayna erişilip hata oluşmasını engeller
    g_sensorData.mutex = xSemaphoreCreateMutex();
    if (!g_sensorData.mutex) {
        DBG_PRINTLN("KRITIK HATA: Sensor verisi mutex'i olusturulamadi");
        while(1) { //esp32 deki dahili led yanıp söner önemli bir hata olduğu için sonsuz döngüye girer
            digitalWrite(STATUS_LED_PIN, HIGH);
            delay(100);
            digitalWrite(STATUS_LED_PIN, LOW);
            delay(100);
        }
    }
    DBG_PRINTLN("Sensor verisi mutex'i olusturuldu");

    DBG_PRINTLN("========== KONTROLCULER BASLATILDI ==========\n");
}

void createTasks() {
    DBG_PRINTLN("\n========== FREERTOS GOREVLERI OLUSTURULUYOR ==========");
    //freeRTOSta çalışması için farklı taskler oluşturuluyor
    //görevleri eş zamanlı yapmaları sağlanıyor

    BaseType_t result;

    // ========================================
    // IMU Okuma Görevi (1000 Hz, Çekirdek 0, En Yüksek Öncelik)
    // Bu görev her iki IMU'dan paralel olarak veri okur.
    // Yüksek frekans, stabilizasyon için düşük gecikme sağlar.
    // ========================================
    result = xTaskCreatePinnedToCore(
        imuReadTask,
        "IMUReadTask",
        IMU_READ_TASK_STACK,
        NULL,
        IMU_READ_TASK_PRIORITY,
        NULL,
        IMU_READ_TASK_CORE
    );
    if (result != pdPASS) {
        DBG_PRINTLN("KRITIK HATA: IMUReadTask olusturulamadi");
        while(1);
    }
    DBG_PRINTF("✓ IMUReadTask olusturuldu\n");
    DBG_PRINTF("    Cekirdek: %d | Oncelik: %d | Hiz: %d Hz | Yigin: %d bayt\n",
                  IMU_READ_TASK_CORE, IMU_READ_TASK_PRIORITY,
                  IMU_READ_FREQ_HZ, IMU_READ_TASK_STACK);

    // ========================================
    // Stabilizasyon Görevi (500 Hz, Çekirdek 0)
    // Gövde hareketlerini telafi ederek gimbal kafasını sabit tutar.
    // PID kontrolcü kullanarak hızlı tepki verir.
    // ========================================
    result = xTaskCreatePinnedToCore(
        stabilizationTask,
        "StabilizationTask",
        STABILIZATION_TASK_STACK,
        NULL,
        STABILIZATION_TASK_PRIORITY,
        NULL,
        STABILIZATION_TASK_CORE
    );
    if (result != pdPASS) {
        DBG_PRINTLN("KRITIK HATA: StabilizationTask olusturulamadi");
        while(1);
    }
    DBG_PRINTF("✓ StabilizationTask olusturuldu\n");
    DBG_PRINTF("    Cekirdek: %d | Oncelik: %d | Hiz: %d Hz | Yigin: %d bayt\n",
                  STABILIZATION_TASK_CORE, STABILIZATION_TASK_PRIORITY,
                  STABILIZATION_FREQ_HZ, STABILIZATION_TASK_STACK);

    // ========================================
    // Konum Kontrol Görevi (200 Hz, Çekirdek 0)
    // Hedef konuma ulaşmak için motor hızlarını hesaplar.
    // Encoder geri bildirimi ile kapalı döngü kontrol yapar.
    // ========================================
    result = xTaskCreatePinnedToCore(
        positionControlTask,
        "PositionControlTask",
        POSITION_TASK_STACK,
        NULL,
        POSITION_TASK_PRIORITY,
        NULL,
        POSITION_TASK_CORE
    );
    if (result != pdPASS) {
        DBG_PRINTLN("KRITIK HATA: PositionControlTask olusturulamadi");
        while(1);
    }
    DBG_PRINTF("✓ PositionControlTask olusturuldu\n");
    DBG_PRINTF("    Cekirdek: %d | Oncelik: %d | Hiz: %d Hz | Yigin: %d bayt\n",
                  POSITION_TASK_CORE, POSITION_TASK_PRIORITY,
                  POSITION_FREQ_HZ, POSITION_TASK_STACK);

    // ========================================
    // Seri Haberleşme Görevi (Asenkron, Çekirdek 1)
    // ROS2 ile haberleşmeyi yönetir.
    // Komutları alır ve durum bilgisi gönderir.
    // ========================================
    result = xTaskCreatePinnedToCore(
        serialTask,
        "SerialTask",
        SERIAL_TASK_STACK,
        NULL,
        SERIAL_TASK_PRIORITY,
        NULL,
        SERIAL_TASK_CORE
    );
    if (result != pdPASS) {
        DBG_PRINTLN("KRITIK HATA: SerialTask olusturulamadi");
        while(1);
    }
    DBG_PRINTF("✓ SerialTask olusturuldu\n");
    DBG_PRINTF("    Cekirdek: %d | Oncelik: %d | Hiz: Asenkron | Yigin: %d bayt\n",
                  SERIAL_TASK_CORE, SERIAL_TASK_PRIORITY, SERIAL_TASK_STACK);

    // ========================================
    // Tanılama Görevi (10 Hz, Çekirdek 1)
    // Sistem durumunu izler ve raporlar.
    // Hata tespiti ve debug için kullanılır.
    // ========================================
    result = xTaskCreatePinnedToCore(
        diagnosticsTask,
        "DiagnosticsTask",
        DIAGNOSTICS_TASK_STACK,
        NULL,
        DIAGNOSTICS_TASK_PRIORITY,
        NULL,
        DIAGNOSTICS_TASK_CORE
    );
    if (result != pdPASS) {
        DBG_PRINTLN("KRITIK HATA: DiagnosticsTask olusturulamadi");
        while(1);
    }
    DBG_PRINTF("✓ DiagnosticsTask olusturuldu\n");
    DBG_PRINTF("    Cekirdek: %d | Oncelik: %d | Hiz: %d Hz | Yigin: %d bayt\n",
                  DIAGNOSTICS_TASK_CORE, DIAGNOSTICS_TASK_PRIORITY,
                  DIAGNOSTICS_FREQ_HZ, DIAGNOSTICS_TASK_STACK);

    DBG_PRINTLN("========== TUM GOREVLER OLUSTURULDU ==========\n");
}

void printSystemInfo() {
    DBG_PRINTLN("nNEBULA LASER MCU");
    DBG_PRINTLN("Version 2.0.0");
    DBG_PRINTLN();
    DBG_PRINTF("Cip: %s\n", ESP.getChipModel());
    DBG_PRINTF("Cekirdek Sayisi: %d\n", ESP.getChipCores());
    DBG_PRINTF("CPU Frekansi: %d MHz\n", ESP.getCpuFreqMHz());
    DBG_PRINTF("Flash Boyutu: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
    DBG_PRINTF("Bos Heap: %d bayt\n", ESP.getFreeHeap());
    DBG_PRINTF("PSRAM: %s\n", ESP.getPsramSize() > 0 ? "Mevcut" : "Mevcut degil");
    DBG_PRINTLN();
}

bool calibrateGroundReference() {
    DBG_PRINTLN("\n========== YERCEKIMI REFERANS KALIBRASYONU ==========");//gimbal modu için yere paralel konumdaki referansı alıyor
    DBG_PRINTLN("Bu islem 'gercek asagi' yonunu kalibre edecek");
    DBG_PRINTLN("");
    DBG_PRINTLN("  ONEMLI:");
    DBG_PRINTLN("  1. Drone'u DUZGUN BIR ZEMINE yerlestirin");
    DBG_PRINTLN("  2. Drone'u TAMAMEN HAREKETSIZ tutun");
    DBG_PRINTLN("  3. Kalibrasyon sirasinda drone'a DOKUNMAYIN");
    DBG_PRINTLN("");
    DBG_PRINTLN("Kalibrasyon 3 saniye icinde basliyor...");

    delay(3000);

    g_buzzer.calibrationWarning();

    if (!g_groundRef.calibrate(g_bodyIMU, 1000)) {
        g_buzzer.noTone();
        g_buzzer.errorAlert();  // Hata sesi
        DBG_PRINTLN("HATA: Yercekimi referans kalibrasyonu basarisiz");
        return false;
    }

    g_buzzer.noTone();
    delay(200);

    return true;
}

bool initializeWatchdog() {//watchdog sistem takılmasında otomatik yeniden başlatmayı sağlamaktadır bizim için önemli
    DBG_PRINTLN("\n========== WATCHDOG BASLATMA ==========");//esp32'ye göre kodlandı laser kartı tasarlanınca değiştirmeliyiz

    esp_err_t err = esp_task_wdt_init(WATCHDOG_TIMEOUT_SEC, WATCHDOG_PANIC);

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        DBG_PRINTF("HATA: Watchdog baslatma basarisiz: %d\n", err);
        return false;
    }

    // CPU 0'daki IDLE görevi yüksek öncelikli görevler (IMU/Stab/Pos) tarafından
    // tamamen bloke edildiğinden watchdog izlemesinden çıkarılır.
    // Uygulama görevleri kendi watchdog beslemelerini yönetir.
    esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(0));

    DBG_PRINTF("  ✓ Watchdog yapilandirildi (zaman asimi=%ds, panik=%s)\n",
                  WATCHDOG_TIMEOUT_SEC,
                  WATCHDOG_PANIC ? "etkin" : "devre disi");

    DBG_PRINTLN("========== WATCHDOG HAZIR ==========\n");
    return true;
}

// ============================================================================
// ARDUINO SETUP
// Bu fonksiyon sistem açılışında bir kez çalışır.
// Tüm donanım ve yazılım bileşenlerini sırayla başlatır.
// ============================================================================

void setup() {
    Serial.begin(SERIAL_BAUD_RATE, SERIAL_8N1, 44, 43);  // RX=GPIO44, TX=GPIO43 (UART0)
    delay(2000);

    DBG_PRINTLN("\n\n\n");
    printSystemInfo();

    if (!initializeHardware()) {//donanımları başlat
        DBG_PRINTLN("\nDONANIM BASLATMA BASARISIZ");
        DBG_PRINTLN("SISTEM DURDURULDU");

        while (true) {//hem led yanıp söner hem hata sesi verir
            digitalWrite(STATUS_LED_PIN, HIGH);
            delay(100);
            digitalWrite(STATUS_LED_PIN, LOW);
            delay(100);
            g_buzzer.errorAlert();
            delay(1000);
        }
    }

    if (!calibrateGroundReference()) {//yerçekimi referansını kalibre et
        DBG_PRINTLN("\nYercekimi referans kalibrasyonu BASARISIZ");
        DBG_PRINTLN("Varsayilan kullaniliyor (hatali olabilir)");
        g_buzzer.errorAlert();
        delay(3000);
    }

    if (!calibrateGyros()) {//imuları kalibre et
        DBG_PRINTLN("\nJIROSKOP KALIBRASYONU BASARISIZ");
        DBG_PRINTLN("Kalibre edilmemis jiroskoplarla devam ediliyor (ONERILMEZ)");
        DBG_PRINTLN("Stabilizasyon performansi dusuk olacak");
        g_buzzer.errorAlert();
        delay(3000);
    }

    if (!loadCalibration()) {//NSVden kalibrasyonu al
        DBG_PRINTLN("\nKALIBRASYON YUKLEME BASARISIZ");
        DBG_PRINTLN("Varsayilanlar kullaniliyor");
    }

    initializeControllers();//target managerı başlat

    createTasks();//FreeRTOS görevlerini başlat

    if (!initializeWatchdog()) {//watchdog'u başlat
        DBG_PRINTLN("\nWATCHDOG BASLATMA BASARISIZ");
        DBG_PRINTLN("Sistem watchdog koruması olmadan devam edecek");
        g_buzzer.errorAlert();
        delay(1000);
    }

    DBG_PRINTLN("\n========== MOTORLAR ETKINLESTIRILIYOR ==========");//motorları başlat
    DBG_PRINTLN("Sistemin kararli hale gelmesi icin 1 saniye bekleniyor...");
    delay(1000);

    digitalWrite(MOTOR_ENABLE_PIN, LOW);
    DBG_PRINTLN("✓ Motorlar etkinlestirildi");
    DBG_PRINTLN("========== MOTORLAR HAZIR ==========\n");


    DBG_PRINTLN("\nSISTEM HAZIR");
    DBG_PRINTLN("ROS2 komutlari bekleniyor...");
    DBG_PRINTLN();

    g_buzzer.systemReady();//sistem hazır bildirim sesi

    for (int i = 0; i < 3; i++) {//dahili pini 3 kez yanıp söndürür
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(200);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(200);
    }

    DBG_PRINTLN("Sistem calisir durumda. Sensor verileri izleniyor...\n");
}


void loop() {
    // Tüm kontrol mantığı FreeRTOS görevlerinde çalışır
    // Bu döngü fiilen kullanılmıyor
    vTaskDelay(portMAX_DELAY);
}
