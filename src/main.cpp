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
TargetManager g_targetMgr; // Hedef konum ve mod yöneticisi
SensorHealth g_sensorHealth;   // Sensör sağlık durumu izleyicisi
GroundReferenceCalibration g_groundRef;  // Yerçekimi referans kalibrasyonu
BuzzerDriver g_buzzer;

bool initializeHardware() { //donanımları başlatır

    Serial.println("I2C veri yollari baslatiliyor..."); //i2c yollarını kuruyoruz

        // 4 adet i2c veri yolu bulunur bunlardan ikisi donanımsal (imu için)
    // diğer ikisi ise yazılımsal (encoder için) 

    Wire.begin(I2C0_SDA, I2C0_SCL, HARDWARE_I2C_SPEED);
    Wire.setTimeout(I2C_TIMEOUT_MS);
    Serial.println("I2C0 (Wire) @ 400kHz - Govde IMU icin");

    Wire1.begin(I2C1_SDA, I2C1_SCL, HARDWARE_I2C_SPEED);
    Wire1.setTimeout(I2C_TIMEOUT_MS);
    Serial.println("I2C1 (Wire1) @ 400kHz - Kafa IMU icin");

    g_softI2C_pan.begin(SOFTWARE_I2C_SPEED);
    Serial.println("SoftI2C Pan @ 100kHz - Pan Encoder icin");

    g_softI2C_tilt.begin(SOFTWARE_I2C_SPEED);
    Serial.println("SoftI2C Tilt @ 100kHz - Tilt Encoder icin");

    /////////////////////////////////////////////////////////////////////////////////////////

    Serial.println("Buzzer baslatiliyor...");//sesli uyarı için buzzer
    if (!g_buzzer.begin(BUZZER_PIN, BUZZER_PWM_CH, BUZZER_FREQ)) {
        Serial.println("UYARI: Buzzer baslatma basarisiz");
    } else {
        Serial.println("Buzzer TAMAM");
    }
    /////////////////////////////////////////////////////////////////////////////////////////

    Serial.println("GPIO yapilandiriliyor...");// gpio pinleri ayarlanıyor

    pinMode(MOTOR_ENABLE_PIN, OUTPUT); //motor enable pini
    digitalWrite(MOTOR_ENABLE_PIN, HIGH);// HIGH olursa devre dışı

    pinMode(LASER_PIN, OUTPUT);//lazer ledi
    digitalWrite(LASER_PIN, LOW);

    pinMode(STATUS_LED_PIN, OUTPUT);//durum ledi (kartun üzerinde)
    digitalWrite(STATUS_LED_PIN, LOW);

    Serial.println("GPIO yapilandirildi");

    /////////////////////////////////////////////////////////////////////////////////////////

    Serial.println("Govde IMU baslatiliyor...");//IMU
    if (!g_bodyIMU.begin(Wire, BMI160_ADDR)) {
        Serial.println("HATA: Govde IMU baslatma basarisiz");
        return false;
    }
    if (!g_bodyIMU.configure(BMI160_ACCEL_RANGE, BMI160_GYRO_RANGE, BMI160_ODR)) {
        Serial.println("HATA: Govde IMU yapilandirma basarisiz");
        return false;
    }
    Serial.println("Govde IMU TAMAM (I2C0, 0x68)");

    /////////////////////////////////////////////////////////////////////////////////////////

    Serial.println("Kafa IMU baslatiliyor..."); //IMU
    if (!g_headIMU.begin(Wire1, BMI160_ADDR)) {
        Serial.println("HATA: Kafa IMU baslatma basarisiz");
        return false;
    }
    if (!g_headIMU.configure(BMI160_ACCEL_RANGE, BMI160_GYRO_RANGE, BMI160_ODR)) {
        Serial.println("HATA: Kafa IMU yapilandirma basarisiz");
        return false;
    }
    Serial.println("Kafa IMU TAMAM (I2C1, 0x68)");

    /////////////////////////////////////////////////////////////////////////////////////////

    Serial.println("Pan Encoder baslatiliyor...");//Encoder
    if (!g_panEncoder.begin(g_softI2C_pan, AS5600_ADDR)) {
        Serial.println("KRITIK HATA: Pan encoder bulunamadi!");
        return false;
    }

    // Mıknatıs durumunu kontrol et - doğru çalışma için önemli
    EncoderDriver::MagnetStatus panMagnet = g_panEncoder.getMagnetStatus();
    if (panMagnet == EncoderDriver::MAGNET_GOOD) {
        Serial.println("Pan Encoder TAMAM (SoftI2C, 0x36, miknatis iyi)");
    } else {
        Serial.printf("Pan Encoder TAMAM ama miknatis durumu: %d (2=iyi)\n", panMagnet);
    }

    /////////////////////////////////////////////////////////////////////////////////////////

    Serial.println("Tilt Encoder baslatiliyor...");//Encoder
    if (!g_tiltEncoder.begin(g_softI2C_tilt, AS5600_ADDR)) {
        Serial.println("KRITIK HATA: Tilt encoder bulunamadi!");
        return false;
    }

    EncoderDriver::MagnetStatus tiltMagnet = g_tiltEncoder.getMagnetStatus();
    if (tiltMagnet == EncoderDriver::MAGNET_GOOD) {
        Serial.println("Tilt Encoder TAMAM (SoftI2C, 0x36, miknatis iyi)");
    } else {
        Serial.printf("Tilt Encoder TAMAM ama miknatis durumu: %d (2=iyi)\n", tiltMagnet);
    }

    /////////////////////////////////////////////////////////////////////////////////////////

    Serial.println("Step motorlar baslatiliyor...");//Step motorlar

    if (!g_panMotor.begin(StepperTimer::PAN, PAN_STEP_PIN, PAN_DIR_PIN)) {
        Serial.println("KRITIK HATA: Pan motor zamanlayicisi baslatma basarisiz!");
        return false;
    }
    Serial.println("Pan Motor TAMAM");

    if (!g_tiltMotor.begin(StepperTimer::TILT, TILT_STEP_PIN, TILT_DIR_PIN)) {
        Serial.println("KRITIK HATA: Tilt motor zamanlayicisi baslatma basarisiz!");
        return false;
    }
    Serial.println("Tilt Motor TAMAM");

    Serial.println("========== DONANIM TAMAM ==========\n");
    return true;
}

bool calibrateGyros() {//imuları kalibre edip NSVye kaydeder
    Serial.println("\n========== JIROSKOP KALIBRASYONU ==========");
    Serial.println("Gimbal 10 saniye boyunca HAREKETSIZ kalmalidir");
    Serial.println("Kalibrasyon sirasinda gimbal'i HAREKET ETTIRMEYIN");
    Serial.println("");
    Serial.println("Kalibrasyon 3 saniye icinde basliyor...");

    delay(3000);

    g_buzzer.calibrationWarning();//kalibrasyon boyunca ses çıkarır


    Serial.println("\nGovde IMU jiroskopu kalibre ediliyor...");
    if (!g_bodyIMU.calibrateGyro(1000)) { //Govde IMU'dan 1000 örnek ile kalibre ediliyor
        g_buzzer.noTone();
        g_buzzer.errorAlert();//Kalibre edilemezse hata sesi çıkartıyor
        Serial.println("HATA: Govde IMU kalibrasyonu basarisiz");
        return false;
    }

    Serial.println("Kafa IMU jiroskopu kalibre ediliyor...");
    if (!g_headIMU.calibrateGyro(1000)) { // Kafa IMU kalibrasyonu
        g_buzzer.noTone();
        g_buzzer.errorAlert();
        Serial.println("HATA: Kafa IMU kalibrasyonu basarisiz");
        return false;
    }

    g_buzzer.noTone();
    delay(200);

    Serial.println("\nKalibrasyon NVS'ye kaydediliyor..."); // kalibre verileri Non-Volatile Storage'e kaydediliyor
    SensorCalibration calib;                                // bu sayede cihaz yeniden başladığında kalibrasyon verileri korunur
    SensorCalibration::CalibrationData data;

    if (calib.loadFromNVS()) {
        data = calib.getData();//NVS'de veri varsa yükler
        Serial.println("Mevcut mekanik limitler korunuyor");
    } else {
        data.pan_min = DEFAULT_PAN_MIN; //yoksa configteki default sınırları alır bu değerleri güncellemeyi unutma
        data.pan_max = DEFAULT_PAN_MAX;
        data.tilt_min = DEFAULT_TILT_MIN;
        data.tilt_max = DEFAULT_TILT_MAX;
        Serial.println("Varsayilan mekanik limitler kullaniliyor");
    }

    // Jiroskop sapma değerlerini güncelle NSVdeki verilere göre
    g_bodyIMU.getGyroBias(data.body_gyro_bias_x, data.body_gyro_bias_y, data.body_gyro_bias_z);
    g_headIMU.getGyroBias(data.head_gyro_bias_x, data.head_gyro_bias_y, data.head_gyro_bias_z);

    if (calib.saveToNVS(data)) {//kaydet
        Serial.println("Jiroskop kalibrasyonu NVS'ye kaydedildi");
    } else {
        Serial.println("Kalibrasyon NVS'ye kaydedilemedi");
        Serial.println("Kalibrasyon yeniden baslatmada kaybolacak");
    }

    Serial.println("========== JIROSKOP KALIBRASYONU TAMAMLANDI ==========\n");
    return true;
}

bool loadCalibration() {//kalibrasyonu uygulama
    Serial.println("\n========== KALIBRASYON YUKLENIYOR ==========");

    SensorCalibration calib;

    // NVS'den kalibrasyon verilerini almaya çalışır
    if (!calib.loadFromNVS()) {
        Serial.println("NVS'de kalibrasyon bulunamadi");
        Serial.println("Varsayilan degerler kullaniliyor");
        Serial.println("Optimum performans icin jiroskop kalibrasyonu yapin");

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
        Serial.println("Kalibrasyon NVS'den yuklendi");
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

    Serial.println("========== KALIBRASYON UYGULANDI ==========\n");
    return true;
}

void initializeControllers() {
    Serial.println("\n========== KONTROLCULER BASLATILIYOR ==========");

    float initialPan = g_panEncoder.readAngleDegrees(); // başlangıç encoder değerleri okunur
    float initialTilt = g_tiltEncoder.readAngleDegrees();

    // encoder okuma hatası kontrolü
    if (initialPan < 0 || initialTilt < 0) {
        Serial.println("HATA: Baslangic encoder konumlari okunamadi");
        Serial.println("HATA: Yedek olarak 0° kullaniliyor");
        initialPan = 0;
        initialTilt = 0;
    }

    Serial.printf("Baslangic encoder konumu:\n");
    Serial.printf("  Pan:  %.2f°\n", initialPan);
    Serial.printf("  Tilt: %.2f°\n", initialTilt);

    // Hedef yöneticisini mevcut konumla başlat
    if (!g_targetMgr.begin(initialPan, initialTilt)) {
        Serial.println("HATA: TargetManager baslatma basarisiz");
    } else {
        Serial.println("TargetManager baslatildi");
    }

    //sensorler için mutex oluşturuyoruz. mutex çoklu işlerde aynı ayna erişilip hata oluşmasını engeller
    g_sensorData.mutex = xSemaphoreCreateMutex();
    if (!g_sensorData.mutex) {
        Serial.println("KRITIK HATA: Sensor verisi mutex'i olusturulamadi");
        while(1) { //esp32 deki dahili led yanıp söner önemli bir hata olduğu için sonsuz döngüye girer
            digitalWrite(STATUS_LED_PIN, HIGH);
            delay(100);
            digitalWrite(STATUS_LED_PIN, LOW);
            delay(100);
        }
    }
    Serial.println("Sensor verisi mutex'i olusturuldu");

    Serial.println("========== KONTROLCULER BASLATILDI ==========\n");
}

void createTasks() {
    Serial.println("\n========== FREERTOS GOREVLERI OLUSTURULUYOR ==========");
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
        Serial.println("KRITIK HATA: IMUReadTask olusturulamadi");
        while(1);
    }
    Serial.printf("✓ IMUReadTask olusturuldu\n");
    Serial.printf("    Cekirdek: %d | Oncelik: %d | Hiz: %d Hz | Yigin: %d bayt\n",
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
        Serial.println("KRITIK HATA: StabilizationTask olusturulamadi");
        while(1);
    }
    Serial.printf("✓ StabilizationTask olusturuldu\n");
    Serial.printf("    Cekirdek: %d | Oncelik: %d | Hiz: %d Hz | Yigin: %d bayt\n",
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
        Serial.println("KRITIK HATA: PositionControlTask olusturulamadi");
        while(1);
    }
    Serial.printf("✓ PositionControlTask olusturuldu\n");
    Serial.printf("    Cekirdek: %d | Oncelik: %d | Hiz: %d Hz | Yigin: %d bayt\n",
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
        Serial.println("KRITIK HATA: SerialTask olusturulamadi");
        while(1);
    }
    Serial.printf("✓ SerialTask olusturuldu\n");
    Serial.printf("    Cekirdek: %d | Oncelik: %d | Hiz: Asenkron | Yigin: %d bayt\n",
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
        Serial.println("KRITIK HATA: DiagnosticsTask olusturulamadi");
        while(1);
    }
    Serial.printf("✓ DiagnosticsTask olusturuldu\n");
    Serial.printf("    Cekirdek: %d | Oncelik: %d | Hiz: %d Hz | Yigin: %d bayt\n",
                  DIAGNOSTICS_TASK_CORE, DIAGNOSTICS_TASK_PRIORITY,
                  DIAGNOSTICS_FREQ_HZ, DIAGNOSTICS_TASK_STACK);

    Serial.println("========== TUM GOREVLER OLUSTURULDU ==========\n");
}

void printSystemInfo() {
    Serial.println("nNEBULA LASER MCU");
    Serial.println("Version 2.0.0");
    Serial.println();
    Serial.printf("Cip: %s\n", ESP.getChipModel());
    Serial.printf("Cekirdek Sayisi: %d\n", ESP.getChipCores());
    Serial.printf("CPU Frekansi: %d MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("Flash Boyutu: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
    Serial.printf("Bos Heap: %d bayt\n", ESP.getFreeHeap());
    Serial.printf("PSRAM: %s\n", ESP.getPsramSize() > 0 ? "Mevcut" : "Mevcut degil");
    Serial.println();
}

bool calibrateGroundReference() {
    Serial.println("\n========== YERCEKIMI REFERANS KALIBRASYONU ==========");//gimbal modu için yere paralel konumdaki referansı alıyor
    Serial.println("Bu islem 'gercek asagi' yonunu kalibre edecek");
    Serial.println("");
    Serial.println("  ONEMLI:");
    Serial.println("  1. Drone'u DUZGUN BIR ZEMINE yerlestirin");
    Serial.println("  2. Drone'u TAMAMEN HAREKETSIZ tutun");
    Serial.println("  3. Kalibrasyon sirasinda drone'a DOKUNMAYIN");
    Serial.println("");
    Serial.println("Kalibrasyon 3 saniye icinde basliyor...");

    delay(3000);

    g_buzzer.calibrationWarning();

    if (!g_groundRef.calibrate(g_bodyIMU, 1000)) {
        g_buzzer.noTone();
        g_buzzer.errorAlert();  // Hata sesi
        Serial.println("HATA: Yercekimi referans kalibrasyonu basarisiz");
        return false;
    }

    g_buzzer.noTone();
    delay(200);

    return true;
}

bool initializeWatchdog() {//watchdog sistem takılmasında otomatik yeniden başlatmayı sağlamaktadır bizim için önemli
    Serial.println("\n========== WATCHDOG BASLATMA ==========");//esp32'ye göre kodlandı laser kartı tasarlanınca değiştirmeliyiz

    esp_err_t err = esp_task_wdt_init(WATCHDOG_TIMEOUT_SEC, WATCHDOG_PANIC);

    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        Serial.printf("HATA: Watchdog baslatma basarisiz: %d\n", err);
        return false;
    }

    Serial.printf("  ✓ Watchdog yapilandirildi (zaman asimi=%ds, panik=%s)\n",
                  WATCHDOG_TIMEOUT_SEC,
                  WATCHDOG_PANIC ? "etkin" : "devre disi");

    Serial.println("========== WATCHDOG HAZIR ==========\n");
    return true;
}

// ============================================================================
// ARDUINO SETUP
// Bu fonksiyon sistem açılışında bir kez çalışır.
// Tüm donanım ve yazılım bileşenlerini sırayla başlatır.
// ============================================================================

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(2000);

    Serial.println("\n\n\n");
    printSystemInfo();

    if (!initializeHardware()) {//donanımları başlat
        Serial.println("\nDONANIM BASLATMA BASARISIZ");
        Serial.println("SISTEM DURDURULDU");

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
        Serial.println("\nYercekimi referans kalibrasyonu BASARISIZ");
        Serial.println("Varsayilan kullaniliyor (hatali olabilir)");
        g_buzzer.errorAlert();
        delay(3000);
    }

    if (!calibrateGyros()) {//imuları kalibre et
        Serial.println("\nJIROSKOP KALIBRASYONU BASARISIZ");
        Serial.println("Kalibre edilmemis jiroskoplarla devam ediliyor (ONERILMEZ)");
        Serial.println("Stabilizasyon performansi dusuk olacak");
        g_buzzer.errorAlert();
        delay(3000);
    }

    if (!loadCalibration()) {//NSVden kalibrasyonu al
        Serial.println("\nKALIBRASYON YUKLEME BASARISIZ");
        Serial.println("Varsayilanlar kullaniliyor");
    }

    initializeControllers();//target managerı başlat

    createTasks();//FreeRTOS görevlerini başlat

    if (!initializeWatchdog()) {//watchdog'u başlat
        Serial.println("\nWATCHDOG BASLATMA BASARISIZ");
        Serial.println("Sistem watchdog koruması olmadan devam edecek");
        g_buzzer.errorAlert();
        delay(1000);
    }

    Serial.println("\n========== MOTORLAR ETKINLESTIRILIYOR ==========");//motorları başlat
    Serial.println("Sistemin kararli hale gelmesi icin 1 saniye bekleniyor...");
    delay(1000);

    digitalWrite(MOTOR_ENABLE_PIN, LOW);
    Serial.println("✓ Motorlar etkinlestirildi");
    Serial.println("========== MOTORLAR HAZIR ==========\n");


    Serial.println("\nSISTEM HAZIR");
    Serial.println("ROS2 komutlari bekleniyor...");
    Serial.println();

    g_buzzer.systemReady();//sistem hazır bildirim sesi

    for (int i = 0; i < 3; i++) {//dahili pini 3 kez yanıp söndürür
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(200);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(200);
    }

    Serial.println("Sistem calisir durumda. Sensor verileri izleniyor...\n");
}


void loop() {
    // Tüm kontrol mantığı FreeRTOS görevlerinde çalışır
    // Bu döngü fiilen kullanılmıyor
    vTaskDelay(portMAX_DELAY);
}
