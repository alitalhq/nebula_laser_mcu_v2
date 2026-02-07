#include "Tasks.h"
#include "esp_task_wdt.h"
#include "../config/HardwareConfig.h"
#include "../config/ControlConfig.h"
#include "../hardware/IMUDriver.h"
#include "../hardware/EncoderDriver.h"
#include "../hardware/SoftI2C.h"
#include "../communication/ParallelIMUReader.h"
#include "../communication/SerialProtocol.h"
#include "../sensors/IMUFusion.h"
#include "../sensors/EncoderFilter.h"
#include "../control/StabilizationController.h"
#include "../control/PositionController.h"
#include "../control/CommandCombiner.h"
#include "../control/LimitEnforcer.h"
#include "../control/GroundLockController.h"
#include "../control/GroundReferenceCalibration.h"
#include "../hardware/StepperTimer.h"
#include "../hardware/TMC2209Driver.h"
#include "../hardware/BuzzerDriver.h"
#include "../communication/TargetManager.h"
#include "../sensors/SensorHealth.h"

SharedSensorData g_sensorData;



// Donanim ornekleri
extern IMUDriver g_bodyIMU; // main.cpp de tanımlı nesneler; extern edildi
extern IMUDriver g_headIMU;
extern EncoderDriver g_panEncoder;
extern EncoderDriver g_tiltEncoder;
extern BuzzerDriver g_buzzer;
extern StepperTimer g_panMotor;
extern StepperTimer g_tiltMotor;
extern TMC2209Driver g_panTMC;
extern TMC2209Driver g_tiltTMC;
extern TargetManager g_targetMgr;
extern SensorHealth g_sensorHealth;




// Sensor isleme ornekleri
static ParallelIMUReader imuReader;  // Paralel IMU okuyucu
static IMUFusion bodyFusion;         // Govde IMU fuzyonu
static IMUFusion headFusion;         // Kafa IMU fuzyonu
static EncoderFilter panFilter;      // Pan encoder filtresi
static EncoderFilter tiltFilter;     // Tilt encoder filtresi




// Kontrolcu ornekleri
static StabilizationController stabCtrl;  // Stabilizasyon kontrolcusu
static PositionController posCtrl;         // Pozisyon kontrolcusu
static CommandCombiner combiner;           // Komut birlestiricisi
static LimitEnforcer limiter;              // Limit zorlayicisi
static GroundLockController groundLockCtrl; // Yer-kilit kontrolcusu



void imuReadTask(void *params) { //IMU okuma görevi
    Serial.println("IMUOkumaGorevi: Basladi");

    esp_err_t wdt_err = esp_task_wdt_add(NULL); //watchdog aboneliği
    if (wdt_err != ESP_OK) {
        Serial.printf("IMUOkumaGorevi: UYARI - Watchdog aboneligi basarisiz (%d)\n", wdt_err);
    } else {
        Serial.println("IMUOkumaGorevi: Watchdog'a abone olundu");
    }

    if (!imuReader.begin(&g_bodyIMU, &g_headIMU)) {//paralel imu okuyucuyu başlat
        Serial.println("OLUMCUL: ParallelIMUReader baslatilamadi");
        esp_task_wdt_delete(NULL);  // Cikmadan once aboneligi iptal et
        vTaskDelete(NULL);
        return;
    }

    bodyFusion.begin(g_fusionConfig.alpha); //imu fusion filtreleri başladı
    headFusion.begin(g_fusionConfig.alpha);

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(IMU_READ_PERIOD_MS);

    uint32_t loopCount = 0;
    uint32_t lastPrintTime = 0;
    uint32_t readFailures = 0;
    uint32_t lastWdtFeedTime = 0;

    while (true) {
        vTaskDelayUntil(&lastWakeTime, period);
        loopCount++;

        ParallelIMUReader::IMUDataPair imuData;//imu okuma

        if (!imuReader.readBlocking(imuData, 2)) { //2ms zaman aşımı ile paralel i2c okunur
            readFailures++;
            g_sensorHealth.recordFailure(SensorHealth::BODY_IMU);
            g_sensorHealth.recordFailure(SensorHealth::HEAD_IMU);
            continue;
        }

        if (!imuData.isValid()) {//veri geçerli mi kontrol edilir
            readFailures++;
            continue;
        }

        g_sensorHealth.recordSuccess(SensorHealth::BODY_IMU);//veriyi kaydet
        g_sensorHealth.recordSuccess(SensorHealth::HEAD_IMU);

        bodyFusion.update(imuData.bodyData, IMU_READ_PERIOD_MS / 1000.0f);//fusion filtresi güncellenir
        headFusion.update(imuData.headData, IMU_READ_PERIOD_MS / 1000.0f);

        IMUFusion::Orientation bodyOri = bodyFusion.getOrientation();
        IMUFusion::Orientation headOri = headFusion.getOrientation();


        float deltaIMUPan = MathUtils::angularError(bodyOri.yaw, headOri.yaw); // imu deltası hesaplanır
        float deltaIMUTilt = MathUtils::angularError(bodyOri.pitch, headOri.pitch);

        if (xSemaphoreTake(g_sensorData.mutex, pdMS_TO_TICKS(1)) == pdTRUE) { // atomic dataya yazar
            g_sensorData.bodyRoll = bodyOri.roll;
            g_sensorData.bodyPitch = bodyOri.pitch;
            g_sensorData.bodyYaw = bodyOri.yaw;

            g_sensorData.headRoll = headOri.roll;
            g_sensorData.headPitch = headOri.pitch;
            g_sensorData.headYaw = headOri.yaw;

            g_sensorData.deltaIMUPan = deltaIMUPan;
            g_sensorData.deltaIMUTilt = deltaIMUTilt;
            g_sensorData.imuTimestamp = micros();

            xSemaphoreGive(g_sensorData.mutex);
        }



        if (millis() - lastWdtFeedTime > IMU_TASK_WDT_FEED_MS) { // watchdog beslenir
            lastWdtFeedTime = millis();
            esp_task_wdt_reset();
        }

        if (millis() - lastPrintTime > 1000) {//saniyede bir verileri yazar
            lastPrintTime = millis();

            Serial.printf("[IMU] Hiz: %d Hz | Govde: Y=%.1f P=%.1f R=%.1f | Kafa: Y=%.1f P=%.1f R=%.1f | Delta: P=%.2f T=%.2f | Hatalar: %u | Zamanasimi: %u\n",
                loopCount,
                bodyOri.yaw, bodyOri.pitch, bodyOri.roll,
                headOri.yaw, headOri.pitch, headOri.roll,
                deltaIMUPan, deltaIMUTilt,
                readFailures,
                imuReader.getTimeoutCount()
            );

            loopCount = 0;
            readFailures = 0;
            imuReader.resetStats();
        }
    }
}

void stabilizationTask(void *params) {//stabilizasyon görevi
    Serial.println("StabilizasyonGorevi: Basladi");

    stabCtrl.begin(g_stabConfig);//stabilizasyon kontrolcüsü başlar

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(STABILIZATION_PERIOD_MS);

    uint32_t loopCount = 0;
    uint32_t lastPrintTime = 0;

    while (true) {
        vTaskDelayUntil(&lastWakeTime, period);
        loopCount++;

        IMUFusion::Orientation bodyOri, headOri;//imu verisi okuunr
        float encoderVelPan = 0, encoderVelTilt = 0;



        if (xSemaphoreTake(g_sensorData.mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            bodyOri.roll = g_sensorData.bodyRoll;
            bodyOri.pitch = g_sensorData.bodyPitch;
            bodyOri.yaw = g_sensorData.bodyYaw;

            headOri.roll = g_sensorData.headRoll;
            headOri.pitch = g_sensorData.headPitch;
            headOri.yaw = g_sensorData.headYaw;

            encoderVelPan = g_sensorData.encoderVelPan;
            encoderVelTilt = g_sensorData.encoderVelTilt;

            xSemaphoreGive(g_sensorData.mutex);
        }

        stabCtrl.update(//kontrolcü güncellenir
            bodyOri,
            headOri,
            encoderVelPan,
            encoderVelTilt,
            STABILIZATION_PERIOD_MS / 1000.0f
        );

        if (millis() - lastPrintTime > 1000) {// saniyede bir yazıılr
            lastPrintTime = millis();

            float stabVelPan = stabCtrl.getVelocityCommandPan();
            float stabVelTilt = stabCtrl.getVelocityCommandTilt();


            Serial.printf("[STAB] Hiz: %d Hz | HizKomutu: P=%.2f T=%.2f derece/s\n",
                loopCount,
                stabVelPan, stabVelTilt
            );

            loopCount = 0;
        }
    }
}

void positionControlTask(void *params) {//pozisyon kontrol görevi
    Serial.println("PozisyonKontrolGorevi: Basladi");

    esp_err_t wdt_err = esp_task_wdt_add(NULL);//watchdog aboneliği
    if (wdt_err != ESP_OK) {
        Serial.printf("PozisyonKontrolGorevi: UYARI - Watchdog aboneligi basarisiz (%d)\n", wdt_err);
    } else {
        Serial.println("PozisyonKontrolGorevi: Watchdog'a abone olundu");
    }

    panFilter.begin(ENCODER_FILTER_WINDOW);// filtre ve kontrolcülri başlat
    tiltFilter.begin(ENCODER_FILTER_WINDOW);
    posCtrl.begin(g_posConfig);
    combiner.begin(g_combinerConfig);
    limiter.begin(g_limitConfig);
    groundLockCtrl.begin();

    extern GroundReferenceCalibration g_groundRef;
    groundLockCtrl.setGroundReference(g_groundRef);

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(POSITION_PERIOD_MS);

    uint32_t loopCount = 0;
    uint32_t lastPrintTime = 0;
    uint32_t lastWdtFeedTime = 0;

    while (true) {
        vTaskDelayUntil(&lastWakeTime, period);
        loopCount++;

        float rawPan = g_panEncoder.readAngleDegrees();//encoder okunur
        float rawTilt = g_tiltEncoder.readAngleDegrees();

        if (rawPan < 0 || rawTilt < 0) {
            g_sensorHealth.recordFailure(SensorHealth::PAN_ENCODER);
            g_sensorHealth.recordFailure(SensorHealth::TILT_ENCODER);
            continue;
        }

        g_sensorHealth.recordSuccess(SensorHealth::PAN_ENCODER);
        g_sensorHealth.recordSuccess(SensorHealth::TILT_ENCODER);

        uint32_t timestamp = micros();//encoder filtrelenir
        panFilter.addSample(rawPan, timestamp);
        tiltFilter.addSample(rawTilt, timestamp);

        float encoderPan = panFilter.getFiltered();
        float encoderTilt = tiltFilter.getFiltered();
        float encoderVelPan = panFilter.getVelocity();
        float encoderVelTilt = tiltFilter.getVelocity();


        float deltaIMUPan = 0, deltaIMUTilt = 0;//imu verisi ve deltayı al
        IMUFusion::Orientation bodyOri;

        if (xSemaphoreTake(g_sensorData.mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            deltaIMUPan = g_sensorData.deltaIMUPan;
            deltaIMUTilt = g_sensorData.deltaIMUTilt;

            bodyOri.roll = g_sensorData.bodyRoll;
            bodyOri.pitch = g_sensorData.bodyPitch;
            bodyOri.yaw = g_sensorData.bodyYaw;

            g_sensorData.encoderPan = encoderPan;
            g_sensorData.encoderTilt = encoderTilt;
            g_sensorData.encoderVelPan = encoderVelPan;
            g_sensorData.encoderVelTilt = encoderVelTilt;
            g_sensorData.encoderTimestamp = timestamp;

            xSemaphoreGive(g_sensorData.mutex);
        }

        float groundLockPan, groundLockTilt; // yer kilit hedefini günceller
        groundLockCtrl.computeGroundLockTarget(bodyOri, groundLockPan, groundLockTilt);
        g_targetMgr.setGroundLockTarget(groundLockPan, groundLockTilt);

        float currentWorldPan = encoderPan + deltaIMUPan; // dünya referanslı pozisyon hesabı
        float currentWorldTilt = encoderTilt + deltaIMUTilt;

        float targetPan, targetTilt, ffPan, ffTilt;
        g_targetMgr.getTargets(targetPan, targetTilt, ffPan, ffTilt);

        posCtrl.update(//kontrolcü güncellenir
            targetPan,
            targetTilt,
            currentWorldPan,
            currentWorldTilt,
            POSITION_PERIOD_MS / 1000.0f
        );

        float stabVelPan = stabCtrl.getVelocityCommandPan();
        float stabVelTilt = stabCtrl.getVelocityCommandTilt();

        float posVelPan = posCtrl.getVelocityCommandPan();
        float posVelTilt = posCtrl.getVelocityCommandTilt();

        combiner.update(
            posVelPan,
            posVelTilt,
            stabVelPan,
            stabVelTilt,
            ffPan,
            ffTilt,
            POSITION_PERIOD_MS / 1000.0f
        );

        float finalVelPan = combiner.getFinalVelocityPan();
        float finalVelTilt = combiner.getFinalVelocityTilt();

        limiter.enforce(
            encoderPan,
            encoderTilt,
            finalVelPan,
            finalVelTilt,
            targetPan,
            targetTilt,
            deltaIMUPan,
            deltaIMUTilt,
            POSITION_PERIOD_MS / 1000.0f
        );

        // Limit hedefi ayarladiysa guncelle
        if (limiter.isAtLimit(true, false) || limiter.isAtLimit(true, true) ||
            limiter.isAtLimit(false, false) || limiter.isAtLimit(false, true)) {
            g_targetMgr.setTarget(targetPan, targetTilt);
        }


        float stepRatePan = finalVelPan * STEPS_PER_DEGREE;//step hızına çevir ve motor sür
        float stepRateTilt = finalVelTilt * STEPS_PER_DEGREE;

        // Yon ayarla
        g_panMotor.setDirection(stepRatePan >= 0);
        g_tiltMotor.setDirection(stepRateTilt >= 0);

        // Frekans ayarla
        g_panMotor.setFrequency(fabsf(stepRatePan));
        g_tiltMotor.setFrequency(fabsf(stepRateTilt));

        if (millis() - lastWdtFeedTime > POS_TASK_WDT_FEED_MS) {//watchdog besleme
            lastWdtFeedTime = millis();
            esp_task_wdt_reset();
        }

        if (millis() - lastPrintTime > 1000) {//saniyede bir yaz
            lastPrintTime = millis();

            float errorPan, errorTilt;
            posCtrl.getError(errorPan, errorTilt);

            TargetManager::Mode mode = g_targetMgr.getMode();
            const char* modeStr = (mode == TargetManager::MODE_GROUND_LOCK) ? "YER_KILIT" : "TAKIP";

            const char* commStr = g_targetMgr.isCommHealthy() ? "OK" : "ZAMANASIMI";

            Serial.printf("[POS] Hiz: %d Hz | Mod: %s | Hab: %s | Enc: P=%.1f T=%.1f | Hedef: P=%.1f T=%.1f | Hata: P=%.2f T=%.2f\n",
                loopCount,
                modeStr,
                commStr,
                encoderPan, encoderTilt,
                targetPan, targetTilt,
                errorPan, errorTilt
            );

            loopCount = 0;
        }
    }
}

void serialTask(void *params) {//serial iletişim görevi
    Serial.println("SeriGorevi: Basladi");

    SerialProtocol protocol;
    uint8_t parseBuffer[32];
    uint8_t parseIndex = 0;
    bool inPacket = false;

    uint32_t packetsReceived = 0;
    uint32_t lastPrintTime = 0;

    while (true) {

        while (Serial.available()) {
            uint8_t byte = Serial.read();

            if (!inPacket) {
                if (byte == PACKET_HEADER1) {//header kontrlü
                    parseBuffer[0] = byte;
                    parseIndex = 1;
                    inPacket = true;
                }
            } else {
                parseBuffer[parseIndex++] = byte;

                if (parseIndex >= PACKET_SIZE) {//paket boyutu tamamlandı mı
                    SerialProtocol::GimbalCommand cmd;

                    if (protocol.parsePacket(parseBuffer, parseIndex, cmd)) {
                        g_targetMgr.handleCommand(cmd);
                        packetsReceived++;
                    } else {
                        Serial.println("[SERI] Gecersiz paket");
                    }

                    // Sonraki paket icin sifirla
                    parseIndex = 0;
                    inPacket = false;
                }

                // Guvenlik: Tampon tasarsa sifirla
                if (parseIndex >= 32) {
                    parseIndex = 0;
                    inPacket = false;
                }
            }
        }


        static uint32_t lastTelemetryTime = 0;// serialden mesaj gönder
        if (millis() - lastTelemetryTime > 100) {  // 10 Hz
            lastTelemetryTime = millis();

            SerialProtocol::TelemetryPacket telem;

            if (xSemaphoreTake(g_sensorData.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                telem.encoder_pan = g_sensorData.encoderPan;
                telem.encoder_tilt = g_sensorData.encoderTilt;
                telem.current_world_pan = g_sensorData.encoderPan + g_sensorData.deltaIMUPan;
                telem.current_world_tilt = g_sensorData.encoderTilt + g_sensorData.deltaIMUTilt;
                xSemaphoreGive(g_sensorData.mutex);
            }

            posCtrl.getError(telem.error_pan, telem.error_tilt);

            // Durum bayraklari
            telem.status_flags = 0;
            if (limiter.isAtLimit(true, false) || limiter.isAtLimit(true, true)) {
                telem.status_flags |= SerialProtocol::STATUS_PAN_AT_LIMIT;
            }
            if (limiter.isAtLimit(false, false) || limiter.isAtLimit(false, true)) {
                telem.status_flags |= SerialProtocol::STATUS_TILT_AT_LIMIT;
            }
            if (g_sensorHealth.shouldEnterSafeMode()) {
                telem.status_flags |= SerialProtocol::STATUS_SAFE_MODE;
            }

            telem.timestamp_ms = millis();

            // Olustur ve gonder
            uint8_t txBuffer[32];
            size_t len = protocol.buildTelemetry(telem, txBuffer);
            Serial.write(txBuffer, len);
        }

        if (millis() - lastPrintTime > 5000) {
            lastPrintTime = millis();
            Serial.printf("[SERI] Alinan paket: %u (son 5s)\n", packetsReceived);
            packetsReceived = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void diagnosticsTask(void *params) {//çıktıları okuyup sensor sağlığıını kontrol eder
    Serial.println("TeshisGorevi: Basladi");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));  // 10 Hz

        if (g_sensorHealth.shouldEnterSafeMode()) {
            Serial.println("\n!!!!! KRITIK: GUVENLI MODA GECILIYOR !!!!!");

            g_targetMgr.forceGroundLock();

            g_panMotor.disable();
            g_tiltMotor.disable();
            digitalWrite(MOTOR_ENABLE_PIN, HIGH);

            while (true) {
                g_buzzer.errorAlert();//hata sesi çıkarır

                digitalWrite(STATUS_LED_PIN, HIGH);
                vTaskDelay(pdMS_TO_TICKS(200));
                digitalWrite(STATUS_LED_PIN, LOW);
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        }

        static uint32_t lastCommWarning = 0;
        TargetManager::CommStatus commStatus = g_targetMgr.getCommStatus();

        if (commStatus == TargetManager::COMM_LOST) {
            // Haberlesme tamamen kayboldu - periyodik uyari yaz
            if (millis() - lastCommWarning > 5000) {
                lastCommWarning = millis();
                Serial.printf("UYARI: Haberlesme kayboldu! Son komut %ums once. Mod: YER_KILIT (guvenlik)\n",
                              g_targetMgr.getTimeSinceLastCommand());
            }

            // Haberlesme kaybini belirtmek icin durum LED'ini yakip sondur
            static bool ledState = false;
            ledState = !ledState;
            digitalWrite(STATUS_LED_PIN, ledState ? HIGH : LOW);
        }

        static uint32_t lastHealthReport = 0;
        if (millis() - lastHealthReport > 10000) {
            lastHealthReport = millis();

            Serial.println("\n===== SISTEM SAGLIGI RAPORU =====");

            Serial.println("Sensorler:");
            for (int i = 0; i < SensorHealth::SENSOR_COUNT; i++) {
                SensorHealth::Sensor s = (SensorHealth::Sensor)i;
                if (s == SensorHealth::MULTIPLEXER) continue;

                Serial.printf("  %s: %.1f%% (%s)\n",
                    g_sensorHealth.getSensorName(s),
                    g_sensorHealth.getSensorReliability(s) * 100.0f,
                    g_sensorHealth.isSensorHealthy(s) ? "OK" : "ARIZA"
                );
            }

            // TMC2209 surucu sagligi
            Serial.println("TMC2209 Suruculer:");
            Serial.printf("  Pan:  %s | Sicaklik: %s | Yuk: %s\n",
                g_panTMC.getStatus() == TMC2209Driver::STATUS_OK ? "OK" : "HATA",
                g_panTMC.isOverTemperature() ? "UYARI" : "Normal",
                g_panTMC.isOpenLoad() ? "ACIK" : "Normal");
            Serial.printf("  Tilt: %s | Sicaklik: %s | Yuk: %s\n",
                g_tiltTMC.getStatus() == TMC2209Driver::STATUS_OK ? "OK" : "HATA",
                g_tiltTMC.isOverTemperature() ? "UYARI" : "Normal",
                g_tiltTMC.isOpenLoad() ? "ACIK" : "Normal");

            // Haberlesme sagligi
            const char* commStatusStr;
            switch (g_targetMgr.getCommStatus()) {
                case TargetManager::COMM_OK:      commStatusStr = "OK"; break;
                case TargetManager::COMM_TIMEOUT: commStatusStr = "ZAMANASIMI"; break;
                case TargetManager::COMM_LOST:    commStatusStr = "KAYIP"; break;
                default:                          commStatusStr = "BILINMIYOR"; break;
            }

            Serial.println("Haberlesme:");
            Serial.printf("  Durum: %s\n", commStatusStr);
            Serial.printf("  Son komut: %ums once\n", g_targetMgr.getTimeSinceLastCommand());
            Serial.printf("  Alinan komut: %u\n", g_targetMgr.getCommandCount());
            Serial.printf("  Zaman asimi: %u\n", g_targetMgr.getTimeoutCount());

            const char* modeStr = (g_targetMgr.getMode() == TargetManager::MODE_GROUND_LOCK)
                                  ? "YER_KILIT" : "TAKIP";
            Serial.printf("Mod: %s\n", modeStr);

            Serial.printf("Bos heap: %u byte\n", ESP.getFreeHeap());

            Serial.println("================================\n");
        }
    }
}
