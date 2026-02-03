#ifndef PARALLEL_IMU_READER_H
#define PARALLEL_IMU_READER_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "../hardware/IMUDriver.h"

class ParallelIMUReader {
public:

    struct IMUDataPair {
        IMUDriver::IMUData bodyData;  // Govde IMU verisi
        IMUDriver::IMUData headData;  // Kafa IMU verisi
        uint32_t timestamp_us;        // Okuma zamani (mikrosaniye)
        bool bodyValid;               // Govde verisi gecerli mi?
        bool headValid;               // Kafa verisi gecerli mi?

        // Tam gecerlilik kontrolu: her iki IMU da basarili olmali
        bool isValid() const { return bodyValid && headValid; }
    };

    ParallelIMUReader();
    ~ParallelIMUReader();

    bool begin(IMUDriver *bodyIMU, IMUDriver *headIMU);

    void end();

    bool triggerRead();

    bool waitForCompletion(uint32_t timeout_ms = 2);

    bool getData(IMUDataPair &data);

    bool readBlocking(IMUDataPair &data, uint32_t timeout_ms = 2);

    bool isRunning() const { return _running; }

    uint32_t getSuccessCount() const { return _successCount; }
    uint32_t getFailureCount() const { return _failureCount; }
    uint32_t getTimeoutCount() const { return _timeoutCount; }
    void resetStats();

private:
    // IMU surucu isaretcileri (sahiplik bu sinifa ait degil)
    IMUDriver *_bodyIMU;
    IMUDriver *_headIMU;

    // Paylasilmis veri yapisi
    IMUDataPair _data;

    // Senkronizasyon primitifleri
    SemaphoreHandle_t _bodyStartSem;    // Govde gorevine baslama sinyali
    SemaphoreHandle_t _headStartSem;    // Kafa gorevine baslama sinyali
    SemaphoreHandle_t _bodyDoneSem;     // Govde gorevi tamamlandi sinyali
    SemaphoreHandle_t _headDoneSem;     // Kafa gorevi tamamlandi sinyali
    SemaphoreHandle_t _dataMutex;       // _data yapisini korur

    // Gorev tutamaclari
    TaskHandle_t _bodyTaskHandle;
    TaskHandle_t _headTaskHandle;

    // Durum
    volatile bool _running;        // Gorevler calisiyor mu?
    volatile bool _stopRequested;  // Durdurma istendi mi?

    // Istatistikler
    volatile uint32_t _successCount;  // Basarili okuma sayaci
    volatile uint32_t _failureCount;  // Basarisiz okuma sayaci
    volatile uint32_t _timeoutCount;  // Zaman asimi sayaci

    // Gorev fonksiyonlari (FreeRTOS ile calismak icin static)
    static void bodyReadTaskFunc(void *param);
    static void headReadTaskFunc(void *param);

    // Gorev yapilandirmasi
    static constexpr uint32_t TASK_STACK_SIZE = 2048;   // Gorev yigin boyutu (byte)
    static constexpr UBaseType_t TASK_PRIORITY = 7;     // Yuksek oncelik (0-24)
    static constexpr BaseType_t TASK_CORE = 0;          // Core 0: zaman-kritik isler
};

#endif
