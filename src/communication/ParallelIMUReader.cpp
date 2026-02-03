#include "ParallelIMUReader.h"

ParallelIMUReader::ParallelIMUReader()
    : _bodyIMU(nullptr)
    , _headIMU(nullptr)
    , _bodyStartSem(nullptr)
    , _headStartSem(nullptr)
    , _bodyDoneSem(nullptr)
    , _headDoneSem(nullptr)
    , _dataMutex(nullptr)
    , _bodyTaskHandle(nullptr)
    , _headTaskHandle(nullptr)
    , _running(false)
    , _stopRequested(false)
    , _successCount(0)
    , _failureCount(0)
    , _timeoutCount(0)
{
    _data.bodyValid = false;
    _data.headValid = false;
    _data.timestamp_us = 0;
}

ParallelIMUReader::~ParallelIMUReader() {
    end();
}

bool ParallelIMUReader::begin(IMUDriver *bodyIMU, IMUDriver *headIMU) {
    if (_running) {
        Serial.println("ParallelIMUReader: Zaten calisiyor");
        return false;
    }

    if (!bodyIMU || !headIMU) {
        Serial.println("ParallelIMUReader: Gecersiz IMU isaretcileri");
        return false;
    }

    _bodyIMU = bodyIMU;
    _headIMU = headIMU;
    _stopRequested = false;

    _bodyStartSem = xSemaphoreCreateBinary();  // Govde baslama sinyali
    _headStartSem = xSemaphoreCreateBinary();  // Kafa baslama sinyali
    _bodyDoneSem = xSemaphoreCreateBinary();   // Govde tamamlandi sinyali
    _headDoneSem = xSemaphoreCreateBinary();   // Kafa tamamlandi sinyali
    _dataMutex = xSemaphoreCreateMutex();      // Veri erisim korumasi

    if (!_bodyStartSem || !_headStartSem || !_bodyDoneSem || !_headDoneSem || !_dataMutex) {
        Serial.println("ParallelIMUReader: Semaphore olusturulamadi");
        end();
        return false;
    }

    _data.bodyValid = false;
    _data.headValid = false;
    _data.timestamp_us = 0;

    // Kalici govde okuma gorevini olustur
    // xTaskCreatePinnedToCore: Belirli bir CPU cekirdegine sabitler
    BaseType_t result1 = xTaskCreatePinnedToCore(
        bodyReadTaskFunc,      // Gorev fonksiyonu
        "BodyIMUTask",         // Gorev adi (hata ayiklama icin)
        TASK_STACK_SIZE,       // Yigin boyutu (byte)
        this,                  // Parametre (this isaretcisi)
        TASK_PRIORITY,         // Oncelik (7 = yuksek)
        &_bodyTaskHandle,      // Tutamac ciktisi
        TASK_CORE              // CPU cekirdegi (0)
    );

    if (result1 != pdPASS) {
        Serial.println("ParallelIMUReader: Govde gorevi olusturulamadi");
        end();
        return false;
    }

    BaseType_t result2 = xTaskCreatePinnedToCore(
        headReadTaskFunc,
        "HeadIMUTask",
        TASK_STACK_SIZE,
        this,
        TASK_PRIORITY,
        &_headTaskHandle,
        TASK_CORE
    );

    if (result2 != pdPASS) {
        Serial.println("ParallelIMUReader: Kafa gorevi olusturulamadi");
        end();
        return false;
    }

    _running = true;
    Serial.println("ParallelIMUReader: Kalici gorevler ile baslatildi");

    return true;
}


void ParallelIMUReader::end() {
    // Gorevlere durdurma istegi gonder
    _stopRequested = true;
    _running = false;

    if (_bodyStartSem) xSemaphoreGive(_bodyStartSem);
    if (_headStartSem) xSemaphoreGive(_headStartSem);

    vTaskDelay(pdMS_TO_TICKS(10));

    if (_bodyTaskHandle) {
        vTaskDelete(_bodyTaskHandle);
        _bodyTaskHandle = nullptr;
    }
    if (_headTaskHandle) {
        vTaskDelete(_headTaskHandle);
        _headTaskHandle = nullptr;
    }

    if (_bodyStartSem) {
        vSemaphoreDelete(_bodyStartSem);
        _bodyStartSem = nullptr;
    }
    if (_headStartSem) {
        vSemaphoreDelete(_headStartSem);
        _headStartSem = nullptr;
    }
    if (_bodyDoneSem) {
        vSemaphoreDelete(_bodyDoneSem);
        _bodyDoneSem = nullptr;
    }
    if (_headDoneSem) {
        vSemaphoreDelete(_headDoneSem);
        _headDoneSem = nullptr;
    }
    if (_dataMutex) {
        vSemaphoreDelete(_dataMutex);
        _dataMutex = nullptr;
    }

    _bodyIMU = nullptr;
    _headIMU = nullptr;
}

bool ParallelIMUReader::triggerRead() {
    if (!_running) {
        return false;
    }

    // Okumalari tetiklemeden ONCE gecerlilik bayraklarini sifirla
    // Bu, mutex zaman asimi durumunda eski 'true' degerlerinin kalmamasi icin
    if (xSemaphoreTake(_dataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        _data.bodyValid = false;
        _data.headValid = false;
        xSemaphoreGive(_dataMutex);
    }
    // Not: Mutex zaman asiminda bayraklar eski kalabilir, ama bu nadir durumdur
    // ve gorevler yine de guncelleyecektir

    // Her iki goreve baslama sinyali gonder
    // xSemaphoreGive: Binary semaphore'u 1'e ayarlar, bekleyen gorevi uyandirir
    xSemaphoreGive(_bodyStartSem);
    xSemaphoreGive(_headStartSem);

    return true;
}

bool ParallelIMUReader::waitForCompletion(uint32_t timeout_ms) {
    if (!_running) {
        return false;
    }

    TickType_t timeout = pdMS_TO_TICKS(timeout_ms);
    TickType_t startTick = xTaskGetTickCount();

    bool bodyOk = (xSemaphoreTake(_bodyDoneSem, timeout) == pdTRUE);

    TickType_t elapsed = xTaskGetTickCount() - startTick;
    TickType_t remaining = (elapsed < timeout) ? (timeout - elapsed) : 0;

    bool headOk = (xSemaphoreTake(_headDoneSem, remaining) == pdTRUE);

    if (!bodyOk || !headOk) {
        _timeoutCount++;
        return false;
    }

    return true;
}

bool ParallelIMUReader::getData(IMUDataPair &data) {
    if (!_running || !_dataMutex) {
        return false;
    }

    if (xSemaphoreTake(_dataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        data = _data;
        xSemaphoreGive(_dataMutex);
        return true;
    }

    return false;
}

bool ParallelIMUReader::readBlocking(IMUDataPair &data, uint32_t timeout_ms) {
    if (!triggerRead()) {
        _failureCount++;
        return false;
    }

    if (!waitForCompletion(timeout_ms)) {
        _failureCount++;
        return false;
    }

    if (!getData(data)) {
        _failureCount++;
        return false;
    }

    if (data.isValid()) {
        _successCount++;
        return true;
    }

    _failureCount++;
    return false;
}

void ParallelIMUReader::resetStats() {
    _successCount = 0;
    _failureCount = 0;
    _timeoutCount = 0;
}

void ParallelIMUReader::bodyReadTaskFunc(void *param) {
    ParallelIMUReader *reader = static_cast<ParallelIMUReader*>(param);

    Serial.println("GovdeIMUGorev: Basladi (kalici)");

    while (!reader->_stopRequested) {
        // Tetikleme sinyali bekle (suresiz bloke olur)
        if (xSemaphoreTake(reader->_bodyStartSem, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (reader->_stopRequested) {
            break;
        }

        IMUDriver::IMUData bodyData;
        bool success = reader->_bodyIMU->read(bodyData);

        if (xSemaphoreTake(reader->_dataMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            reader->_data.bodyData = bodyData;
            reader->_data.bodyValid = success && bodyData.valid;
            xSemaphoreGive(reader->_dataMutex);
        }

        xSemaphoreGive(reader->_bodyDoneSem);
    }

    Serial.println("GovdeIMUGorev: Cikiyor");
    vTaskDelete(NULL);  // Gorevi sil
}

void ParallelIMUReader::headReadTaskFunc(void *param) {
    ParallelIMUReader *reader = static_cast<ParallelIMUReader*>(param);

    Serial.println("KafaIMUGorev: Basladi (kalici)");

    while (!reader->_stopRequested) {
        if (xSemaphoreTake(reader->_headStartSem, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (reader->_stopRequested) {
            break;
        }

        IMUDriver::IMUData headData;
        bool success = reader->_headIMU->read(headData);

        if (xSemaphoreTake(reader->_dataMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            reader->_data.headData = headData;
            reader->_data.headValid = success && headData.valid;
            reader->_data.timestamp_us = micros();
            xSemaphoreGive(reader->_dataMutex);
        }

        xSemaphoreGive(reader->_headDoneSem);
    }

    Serial.println("KafaIMUGorev: Cikiyor");
    vTaskDelete(NULL);
}
