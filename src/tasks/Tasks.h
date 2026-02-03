#ifndef TASKS_H
#define TASKS_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"


/*
 * IMU okuma gorevi (1000 Hz - Core 0)
 * Her iki IMU'yu paralel olarak okur ve fusion filtrelerini gunceller.
 * En yuksek frekansta calisan gorevdir cunku stabilizasyon dogrulugu
 * yuksek ornekleme hizina baglidir.
 */
void imuReadTask(void *params);


/*
 * Stabilizasyon gorevi (500 Hz - Core 0)
 * IMU verilerini kullanarak govde titresimlerini telafi eder.
 * Ileri besleme kontrolu ile bozuculara onceden tepki verir.
 */
void stabilizationTask(void *params);


/*
 * Pozisyon kontrol gorevi (200 Hz - Core 0)
 * Encoder'lari okur, dunya referansli pozisyonu hesaplar,
 * hedef takibi yapar ve motor komutlarini uretir.
 */
void positionControlTask(void *params);


/*
 * Seri haberlesme gorevi (Asenkron - Core 1)
 * ROS2'den gelen komutlari isler ve telemetri gonderir.
 * Dusuk oncelikli gorevdir, kontrol gorevlerini engellemez.
 */
void serialTask(void *params);


/*
 * Teshis gorevi (10 Hz - Core 1)
 * Sistem sagligini izler, hata durumlarini tespit eder,
 * periyodik durum raporlari olusturur.
 */
void diagnosticsTask(void *params);


/*
 * Sensor verilerinin paylasilmis tamponu
 *
 * Bu yapi, farkli gorevlerin urettigi ve tuketigi sensor verilerini
 * merkezi bir yerde toplar. Mutex ile korunur.
 *
 * Veri akisi:
 * - IMUReadTask yazar: IMU oryantasyonlari, IMU deltasi
 * - PositionControlTask yazar: encoder okumalari
 * - StabilizationTask okur: IMU ve encoder verileri
 * - SerialTask okur: telemetri icin tum veriler
 */

struct SharedSensorData {
    // IMU oryantasyonlari (derece cinsinden)
    // Govde IMU: drone govdesine bagli
    float bodyRoll;   // X ekseni etrafinda donme (yan yatma)
    float bodyPitch;  // Y ekseni etrafinda donme (one/arkaya egim)
    float bodyYaw;    // Z ekseni etrafinda donme (yonelim)

    // Kafa IMU: gimbal kafasina bagli
    float headRoll;
    float headPitch;
    float headYaw;

    // IMU deltasi (govde - kafa farki)
    // Stabilizasyon kontrolu icin kritik
    float deltaIMUPan;   // Yaw deltasi (pan ekseni)
    float deltaIMUTilt;  // Pitch deltasi (tilt ekseni)

    // Encoder okumalari (derece cinsinden)
    float encoderPan;    // Pan encoder konumu [0, 360)
    float encoderTilt;   // Tilt encoder konumu [0, 360)

    // Encoder hizlari (derece/saniye)
    // Filtrelenmis degerler (gurultu azaltilmis)
    float encoderVelPan;
    float encoderVelTilt;

    // Zaman damgalari (mikrosaniye)
    // Veri tazeligi kontrolu icin
    uint32_t imuTimestamp;      // Son IMU guncelleme zamani
    uint32_t encoderTimestamp;  // Son encoder guncelleme zamani

    // Thread-safety icin mutex
    // Her erisimde alinmali ve birakilmali
    SemaphoreHandle_t mutex;
};

// Global paylasilmis sensor verisi ornegi
extern SharedSensorData g_sensorData;

#endif
