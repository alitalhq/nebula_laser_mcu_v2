#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include <Arduino.h>

// Step Motor Pinleri
#define PAN_STEP_PIN        1   // Pan motoru step pini
#define PAN_DIR_PIN         2   // Pan motoru yön pini
#define TILT_STEP_PIN       4   // Tilt motoru step pini
#define TILT_DIR_PIN        5   // Tilt motoru yön pini
#define MOTOR_ENABLE_PIN    6   // Ortak enable pini (LOW = etkin)

// TMC2209 UART Pinleri
// Her sürücü ayrı HardwareSerial kullanır (MS1/MS2 bağlı değil = adres 0x00)
#define TMC_PAN_TX_PIN      17  // Pan TMC2209 UART TX pini
#define TMC_PAN_RX_PIN      18  // Pan TMC2209 UART RX pini
#define TMC_TILT_TX_PIN     19  // Tilt TMC2209 UART TX pini
#define TMC_TILT_RX_PIN     20  // Tilt TMC2209 UART RX pini
#define TMC_UART_BAUD       115200  // TMC2209 UART hızı
#define TMC_DRIVER_ADDRESS  0x00    // MS1/MS2 bağlı değil = dahili pull-down = 0b00

// TMC2209 Sürücü Parametreleri
#define TMC_RSENSE          0.11f   // Akım algılama direnci (ohm)
#define TMC_RMS_CURRENT     1200    // 17HS4401: 1.7A nominal, 1200mA RMS
#define TMC_MICROSTEPS      32      // Mikro adım (UART ile ayarlanır)
#define TMC_TPWMTHRS        0       // StealthChop eşiği (0 = her zaman StealthChop)
#define TMC_IHOLD           16      // Bekleme akımı (0-31, ~%50)
#define TMC_IHOLDDELAY      6       // Hareket -> bekleme geçiş gecikmesi (0-15)

// Lazer Kontrolü
#define LASER_PIN           7

// Donanımsal I2C Veri Yolları (IMU'lar için - zaman-kritik)
#define I2C0_SDA            8   // Gövde IMU - SDA pini
#define I2C0_SCL            9   // Gövde IMU - SCL pini
#define I2C1_SDA            10  // Kafa IMU - SDA pini
#define I2C1_SCL            11  // Kafa IMU - SCL pini

// Yazılımsal I2C Pinleri (Encoder'lar için - zaman-kritik değil)
#define SOFT_I2C_PAN_SDA    12  // Pan Encoder - SDA pini
#define SOFT_I2C_PAN_SCL    13  // Pan Encoder - SCL pini
#define SOFT_I2C_TILT_SDA   14  // Tilt Encoder - SDA pini
#define SOFT_I2C_TILT_SCL   15  // Tilt Encoder - SCL pini

// Buzzer Yapılandırması
#define BUZZER_PIN      39     // Buzzer kontrol pini
#define BUZZER_PWM_CH   0      // PWM kanalı (0-15 arası)
#define BUZZER_FREQ     2000   // Varsayılan frekans (Hz)

// Durum LED'i
#define STATUS_LED_PIN      LED_BUILTIN //Dahili

// I2C Adresleri
#define BMI160_ADDR         0x68    // BMI160 IMU varsayılan adresi
#define AS5600_ADDR         0x36    // AS5600 encoder sabit adresi (değiştirilemez)

// I2C veri yolu hızları
#define HARDWARE_I2C_SPEED  400000  // IMU'lar için 400 kHz (hızlı mod)
#define SOFTWARE_I2C_SPEED  100000  // Encoder'lar için 100 kHz (standart mod)
#define I2C_TIMEOUT_MS      5       // Maksimum bekleme süresi: 5 ms

// Motor Parametreleri
#define MICROSTEPS              32  // Mikro adım ayarı
#define STEPS_PER_REVOLUTION    200 // Adım sayısı
#define REDUCTION_RATIO         1.0f // Redüktör oranı

// Derece başına step sayısı hesaplaması
#define STEPS_PER_DEGREE        ((STEPS_PER_REVOLUTION * MICROSTEPS * REDUCTION_RATIO) / 360.0f)

// Mekanik Limitler
#define DEFAULT_PAN_MIN         -30.0f   //Mekanik netleşince değerler değişcek
#define DEFAULT_PAN_MAX         30.0f 
#define DEFAULT_TILT_MIN        -20.0f
#define DEFAULT_TILT_MAX        20.0f 
#define SOFT_LIMIT_MARGIN       5.0f    // Yumuşak limit marjı

// Haberleşme Sabitleri
#define SERIAL_BAUD_RATE        921600
#define PACKET_HEADER1          0xAA    // Header 1
#define PACKET_HEADER2          0xFF    // Header 2
#define PACKET_SIZE             22

// Kontrol Döngüsü Frekansları
#define IMU_READ_FREQ_HZ        1000
#define STABILIZATION_FREQ_HZ   500
#define POSITION_FREQ_HZ        200
#define DIAGNOSTICS_FREQ_HZ     10

// Periyot hesaplamaları (milisaniye cinsinden)
#define IMU_READ_PERIOD_MS      (1000 / IMU_READ_FREQ_HZ)
#define STABILIZATION_PERIOD_MS (1000 / STABILIZATION_FREQ_HZ)
#define POSITION_PERIOD_MS      (1000 / POSITION_FREQ_HZ)
#define DIAGNOSTICS_PERIOD_MS   (1000 / DIAGNOSTICS_FREQ_HZ)

// FreeRTOS Öncelik Katsayıları
#define IMU_READ_TASK_PRIORITY      6   // En yüksek öncelik
#define STABILIZATION_TASK_PRIORITY 5
#define POSITION_TASK_PRIORITY      4
#define SERIAL_TASK_PRIORITY        3
#define DIAGNOSTICS_TASK_PRIORITY   1

// FreeRTOS Stack Boyutları
#define IMU_READ_TASK_STACK         3072
#define STABILIZATION_TASK_STACK    4096
#define POSITION_TASK_STACK         4096
#define SERIAL_TASK_STACK           3072
#define DIAGNOSTICS_TASK_STACK      2048

// Çekirdek atamaları
// Çekirdek 0: Zaman-kritik görevler (IMU, stabilizasyon, konum)
// Çekirdek 1: Düşük öncelikli görevler (seri, tanılama)
#define IMU_READ_TASK_CORE          0
#define STABILIZATION_TASK_CORE     0
#define POSITION_TASK_CORE          0
#define SERIAL_TASK_CORE            1
#define DIAGNOSTICS_TASK_CORE       1

// BMI160 İvmeölçer Aralığı Register Değerleri:
//   0x03 = ±2g,  0x05 = ±4g,  0x08 = ±8g,  0x0C = ±16g
// ±4g gimbal uygulamaları için iyi bir denge sağlar
#define BMI160_ACCEL_RANGE      0x05    // ±4g seçili

// BMI160 Jiroskop Aralığı Register Değerleri:
//   0x00 = ±2000°/s, 0x01 = ±1000°/s, 0x02 = ±500°/s, 0x03 = ±250°/s, 0x04 = ±125°/s
// ±500°/s çoğu gimbal hareketi için yeterli çözünürlük sağlar
#define BMI160_GYRO_RANGE       0x02    // ±500°/s seçili

// BMI160 Çıkış Veri Hızı (Hz)
#define BMI160_ODR              400

// AS5600 Encoder özellikleri
#define ENCODER_BITS            12      // 12-bit çözünürlük
#define ENCODER_COUNTS          4096    // 2^12 = 4096 sayım/tur
#define ENCODER_FILTER_WINDOW   5       // Median filtre pencere boyutu

// Görev Watchdog Zamanlayıcısı
#define WATCHDOG_TIMEOUT_SEC    3       // Watchdog zaman aşımı (saniye)
#define WATCHDOG_PANIC          true    // true = zaman aşımında sıfırla, false = sadece uyar

// Bireysel görev watchdog besleme aralıkları (WATCHDOG_TIMEOUT_SEC'den küçük olmalı)
#define IMU_TASK_WDT_FEED_MS    500     // IMU görevinde her 500ms'de watchdog besle
#define POS_TASK_WDT_FEED_MS    500     // Konum görevinde her 500ms'de watchdog besle

// Bu süre boyunca ROS2 komutu alınmazsa, yer-kilit moduna dön
#define ROS2_COMMAND_TIMEOUT_MS     2000    // 2 saniye

// Bu süre boyunca geçerli seri veri alınmazsa, haberleşme kaybı sayılır
#define SERIAL_HEARTBEAT_TIMEOUT_MS 5000    // 5 saniye

// Mod değişikliği bildirimleri arasındaki minimum süre (buzzer spam'ini önler)
#define MODE_CHANGE_DEBOUNCE_MS     1000    // 1 saniye

#endif
