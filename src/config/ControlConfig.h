#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

#include <Arduino.h>

// StabilizationController parametreleri
struct StabilizationConfig {
    // İleri besleme kazançları
    // Gövde açısal hızının ne kadarının doğrudan telafi edileceğini belirler
    // 1.0 = tam telafi, 0.0 = telafi yok
    float k_feedforward_pan  = 0.9f;    // [TUNING adim 4] baslangic: 0
    float k_feedforward_tilt = 0.9f;    // [TUNING adim 4] baslangic: 0

    float k_damping_pan      = 0.0f;    // [TUNING adim 3] baslangic: 0
    float k_damping_tilt     = 0.0f;    // [TUNING adim 3] baslangic: 0

    // Jiroskop düşük geçiren filtre kesim frekansı (Hz)
    // Gürültüyü azaltır ancak gecikme ekler
    // Daha düşük değer = daha fazla filtreleme, daha fazla gecikme
    float gyro_filter_cutoff = 50.0f;
};

// PositionController parametreleri
struct PositionConfig {
    // Oransal kazançlar (Kp)
    // Hata ne kadar büyükse, düzeltme o kadar güçlü
    float kp_pan    = 20.0f;     // [TUNING adim 1]
    float ki_pan    = 0.0f;     // [TUNING adim 5] simdilik sifir
    float kd_pan    = 5.5f;     // [TUNING adim 2] simdilik sifir
    float i_max_pan = 10.0f;

    float kp_tilt    = 13.0f;    //13 [TUNING adim 1]
    float ki_tilt    = 0.0f;    // [TUNING adim 5] simdilik sifir
    float kd_tilt    = 4.5f;    //4.5 [TUNING adim 2] simdilik sifir
    float i_max_tilt = 10.0f;

    float deadzone_pan  = 2.0f;  // GROUND_LOCK — titreşimi bastırır
    float deadzone_tilt = 2.0f;

    float deadzone_pan_tracking  = 0.3f;  // TRACKING — hassas kilitleme
    float deadzone_tilt_tracking = 0.3f;
};

// CommandCombiner parametreleri
struct CombinerConfig {
    // Hız limitleri (derece/saniye)
    // Motor ve mekanik sistemin güvenli çalışma sınırları
    float max_velocity_pan  = 200.0f;   // Pan maksimum hız

    float max_velocity_tilt = 200.0f;    // Tilt maksimum hız

    // İvme limitleri (derece/saniye²) - step kaybını önler
    // Çok hızlı ivme değişimleri motorun adım kaçırmasına neden olabilir
    float max_acceleration_pan  = 500.0f;   // Pan maksimum ivme

    float max_acceleration_tilt = 400.0f;   // Tilt maksimum ivme

    // ROS2 hız ipuçlarından ileri besleme kazancı
    // Hareket tahmini için kullanılır
    float feedforward_gain = 0.5f;
};

// LimitEnforcer parametreleri
struct LimitConfig {
    // Encoder home açıları — manuel girilir, motorlar durmadayken ölçülür
    float pan_offset  = 180.0f;  // pan encoder home açısı  (0-360°)
    float tilt_offset = 180.0f;  // tilt encoder home açısı (0-360°)

    // Home'dan her iki yönde izin verilen max sapma
    float pan_range  = 30.0f;   // derece
    float tilt_range = 20.0f;   // derece

    float soft_margin = 5.0f;
    bool enforce_limits = false;  // offset değerleri ölçülüp girilene kadar kapalı
};

// IMUFusion parametreleri
struct FusionConfig {
    // Tamamlayıcı filtre katsayısı
    // Jiroskop ve ivmeölçer arasındaki güven dengesini belirler
    // Daha yüksek = jiroskopa daha fazla güven (0.95 - 0.99 arası önerilir)
    float alpha = 0.98f;

    // İvmeölçer düşük geçiren filtre kesim frekansı (Hz)
    // Titreşim gürültüsünü filtreler
    float accel_filter_cutoff = 5.0f;

    // Jiroskop sapma tahmini zaman sabiti (saniye)
    // Uzun vadeli sapma düzeltmesi için kullanılır
    float bias_time_constant = 10.0f;
};

// SensorHealth parametreleri
struct HealthConfig {
    // Güvenli moda geçmeden önce maksimum ardışık hata sayısı
    // Bu sayıyı aşan ardışık hatalar sensör arızası olarak değerlendirilir
    uint32_t max_consecutive_failures = 10;

    // Çalışır durumda kalmak için minimum başarı oranı (0.0 - 1.0)
    // Bu oranın altına düşerse sistem güvenli moda geçer
    float min_success_rate = 0.90f;

    // Sağlık kontrolü aralığı (milisaniye)
    uint32_t check_interval_ms = 1000;
};


extern StabilizationConfig g_stabConfig;    // Stabilizasyon yapılandırması

extern PositionConfig g_posConfig;          // Konum kontrolü yapılandırması

extern CombinerConfig g_combinerConfig;     // Komut birleştirici yapılandırması

extern LimitConfig g_limitConfig;           // Limit uygulayıcı yapılandırması

extern FusionConfig g_fusionConfig;         // Sensör füzyon yapılandırması

extern HealthConfig g_healthConfig;         // Sensör sağlık yapılandırması

#endif
