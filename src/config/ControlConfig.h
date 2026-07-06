#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

#include <Arduino.h>

// StabilizationController parametreleri
struct StabilizationConfig {
    // İleri besleme kazançları
    // Gövde açısal hızının ne kadarının doğrudan telafi edileceğini belirler
    // 1.0 = tam telafi, 0.0 = telafi yok
    float k_feedforward_pan  = 0.6f;    // eski calisan deger (IMU duzelince geri yuklendi)
    float k_feedforward_tilt = 0.6f;    // eski calisan deger

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
    float kp_pan    = 14.0f;    // eski calisan deger (IMU duzelince geri yuklendi)
    float ki_pan    = 0.01f;
    float kd_pan    = 6.5f;     // eski calisan deger
    float i_max_pan = 10.0f;

    float kp_tilt    = 14.0f;   // eski calisan deger (IMU duzelince geri yuklendi)
    float ki_tilt    = 0.01f;
    float kd_tilt    = 6.5f;    // eski calisan deger
    float i_max_tilt = 10.0f;

    float deadzone_pan  = 2.0f;  // GROUND_LOCK — titreşimi bastırır
    float deadzone_tilt = 2.0f;

    float deadzone_pan_tracking  = 0.1f;  // TRACKING — hassas kilitleme
    float deadzone_tilt_tracking = 0.1f;

    // Level trim — kafa IMU "0"i gercek yatay degilse duzeltme ofseti (derece)
    // Kamera GERCEKTEN yatayken [POS] logundaki world degeri ne ise buraya girilir
    float level_trim_pan  = 0.0f;   // headRoll = bu deger → gercek yatay
    float level_trim_tilt = 0.0f;   // headPitch = bu deger → gercek yatay
};

// CommandCombiner parametreleri
struct CombinerConfig {
    // Hız limitleri (derece/saniye)
    // Motor ve mekanik sistemin güvenli çalışma sınırları
    float max_velocity_pan  = 200.0f;   // eski calisan deger (IMU duzelince geri yuklendi)

    float max_velocity_tilt = 200.0f;   // eski calisan deger

    // İvme limitleri (derece/saniye²) - step kaybını önler
    // Çok hızlı ivme değişimleri motorun adım kaçırmasına neden olabilir
    float max_acceleration_pan  = 500.0f;   // eski calisan deger

    float max_acceleration_tilt = 400.0f;   // eski calisan deger

    // ROS2 hız ipuçlarından ileri besleme kazancı
    // Hareket tahmini için kullanılır
    float feedforward_gain = 0.5f;
};

// LimitEnforcer parametreleri
struct LimitConfig {
    // Mekanik limitlerin ORTASI = home (limiter'in gordugu sw/invert-sonrasi koordinat)
    // Olculen ham AS5600 sayimlarindan hesaplandi (2026-07-05):
    //   PAN  (invert YOK): min=1170(102.83°) max=571(50.19°) -> offset=76.51 range=26.32
    //   TILT (invert VAR): ham min=3932 max=658; sw=(4095-ham) -> 14.33°/302.09°
    //                      wrap-ortalama -> offset=338.21 range=36.12 (toplam ~72° hareket)
    float pan_offset  = 76.51f;  // pan encoder home açısı  (0-360°)
    float tilt_offset = 338.21f; // tilt encoder home açısı (0-360°)

    // Home'dan her iki yönde izin verilen max sapma
    float pan_range  = 26.32f;  // derece
    float tilt_range = 36.12f;  // derece

    float soft_margin = 5.0f;
    bool enforce_limits = false;  // olculen mekanik limitler girildi
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
