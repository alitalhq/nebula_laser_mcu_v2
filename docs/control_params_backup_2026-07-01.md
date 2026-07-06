# Kontrol & Sensör Parametreleri Referansı — 2026-07-01

Tüm ayarlanabilir sayılar ve mevcut (= orijinal working) değerleri.
Kaynaklar: `src/config/ControlConfig.h`, `src/config/HardwareConfig.h`, `src/tasks/Tasks.cpp`.

---

## 1) PID / Pozisyon (posCtrl — açı hatası düzeltme)
| Parametre | Pan | Tilt | Ne yapar |
|---|---|---|---|
| kp | 20.0 | 13.0 | Oransal: hata büyükse ne kadar sert çeker |
| ki | 0.0 | 0.0 | İntegral: kalıcı ofseti siler (şimdilik 0) |
| kd | 8.0 | 7.0 | Türev: overshoot/salınımı söner |
| i_max | 10.0 | 10.0 | İntegral doyma sınırı |
| deadzone (GROUND_LOCK) | 3.5 | 3.5 | Bu açının altında düzeltme yok (titreşim bastırır) |
| deadzone_tracking (TRACKING) | 0.3 | 0.3 | Nişan modunda hassas |

## 2) Stabilizasyon (stabCtrl — gyro ileri besleme)
| Parametre | Pan | Tilt | Ne yapar |
|---|---|---|---|
| k_feedforward | 0.6 | 0.6 | Gövde hareketini önceden telafi (1=tam, 0=kapalı) |
| k_damping | 0.0 | 0.0 | Gyro sönümleme kazancı |
| gyro_filter_cutoff | 50.0 Hz (ortak) | | Gyro alçak geçiren; gürültü↓ ama gecikme↑ |

## 3) Combiner (hız/ivme limitleri)
| Parametre | Pan | Tilt | Ne yapar |
|---|---|---|---|
| max_velocity | 200.0 °/s | 200.0 °/s | Motor max dönüş hızı |
| max_acceleration | 500.0 °/s² | 400.0 °/s² | Max ivme (adım kaybını önler) |
| feedforward_gain | 0.5 (ortak) | | ROS2 hız ipuçlarından ileri besleme |

## 4) Tasks.cpp — hardcoded gyro sönümleme
| Parametre | Değer | Ne yapar |
|---|---|---|
| KD_PAN_GYRO | 0.1 | Head gyro hızını doğrudan fren olarak ekler (pan) |
| KD_TILT_GYRO | 0.1 | Aynısı (tilt) |

---

## 5) IMU Füzyon — "IMU verisine ne kadar güvenilecek" ⭐
Kaynak: `FusionConfig` (`ControlConfig.h`) + `IMUFusion.cpp`

| Parametre | Değer | Ne yapar |
|---|---|---|
| **alpha** | **0.98** | **Tamamlayıcı filtre.** `açı = alpha·(gyro) + (1−alpha)·(accel)`. 0.98 = **%98 gyro'ya, %2 ivmeölçere güven**. Yüksek → pürüzsüz ama uzun vadede kayar; düşük → gürültülü ama kaymaz. Clamp: [0.9, 0.999] |
| accel_filter_cutoff | 5.0 Hz | İvmeölçer alçak geçiren; titreşim gürültüsünü süzer |
| bias_time_constant | 10.0 s | Gyro sapma (drift) tahmini zaman sabiti |

> Not: `alpha` hem `FusionConfig`'te (0.98) hem `IMUFusion` yapıcısında default (0.98) var; kullanılan değer `g_fusionConfig.alpha`.

## 6) Donanım Sensör Ayarları
Kaynak: `HardwareConfig.h`

| Parametre | Değer | Ne yapar |
|---|---|---|
| BMI160_ACCEL_RANGE | ±4g (0x05) | İvmeölçer ölçüm aralığı |
| BMI160_GYRO_RANGE | ±500°/s (0x02) | Jiroskop ölçüm aralığı |
| BMI160_ODR | 400 Hz | IMU çıkış veri hızı |
| ENCODER_FILTER_WINDOW | 5 | Encoder medyan filtre pencere boyutu |
| HARDWARE_I2C_SPEED | 400 kHz | IMU I2C hızı |
| SOFTWARE_I2C_SPEED | 100 kHz | Encoder I2C hızı |
| I2C_TIMEOUT_MS | 5 ms | Maks I2C bekleme |

## 7) Limitler (LimitConfig)
| Parametre | Değer |
|---|---|
| pan_offset / tilt_offset | 180.0 / 180.0 (⚠ gerçek encoder ~303/18 ile uyuşmuyor) |
| pan_range / tilt_range | 30.0 / 20.0 |
| soft_margin | 5.0 |
| enforce_limits | false |

## 8) TMC2209 Akım (HardwareConfig.h)
| Parametre | Değer |
|---|---|
| TMC_RMS_CURRENT | 1200 mA (IRUN, CS=31'de ~1.05A capli) |
| TMC_IHOLD | 16 (~0.56A durma akımı) |
| TMC_IHOLDDELAY | 6 |
| TMC_MICROSTEPS | 32 (UART ile ayarlanır — UART yoksa MS pinleri 1/2 verir!) |
| TMC_RSENSE | 0.22 Ω |
