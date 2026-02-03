#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <Arduino.h>

namespace MathUtils {//gimbalın sık kullandığı matematiksel fonksiyonlar

    inline float wrapAngle360(float angle) {//Aciyi [0, 360) araliginda tutar
        angle = fmodf(angle, 360.0f);

        if (angle < 0.0f) {
            angle += 360.0f;
        }
        return angle;
    }

    inline float wrapAngle180(float angle) {//Aciyi [-180, 180) araliginda tutar
        angle = fmodf(angle + 180.0f, 360.0f);

        if (angle < 0.0f) {
            angle += 360.0f;
        }
        return angle - 180.0f;
    }

    inline float angularError(float target, float current) {//en kısa açısal yolu hesaplar
        float error = target - current;
        return wrapAngle180(error);
    }

    inline float clamp(float value, float min_val, float max_val) { //değeri max min değerleri içinde tutar
        if (value < min_val) return min_val;
        if (value > max_val) return max_val;
        return value;
    }

    inline float lowPassFilter(float current, float new_sample, float alpha) {//Dusuk geciren filtre (ustel hareketli ortalama)
        return alpha * new_sample + (1.0f - alpha) * current;//dalgalanmalardan korur alpha Karistirma faktoru (0-1) 0 = tamamen eski (filtreleme yok) 1 = tamamen yeni (gecikme yok)
    }

    inline bool isValid(float value) { // float değerinin NaN ve Infinity olmadığını teyit eder
        return !isnan(value) && !isinf(value);
    }


    inline float degToRad(float degrees) { // derece to radyan
        return degrees * (M_PI / 180.0f);
    }

    inline float radToDeg(float radians) { // radyan to derece
        return radians * (180.0f / M_PI);
    }

    void eulerToRotationMatrix(float roll, float pitch, float yaw, float R[3][3]);// Euler (roll, pitch, yaw) açılarını ZYX sırasıyla birleştirerek 3x3 rotasyon matrisi üretir.

    void rotateVector(const float v[3], const float R[3][3], float out[3]);// Verilen 3D vektörü, 3x3 rotasyon matrisi ile döndürerek yeni koordinat sistemine taşır.

}

#endif
