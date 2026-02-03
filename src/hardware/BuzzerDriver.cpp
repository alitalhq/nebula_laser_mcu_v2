#include "BuzzerDriver.h"

BuzzerDriver::BuzzerDriver()
    : _pin(39)
    , _pwmChannel(0)
    , _defaultFrequency(2000)
    , _active(false)
{
}

bool BuzzerDriver::begin(uint8_t pin, uint8_t pwmChannel, uint16_t frequency) {
    _pin = pin;
    _pwmChannel = pwmChannel;
    _defaultFrequency = frequency;

    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);


    ledcSetup(_pwmChannel, _defaultFrequency, 8);
    ledcAttachPin(_pin, _pwmChannel);
    ledcWrite(_pwmChannel, 0);  // Sessiz başla

    Serial.printf("BuzzerDriver: GPIO %d, PWM kanal %d, Frekans %d Hz ile baslatildi\n",
                  _pin, _pwmChannel, _defaultFrequency);

    return true;
}

void BuzzerDriver::tone(uint16_t frequency) {// ton kontrolu
    if (frequency == 0) {
        frequency = _defaultFrequency;
    }

    ledcSetup(_pwmChannel, frequency, 8);
    ledcAttachPin(_pin, _pwmChannel);


    ledcWrite(_pwmChannel, 64); //buradaki sayı duty cycle (0-255) ses seviyesini buradan ayarla

    _active = true;
}

void BuzzerDriver::noTone() { //susturur
    ledcWrite(_pwmChannel, 0);
    _active = false;
}


void BuzzerDriver::beep(uint16_t duration_ms) {// verilen süre kadar tek beep sesi
    tone();
    delay(duration_ms);
    noTone();
}

void BuzzerDriver::beepPattern(uint8_t count, uint16_t beep_duration_ms, uint16_t pause_duration_ms) {//çoklu beep
    for (uint8_t i = 0; i < count; i++) {
        tone();
        delay(beep_duration_ms);
        noTone();

        if (i < count - 1) {
            delay(pause_duration_ms);
        }
    }
}

void BuzzerDriver::calibrationWarning() {//kalibrasyon bitene kadar uzun süre öter
    tone(1500);
}

void BuzzerDriver::systemReady() { // sistem hazır, yükselen 3 beep sesi

    tone(1800);
    delay(150);
    noTone();
    delay(100);

    tone(2000);
    delay(150);
    noTone();
    delay(100);

    tone(2200);
    delay(150);
    noTone();
}

void BuzzerDriver::errorAlert() { // hata var, aynı frekansta 3 beep sesi

    tone(2500);
    delay(200);
    noTone();
    delay(200);
    tone(2500);
    delay(200);
    noTone();
    delay(200);
    tone(2500);
    delay(200);
    noTone();
}

void BuzzerDriver::modeChange() { // mod değişikliği tek beep
    tone(2000);
    delay(100);
    noTone();
}
