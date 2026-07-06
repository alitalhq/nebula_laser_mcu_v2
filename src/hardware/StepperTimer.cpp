#include "StepperTimer.h"

StepperTimer* StepperTimer::_instances[2] = {nullptr, nullptr};// static değişkenler ISR'den örneğe erişim için gerekli

StepperTimer::StepperTimer()
    : _timer(nullptr)
    , _stepPin(0)
    , _dirPin(0)
    , _axis(PAN)
    , _stepCount(0)
    , _isrOverruns(0)
    , _currentDirection(true)  // Varsayılan: saat yönünde
{
}

bool StepperTimer::begin(Axis axis, uint8_t stepPin, uint8_t dirPin) {
    _axis = axis;
    _stepPin = stepPin;
    _dirPin = dirPin;

    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    digitalWrite(_stepPin, LOW);
    digitalWrite(_dirPin, LOW);

    // Donanım zamanlayıcısını oluştur
    // PAN için Timer 0, TILT için Timer 1
    // 80 = prescaler (80 MHz / 80 = 1 MHz = 1 μs çözünürlük)
    // true = sayma yukarı
    uint8_t timerNum = (_axis == PAN) ? 0 : 1;
    _timer = timerBegin(timerNum, 80, true);

    if (_timer == nullptr) {
        Serial.printf("Zamanlayici %d olusturulamadi\n", timerNum);
        return false;
    }

    _instances[_axis] = this;

    if (_axis == PAN) {
        timerAttachInterrupt(_timer, &StepperTimer::timerISR_Pan, true);
    } else {
        timerAttachInterrupt(_timer, &StepperTimer::timerISR_Tilt, true);
    }

    timerAlarmWrite(_timer, 1000000, true);

    timerAlarmDisable(_timer);

    Serial.printf("StepperTimer: Eksen %d baslatildi\n", _axis);
    return true;
}

void StepperTimer::setFrequency(float stepsPerSecond) {
    if (stepsPerSecond <= 0.0f) {//0dan küçükse motoru durdur
        disable();
        return;
    }

    // Makul aralıkla sınırla
    if (stepsPerSecond > 20000.0f) {//20kHz üzerini dikkate alma ve sınırla
        stepsPerSecond = 20000.0f;
    }

    float interruptFreq = stepsPerSecond * 2.0f;

    // Zamanlayıcı 1 MHz'de sayar (1 μs tik)
    // Periyot = 1.000.000 / kesme_frekansı
    uint64_t period = (uint64_t)(1000000.0f / interruptFreq);

    if (period < 10) {
        period = 10;  // Minimum 10 μs (güvenlik marjı)
    }

    // Zamanlayıcı alarmını yapılandır
    // true = otomatik yeniden yükle (periyodik)
    timerAlarmWrite(_timer, period, true);

    // Henüz etkin değilse etkinleştir
    if (!timerAlarmEnabled(_timer)) {
        enable();
    }
}

void StepperTimer::setDirection(bool clockwise) {//yön kontrolü sağlanıyor
    _currentDirection = clockwise;
    digitalWrite(_dirPin, clockwise ? HIGH : LOW);

    delayMicroseconds(1);
}

void StepperTimer::enable() {//etkinleştirme ve devre dışı bırakma
    if (_timer) {
        timerAlarmEnable(_timer);
    }
}

void StepperTimer::disable() {
    if (_timer) {
        timerAlarmDisable(_timer);
        digitalWrite(_stepPin, LOW);
    }
}

int32_t StepperTimer::getStepCount() const {//step sayma metodları
    return _stepCount;
}

void StepperTimer::resetStepCount() {
    _stepCount = 0;
}

uint32_t StepperTimer::getISROverruns() const {
    return _isrOverruns;
}

void IRAM_ATTR StepperTimer::timerISR_Pan() {//IRAM_ATTR sayesinde fonksiyon ramde saklanır, hızlı erişim için
    StepperTimer *instance = _instances[0];//interrupt ile değişiklilkte çağrılır
    if (!instance) return;

    // Pin durumunu toggle et
    static bool stepState = false;
    stepState = !stepState;
    digitalWrite(instance->_stepPin, stepState);

    // Yükselen kenarda (LOW→HIGH) step sayacını güncelle
    if (stepState) {
        if (instance->_currentDirection) {
            instance->_stepCount++;
        } else {
            instance->_stepCount--;
        }
    }
}

void IRAM_ATTR StepperTimer::timerISR_Tilt() {
    StepperTimer *instance = _instances[1];
    if (!instance) return;

    static bool stepState = false;
    stepState = !stepState;
    digitalWrite(instance->_stepPin, stepState);

    if (stepState) {
        if (instance->_currentDirection) {
            instance->_stepCount++;
        } else {
            instance->_stepCount--;
        }
    }
}
