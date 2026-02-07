#include "TMC2209Driver.h"

TMC2209Driver::TMC2209Driver()
    : _driver(nullptr)
    , _status(STATUS_NOT_INITIALIZED)
    , _name("UNKNOWN")
{
}

bool TMC2209Driver::begin(HardwareSerial &serial, uint8_t rxPin, uint8_t txPin, const char* name) {
    _name = name;

    Serial.printf("TMC2209[%s]: Baslatiliyor (TX=%d, RX=%d)...\n", _name, txPin, rxPin);

    // HardwareSerial'i belirtilen pinlerle baslat
    serial.begin(TMC_UART_BAUD, SERIAL_8N1, rxPin, txPin);
    delay(10);

    // TMC2209Stepper nesnesini olustur
    _driver = new TMC2209Stepper(&serial, TMC_RSENSE, TMC_DRIVER_ADDRESS);
    _driver->begin();

    // UART iletisimini test et (3 deneme)
    bool connected = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        if (testConnection()) {
            connected = true;
            break;
        }
        Serial.printf("TMC2209[%s]: Deneme %d basarisiz, tekrar deneniyor...\n", _name, attempt + 1);
        delay(50);
    }

    if (!connected) {
        Serial.printf("TMC2209[%s]: HATA - Iletisim kurulamadi!\n", _name);
        _status = STATUS_COMM_ERROR;
        return false;
    }

    Serial.printf("TMC2209[%s]: Iletisim basarili\n", _name);

    // Surucuyu yapilandir
    if (!configure()) {
        Serial.printf("TMC2209[%s]: HATA - Yapilandirma basarisiz!\n", _name);
        _status = STATUS_CONFIG_ERROR;
        return false;
    }

    // Yapilandirmayi dogrula
    if (!verifyConfiguration()) {
        Serial.printf("TMC2209[%s]: UYARI - Dogrulama basarisiz\n", _name);
    }

    _status = STATUS_OK;
    Serial.printf("TMC2209[%s]: Basarili - %dmA RMS, %d mikro adim, StealthChop etkin\n",
                  _name, TMC_RMS_CURRENT, TMC_MICROSTEPS);
    return true;
}

bool TMC2209Driver::testConnection() {
    // IOIN register'indan version oku - TMC2209 = 0x21
    uint32_t ioin = _driver->IOIN();
    uint8_t version = (ioin >> 24) & 0xFF;

    if (version == 0x21) {
        Serial.printf("TMC2209[%s]: Surucu dogrulandi (version=0x%02X)\n", _name, version);
        return true;
    }

    Serial.printf("TMC2209[%s]: Beklenmeyen version=0x%02X (beklenen 0x21)\n", _name, version);
    return false;
}

bool TMC2209Driver::configure() {
    // 1. UART modunu etkinlestir
    _driver->pdn_disable(true);         // PDN_UART pinini UART icin kullan
    _driver->I_scale_analog(false);     // Dahili VREF kullan (VREF pini degil)
    _driver->mstep_reg_select(true);    // Mikro adim UART ile ayarlanacak (MS1/MS2 degil)

    // 2. Akim ayari
    _driver->rms_current(TMC_RMS_CURRENT);
    _driver->ihold(TMC_IHOLD);          // Bekleme akimi (0-31)
    _driver->iholddelay(TMC_IHOLDDELAY);

    // 3. Mikro adim
    _driver->microsteps(TMC_MICROSTEPS);

    // 4. StealthChop yapilandirmasi
    _driver->en_spreadCycle(false);     // StealthChop etkin
    _driver->TPWMTHRS(TMC_TPWMTHRS);   // Gecis esigi

    // 5. CHOPCONF ince ayarlari
    _driver->toff(3);                   // Chopper kapali suresi
    _driver->hstrt(4);                  // Hysteresis baslangici
    _driver->hend(1);                   // Hysteresis sonu
    _driver->tbl(2);                    // Blanking time

    // 6. StallGuard / CoolStep devre disi
    _driver->SGTHRS(0);
    _driver->TCOOLTHRS(0);

    // 7. Hata bitlerini temizle
    _driver->GSTAT(0x07);

    return true;
}

bool TMC2209Driver::verifyConfiguration() {
    bool allOk = true;

    // Mikro adim dogrulama
    uint16_t readMicrosteps = _driver->microsteps();
    if (readMicrosteps != TMC_MICROSTEPS) {
        Serial.printf("TMC2209[%s]: UYARI - Mikro adim: beklenen=%d, okunan=%d\n",
                      _name, TMC_MICROSTEPS, readMicrosteps);
        allOk = false;
    } else {
        Serial.printf("TMC2209[%s]: Mikro adim: %d OK\n", _name, readMicrosteps);
    }

    // StealthChop dogrulama
    if (_driver->en_spreadCycle()) {
        Serial.printf("TMC2209[%s]: UYARI - StealthChop etkin degil\n", _name);
        allOk = false;
    } else {
        Serial.printf("TMC2209[%s]: StealthChop etkin OK\n", _name);
    }

    // pdn_disable dogrulama
    if (!_driver->pdn_disable()) {
        Serial.printf("TMC2209[%s]: UYARI - pdn_disable ayarlanmamis\n", _name);
        allOk = false;
    }

    // mstep_reg_select dogrulama
    if (!_driver->mstep_reg_select()) {
        Serial.printf("TMC2209[%s]: UYARI - mstep_reg_select ayarlanmamis\n", _name);
        allOk = false;
    }

    if (allOk) {
        Serial.printf("TMC2209[%s]: Tum dogrulamalar basarili\n", _name);
    }

    return allOk;
}

void TMC2209Driver::printStatus() {
    if (_driver == nullptr) {
        Serial.printf("TMC2209[%s]: Surucu baslatilmamis\n", _name);
        return;
    }

    Serial.printf("\n--- TMC2209[%s] Durum ---\n", _name);
    Serial.printf("  GCONF: en_spreadCycle=%d, pdn_disable=%d, mstep_reg_select=%d\n",
                  _driver->en_spreadCycle(), _driver->pdn_disable(), _driver->mstep_reg_select());
    Serial.printf("  IRUN=%d, IHOLD=%d, IHOLDDELAY=%d\n",
                  _driver->irun(), _driver->ihold(), _driver->iholddelay());
    Serial.printf("  Mikro adim: %d\n", _driver->microsteps());
    Serial.printf("  CHOPCONF: toff=%d, hstrt=%d, hend=%d, tbl=%d\n",
                  _driver->toff(), _driver->hstrt(), _driver->hend(), _driver->tbl());
    Serial.printf("  TPWMTHRS: %u\n", _driver->TPWMTHRS());

    uint32_t drvStatus = _driver->DRV_STATUS();
    Serial.printf("  DRV_STATUS: 0x%08X\n", drvStatus);
    Serial.printf("    otpw=%d, ot=%d, s2ga=%d, s2gb=%d, ola=%d, olb=%d, stst=%d\n",
                  (drvStatus >> 0) & 1, (drvStatus >> 1) & 1,
                  (drvStatus >> 2) & 1, (drvStatus >> 3) & 1,
                  (drvStatus >> 4) & 1, (drvStatus >> 5) & 1,
                  (drvStatus >> 31) & 1);
    Serial.printf("--- TMC2209[%s] Bitti ---\n\n", _name);
}

bool TMC2209Driver::isOverTemperature() {
    if (_driver == nullptr) return false;
    uint32_t drvStatus = _driver->DRV_STATUS();
    return (drvStatus & 0x03) != 0; // bit0=otpw, bit1=ot
}

bool TMC2209Driver::isOpenLoad() {
    if (_driver == nullptr) return false;
    uint32_t drvStatus = _driver->DRV_STATUS();
    return (drvStatus & 0x30) != 0; // bit4=ola, bit5=olb
}
