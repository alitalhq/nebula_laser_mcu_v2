#include "TMC2209Driver.h"

TMC2209Driver::TMC2209Driver()
    : _driver(nullptr)
    , _status(STATUS_NOT_INITIALIZED)
    , _uartOk(false)
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

    _uartOk = connected;
    if (!connected) {
        Serial.printf("TMC2209[%s]: UYARI - UART baglantisi yok, standalone modda devam ediliyor\n", _name);
    } else {
        Serial.printf("TMC2209[%s]: UART iletisim basarili (version=0x21)\n", _name);
    }

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

    Serial.printf("TMC2209[%s]: IOIN=0x%08X, version=0x%02X\n", _name, ioin, version);

    if (version == 0x21) {
        Serial.printf("TMC2209[%s]: Surucu dogrulandi (TMC2209)\n", _name);
        return true;
    }

    Serial.printf("TMC2209[%s]: HATA - Beklenmeyen version=0x%02X (beklenen 0x21)\n", _name, version);
    if (version == 0x00) {
        Serial.printf("TMC2209[%s]: IPUCU - UART hattinda 1K direnç var mi? TX-RX arasi 1K ohm gerekli.\n", _name);
    }
    return false;
}

bool TMC2209Driver::configure() {
    // Her register yazimi arasinda gecikme ekle (TMC2209 UART zamanlama gereksinimleri)
    const uint8_t REG_DELAY_MS = 10;

    // 1. Oncelikle GCONF register'ini oku, sonra degistir
    //    (read-modify-write - mevcut bitleri korumak icin)
    _driver->pdn_disable(true);         // PDN_UART pinini UART icin kullan
    delay(REG_DELAY_MS);
    _driver->I_scale_analog(false);     // Dahili VREF kullan (VREF pini degil)
    delay(REG_DELAY_MS);
    _driver->mstep_reg_select(true);    // Mikro adim UART ile ayarlanacak (MS1/MS2 degil)
    delay(REG_DELAY_MS);

    // GCONF dogrulamasi - kritik register
    bool gconf_ok = _driver->pdn_disable() && _driver->mstep_reg_select();
    if (!gconf_ok) {
        Serial.printf("TMC2209[%s]: GCONF yazimi basarisiz, tekrar deneniyor...\n", _name);
        delay(50);
        // Tum GCONF'u tek seferde yaz
        // pdn_disable=1(bit6), mstep_reg_select=1(bit7), en_spreadcycle=0(bit2), i_scale_analog=0(bit0)
        _driver->GCONF(0xC0); // bit6 + bit7
        delay(REG_DELAY_MS);

        gconf_ok = _driver->pdn_disable() && _driver->mstep_reg_select();
        if (!gconf_ok) {
            Serial.printf("TMC2209[%s]: KRITIK - GCONF hala yazilamiyor! UART baglantisini kontrol edin.\n", _name);
            Serial.printf("TMC2209[%s]:   - TX ve RX arasi 1K ohm direnç var mi?\n", _name);
            Serial.printf("TMC2209[%s]:   - TMC2209 besleme gerilimi (VCC_IO) 3.3V mi?\n", _name);
            Serial.printf("TMC2209[%s]:   - Pin atamalari dogru mu? (TX=%d, RX=%d)\n", _name,
                          TMC_PAN_TX_PIN, TMC_PAN_RX_PIN);
        }
    }

    // 2. Akim ayari
    _driver->rms_current(TMC_RMS_CURRENT);
    delay(REG_DELAY_MS);
    _driver->ihold(TMC_IHOLD);          // Bekleme akimi (0-31)
    delay(REG_DELAY_MS);
    _driver->iholddelay(TMC_IHOLDDELAY);
    delay(REG_DELAY_MS);

    // 3. CHOPCONF ayarlari - once toff ayarla (driver'i etkinlestirir)
    _driver->toff(3);                   // Chopper kapali suresi (0=driver kapali!)
    delay(REG_DELAY_MS);
    _driver->hstrt(4);                  // Hysteresis baslangici
    delay(REG_DELAY_MS);
    _driver->hend(1);                   // Hysteresis sonu
    delay(REG_DELAY_MS);
    _driver->tbl(2);                    // Blanking time
    delay(REG_DELAY_MS);

    // 4. Mikro adim (CHOPCONF'a yazar, toff korunmali)
    _driver->microsteps(TMC_MICROSTEPS);
    delay(REG_DELAY_MS);

    // CHOPCONF dogrulamasi
    uint8_t toff_read = _driver->toff();
    if (toff_read == 0) {
        Serial.printf("TMC2209[%s]: UYARI - CHOPCONF.toff=0, driver KAPALI! Tekrar deneniyor...\n", _name);
        delay(50);
        _driver->toff(3);
        delay(REG_DELAY_MS);
        _driver->hstrt(4);
        delay(REG_DELAY_MS);
        _driver->hend(1);
        delay(REG_DELAY_MS);
        _driver->tbl(2);
        delay(REG_DELAY_MS);
        _driver->microsteps(TMC_MICROSTEPS);
        delay(REG_DELAY_MS);
    }

    // 5. StealthChop yapilandirmasi
    _driver->en_spreadCycle(false);     // StealthChop etkin
    delay(REG_DELAY_MS);
    _driver->TPWMTHRS(TMC_TPWMTHRS);   // Gecis esigi
    delay(REG_DELAY_MS);

    // 6. StallGuard / CoolStep devre disi
    _driver->SGTHRS(0);
    delay(REG_DELAY_MS);
    _driver->TCOOLTHRS(0);
    delay(REG_DELAY_MS);

    // 7. Hata bitlerini temizle
    _driver->GSTAT(0x07);
    delay(REG_DELAY_MS);

    return true;
}

bool TMC2209Driver::verifyConfiguration() {
    bool allOk = true;
    delay(20); // Dogrulama oncesi bekle

    // Raw register degerlerini oku
    uint32_t gconf_raw = _driver->GCONF();
    delay(5);
    uint32_t chopconf_raw = _driver->CHOPCONF();
    delay(5);
    uint32_t drv_status_raw = _driver->DRV_STATUS();
    delay(5);

    Serial.printf("TMC2209[%s]: Raw GCONF=0x%08X, CHOPCONF=0x%08X, DRV_STATUS=0x%08X\n",
                  _name, gconf_raw, chopconf_raw, drv_status_raw);

    // Tum registerlar 0 ise UART iletisimi calismiyor
    if (gconf_raw == 0 && chopconf_raw == 0 && drv_status_raw == 0) {
        Serial.printf("TMC2209[%s]: KRITIK - Tum registerlar 0! UART iletisimi calismiyor!\n", _name);
        Serial.printf("TMC2209[%s]: Kontrol listesi:\n", _name);
        Serial.printf("  1. TX ve RX arasi 1K ohm direnç bagli mi?\n");
        Serial.printf("  2. TMC2209 VCC_IO = 3.3V mi?\n");
        Serial.printf("  3. TMC2209 VM (motor gerilimi) bagli mi?\n");
        Serial.printf("  4. ESP32 TX -> TMC2209 PDN_UART pinine mi gidiyor?\n");
        Serial.printf("  5. ENABLE pini LOW mu? (Pin %d)\n", MOTOR_ENABLE_PIN);
        return false;
    }

    // pdn_disable dogrulama
    if (!_driver->pdn_disable()) {
        Serial.printf("TMC2209[%s]: UYARI - pdn_disable=0 (olmali=1)\n", _name);
        allOk = false;
    } else {
        Serial.printf("TMC2209[%s]: pdn_disable=1 OK\n", _name);
    }

    // mstep_reg_select dogrulama
    if (!_driver->mstep_reg_select()) {
        Serial.printf("TMC2209[%s]: UYARI - mstep_reg_select=0 (olmali=1)\n", _name);
        allOk = false;
    } else {
        Serial.printf("TMC2209[%s]: mstep_reg_select=1 OK\n", _name);
    }

    // Mikro adim dogrulama
    uint16_t readMicrosteps = _driver->microsteps();
    if (readMicrosteps != TMC_MICROSTEPS) {
        Serial.printf("TMC2209[%s]: UYARI - Mikro adim: beklenen=%d, okunan=%d\n",
                      _name, TMC_MICROSTEPS, readMicrosteps);
        allOk = false;
    } else {
        Serial.printf("TMC2209[%s]: Mikro adim: %d OK\n", _name, readMicrosteps);
    }

    // CHOPCONF.toff dogrulama (0 = driver kapali)
    uint8_t toff_val = _driver->toff();
    if (toff_val == 0) {
        Serial.printf("TMC2209[%s]: UYARI - toff=0, motor surucusu KAPALI!\n", _name);
        allOk = false;
    } else {
        Serial.printf("TMC2209[%s]: toff=%d OK\n", _name, toff_val);
    }

    // StealthChop dogrulama
    if (_driver->en_spreadCycle()) {
        Serial.printf("TMC2209[%s]: UYARI - StealthChop etkin degil\n", _name);
        allOk = false;
    } else {
        Serial.printf("TMC2209[%s]: StealthChop etkin OK\n", _name);
    }

    // Akim dogrulama
    Serial.printf("TMC2209[%s]: IRUN=%d, IHOLD=%d, IHOLDDELAY=%d\n",
                  _name, _driver->irun(), _driver->ihold(), _driver->iholddelay());

    if (allOk) {
        Serial.printf("TMC2209[%s]: Tum dogrulamalar basarili!\n", _name);
    } else {
        Serial.printf("TMC2209[%s]: Bazi dogrulamalar basarisiz - yukaridaki uyarilara bakin\n", _name);
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

bool TMC2209Driver::uartLoopbackTest(HardwareSerial &serial, uint8_t rxPin, uint8_t txPin, const char* name) {
    Serial.printf("\n=== UART LOOPBACK TESTI [%s] ===\n", name);
    Serial.printf("TX=%d, RX=%d, Baud=%d\n", txPin, rxPin, TMC_UART_BAUD);
    Serial.printf("DIKKAT: TMC2209'u sokun ve TX pinini RX pinine dogrudan baglayin!\n");
    Serial.printf("(GPIO %d ile GPIO %d arasina kisa kablo)\n\n", txPin, rxPin);

    serial.begin(TMC_UART_BAUD, SERIAL_8N1, rxPin, txPin);
    delay(50);

    // RX buffer'ini temizle
    while (serial.available()) serial.read();

    // Test verileri gonder
    const uint8_t testData[] = {0x05, 0x00, 0x06, 0xA1, 0x55};
    const int testLen = sizeof(testData);
    int successCount = 0;

    for (int round = 0; round < 3; round++) {
        // Gonder
        for (int i = 0; i < testLen; i++) {
            serial.write(testData[i]);
        }
        serial.flush(); // Gonderme tamamlanana kadar bekle
        delay(10);

        // Oku
        uint8_t received[16] = {0};
        int rxCount = 0;
        unsigned long start = millis();
        while (rxCount < testLen && (millis() - start) < 100) {
            if (serial.available()) {
                received[rxCount++] = serial.read();
            }
        }

        // Karsilastir
        bool match = (rxCount == testLen);
        if (match) {
            for (int i = 0; i < testLen; i++) {
                if (received[i] != testData[i]) {
                    match = false;
                    break;
                }
            }
        }

        Serial.printf("  Deneme %d: Gonderilen=%d bayt, Alinan=%d bayt -> %s\n",
                      round + 1, testLen, rxCount, match ? "BASARILI" : "BASARISIZ");

        if (!match && rxCount > 0) {
            Serial.printf("    Gonderilen: ");
            for (int i = 0; i < testLen; i++) Serial.printf("0x%02X ", testData[i]);
            Serial.printf("\n    Alinan:     ");
            for (int i = 0; i < rxCount; i++) Serial.printf("0x%02X ", received[i]);
            Serial.printf("\n");
        } else if (rxCount == 0) {
            Serial.printf("    Hic veri alinamadi! Pin atamasi yanlis olabilir.\n");
        }

        if (match) successCount++;

        // Buffer temizle
        while (serial.available()) serial.read();
        delay(20);
    }

    serial.end();

    Serial.printf("\nSONUC: %d/3 basarili\n", successCount);
    if (successCount == 3) {
        Serial.printf("  -> ESP32 UART bu pinlerde CALISIYOR. Sorun TMC2209 baglantisinda.\n");
    } else if (successCount == 0) {
        Serial.printf("  -> ESP32 UART bu pinlerde CALISMIYOR!\n");
        Serial.printf("  -> Pin atamasini kontrol edin veya farkli pinleri deneyin.\n");
    }
    Serial.printf("=== LOOPBACK TESTI BITTI ===\n\n");

    return successCount == 3;
}
