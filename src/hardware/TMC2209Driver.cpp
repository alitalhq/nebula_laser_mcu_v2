#include "TMC2209Driver.h"

// ─── TMC2209 Register Adresleri ──────────────────────────────────
#define REG_GCONF       0x00
#define REG_GSTAT       0x01
#define REG_IHOLD_IRUN  0x10
#define REG_TPWMTHRS    0x13
#define REG_TCOOLTHRS   0x14
#define REG_CHOPCONF    0x6C
#define REG_DRV_STATUS  0x6F
#define REG_IOIN        0x06
#define REG_SGTHRS      0x40

// ─── CRC ─────────────────────────────────────────────────────────
uint8_t TMC2209Driver::_calcCRC(uint8_t *data, int len) {
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        uint8_t b = data[i];
        for (int j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (b & 0x01)) crc = (crc << 1) ^ 0x07;
            else                           crc <<= 1;
            b >>= 1;
        }
    }
    return crc;
}

// ─── UART OKUMA ──────────────────────────────────────────────────
bool TMC2209Driver::_read(uint8_t reg, uint32_t &value) {
    uint8_t tx[4] = { 0x05, TMC_DRIVER_ADDRESS, reg, 0 };
    tx[3] = _calcCRC(tx, 3);

    _serial->flush();
    while (_serial->available()) _serial->read();

    _serial->write(tx, 4);
    _serial->flush();

    uint32_t t = millis();
    while (_serial->available() < 12 && millis() - t < 50);

    // Echo'yu at (4 byte)
    for (int i = 0; i < 4 && _serial->available(); i++) _serial->read();

    uint8_t rx[8];
    int got = _serial->readBytes(rx, 8);
    if (got < 8) return false;

    uint8_t crc = _calcCRC(rx, 7);
    if (crc != rx[7]) return false;

    value = ((uint32_t)rx[3] << 24) | ((uint32_t)rx[4] << 16) |
            ((uint32_t)rx[5] << 8)  |  (uint32_t)rx[6];
    return true;
}

// ─── UART YAZMA ──────────────────────────────────────────────────
void TMC2209Driver::_write(uint8_t reg, uint32_t value) {
    uint8_t tx[8] = {
        0x05, TMC_DRIVER_ADDRESS, (uint8_t)(reg | 0x80),
        (uint8_t)(value >> 24), (uint8_t)(value >> 16),
        (uint8_t)(value >> 8),  (uint8_t)(value),
        0
    };
    tx[7] = _calcCRC(tx, 7);
    _serial->write(tx, 8);
    _serial->flush();
    delay(2);
}

// ─── CONSTRUCTOR ─────────────────────────────────────────────────
TMC2209Driver::TMC2209Driver()
    : _serial(nullptr)
    , _status(STATUS_NOT_INITIALIZED)
    , _uartOk(false)
    , _name("UNKNOWN")
{}

// ─── BEGIN ───────────────────────────────────────────────────────
bool TMC2209Driver::begin(HardwareSerial &serial, uint8_t rxPin, uint8_t txPin, const char* name) {
    _name   = name;
    _serial = &serial;

    Serial.printf("TMC2209[%s]: Baslatiliyor (TX=%d, RX=%d)...\n", _name, txPin, rxPin);

    serial.begin(TMC_UART_BAUD, SERIAL_8N1, rxPin, txPin);
    serial.setTimeout(20);
    delay(10);

    // UART bağlantı testi (3 deneme)
    bool connected = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        if (testConnection()) { connected = true; break; }
        Serial.printf("TMC2209[%s]: Deneme %d basarisiz\n", _name, attempt + 1);
        delay(50);
    }

    _uartOk = connected;
    if (!connected) {
        Serial.printf("TMC2209[%s]: UYARI - UART yok, standalone modda devam\n", _name);
    } else {
        Serial.printf("TMC2209[%s]: UART baglantisi basarili\n", _name);
    }

    if (!configure()) {
        Serial.printf("TMC2209[%s]: HATA - Yapilandirma basarisiz!\n", _name);
        _status = STATUS_CONFIG_ERROR;
        return false;
    }

    verifyConfiguration();

    _status = STATUS_OK;
    Serial.printf("TMC2209[%s]: Hazir — %dmA RMS, %d mikro adim, StealthChop\n",
                  _name, TMC_RMS_CURRENT, TMC_MICROSTEPS);
    return true;
}

// ─── BAĞLANTI TESTİ ──────────────────────────────────────────────
bool TMC2209Driver::testConnection() {
    uint32_t ioin = 0;
    if (!_read(REG_IOIN, ioin)) {
        Serial.printf("TMC2209[%s]: IOIN okunamadi\n", _name);
        return false;
    }
    uint8_t version = (ioin >> 24) & 0xFF;
    Serial.printf("TMC2209[%s]: IOIN=0x%08X, version=0x%02X\n", _name, ioin, version);
    if (version == 0x21) {
        Serial.printf("TMC2209[%s]: Dogrulandi (TMC2209)\n", _name);
        return true;
    }
    Serial.printf("TMC2209[%s]: Beklenmeyen version=0x%02X\n", _name, version);
    return false;
}

// ─── YAPILANDIRMA ────────────────────────────────────────────────
bool TMC2209Driver::configure() {
    // GCONF: pdn_disable=1 (bit6), mstep_reg_select=1 (bit7), StealthChop, I_scale_analog=0
    _write(REG_GCONF, 0x000000C0);
    delay(10);

    // IHOLD_IRUN: akım ayarı
    int irun = (int)(TMC_RMS_CURRENT / 1000.0f * 32.0f * 1.41421f * TMC_RSENSE / 0.325f) - 1;
    if (irun > 31) irun = 31;
    if (irun < 0)  irun = 0;
    uint32_t ihold_irun = ((uint32_t)TMC_IHOLDDELAY << 16) |
                          ((uint32_t)irun            <<  8) |
                           (uint32_t)TMC_IHOLD;
    _write(REG_IHOLD_IRUN, ihold_irun);
    delay(10);

    // CHOPCONF: toff=3, hstrt=4, hend=1, tbl=2, intpol=1, mres
    uint8_t mres;
    switch (TMC_MICROSTEPS) {
        case 256: mres = 0; break; case 128: mres = 1; break;
        case  64: mres = 2; break; case  32: mres = 3; break;
        case  16: mres = 4; break; case   8: mres = 5; break;
        case   4: mres = 6; break; case   2: mres = 7; break;
        default:  mres = 3; break;
    }
    uint32_t chopconf = 0x10000000              // intpol=1
                      | ((uint32_t)mres << 24)  // MRES
                      | (2UL << 16)             // tbl=2
                      | (1UL <<  7)             // hend=1
                      | (4UL <<  4)             // hstrt=4
                      | 3UL;                    // toff=3
    _write(REG_CHOPCONF, chopconf);
    delay(10);

    // StealthChop eşiği (0 = her zaman StealthChop)
    _write(REG_TPWMTHRS, TMC_TPWMTHRS);
    delay(5);

    // StallGuard / CoolStep devre dışı
    _write(REG_SGTHRS,    0);
    delay(5);
    _write(REG_TCOOLTHRS, 0);
    delay(5);

    // Hata bitlerini temizle
    _write(REG_GSTAT, 0x07);
    delay(5);

    return true;
}

// ─── DOĞRULAMA ───────────────────────────────────────────────────
bool TMC2209Driver::verifyConfiguration() {
    delay(20);
    uint32_t gconf = 0, chopconf = 0, drv = 0;
    bool gconf_ok    = _read(REG_GCONF,      gconf);
    bool chop_ok     = _read(REG_CHOPCONF,   chopconf);
    bool drv_ok      = _read(REG_DRV_STATUS, drv);

    Serial.printf("TMC2209[%s]: GCONF=0x%08X, CHOPCONF=0x%08X, DRV_STATUS=0x%08X\n",
                  _name, gconf, chopconf, drv);

    if (!gconf_ok && !chop_ok && !drv_ok) {
        Serial.printf("TMC2209[%s]: KRITIK - Tum registerlar okunamadi!\n", _name);
        return false;
    }

    // pdn_disable kontrolü
    if (!(gconf & (1 << 6)))
        Serial.printf("TMC2209[%s]: UYARI - pdn_disable=0\n", _name);

    // toff kontrolü
    if ((chopconf & 0x0F) == 0)
        Serial.printf("TMC2209[%s]: UYARI - toff=0, surucu KAPALI!\n", _name);

    // StealthChop kontrolü
    if (gconf & (1 << 2))
        Serial.printf("TMC2209[%s]: UYARI - SpreadCycle aktif (StealthChop degil)\n", _name);

    uint8_t irun_read = (chopconf >> 8) & 0x1F;
    Serial.printf("TMC2209[%s]: MRES=%d, toff=%d\n",
                  _name, (chopconf >> 24) & 0x0F, chopconf & 0x0F);

    return true;
}

// ─── DURUM YAZDIR ────────────────────────────────────────────────
void TMC2209Driver::printStatus() {
    uint32_t ioin = 0, gconf = 0, chopconf = 0, drv = 0;
    _read(REG_IOIN,       ioin);
    _read(REG_GCONF,      gconf);
    _read(REG_CHOPCONF,   chopconf);
    _read(REG_DRV_STATUS, drv);

    Serial.printf("\n--- TMC2209[%s] Durum ---\n", _name);
    Serial.printf("  VERSION:   0x%02X %s\n", (ioin >> 24) & 0xFF,
                  ((ioin >> 24) & 0xFF) == 0x21 ? "[OK]" : "[!!]");
    Serial.printf("  GCONF:     0x%08X\n", gconf);
    Serial.printf("  CHOPCONF:  0x%08X (MRES=%d, toff=%d)\n",
                  chopconf, (chopconf >> 24) & 0x0F, chopconf & 0x0F);
    Serial.printf("  DRV_STATUS:0x%08X\n", drv);
    Serial.printf("    otpw=%d ot=%d s2ga=%d s2gb=%d ola=%d olb=%d stst=%d\n",
                  (drv>>0)&1, (drv>>1)&1, (drv>>2)&1, (drv>>3)&1,
                  (drv>>4)&1, (drv>>5)&1, (drv>>31)&1);
    Serial.printf("--- TMC2209[%s] Bitti ---\n\n", _name);
}

// ─── AKIM / SICAKLIK TANI ────────────────────────────────────────
// CS_ACTUAL: surucunun o an uyguladigi gercek akim skalasi (0-31)
// Durunca IHOLD'u, hareket edince IRUN'u yansitir.
// Sicaklik bantlari (t120/t143/t150/t157) sensorsuz kaba sicaklik verir.
void TMC2209Driver::printCurrentDiag() {
    uint32_t drv = 0;
    if (!_read(REG_DRV_STATUS, drv)) {
        Serial.printf("  TMC[%s]: DRV_STATUS okunamadi\n", _name);
        return;
    }

    uint8_t cs_actual = (drv >> 16) & 0x1F;
    // I_RMS = (CS+1)/32 * Vfs / (Rsense * sqrt2),  Vfs=0.325 (vsense=0)
    float i_rms = (cs_actual + 1) / 32.0f * 0.325f / (TMC_RSENSE * 1.41421f);

    bool otpw = (drv >> 0) & 1;   // >120C on-uyari
    bool ot   = (drv >> 1) & 1;   // >143C kapanma
    bool stst = (drv >> 31) & 1;  // standstill

    const char* temp;
    if      ((drv >> 11) & 1) temp = ">157C";
    else if ((drv >> 10) & 1) temp = ">150C";
    else if ((drv >>  9) & 1) temp = ">143C";
    else if ((drv >>  8) & 1) temp = ">120C";
    else                      temp = "<120C";

    Serial.printf("  TMC[%s] %-7s | CS_ACTUAL=%2d (~%.2fA RMS) | sicaklik=%s | otpw=%d ot=%d\n",
                  _name, stst ? "DURUYOR" : "HAREKET",
                  cs_actual, i_rms, temp, otpw, ot);
}

// ─── AŞIRI ISI ───────────────────────────────────────────────────
bool TMC2209Driver::isOverTemperature() {
    uint32_t drv = 0;
    _read(REG_DRV_STATUS, drv);
    return (drv & 0x03) != 0;
}

// ─── AÇIK YÜKÜ ───────────────────────────────────────────────────
bool TMC2209Driver::isOpenLoad() {
    uint32_t drv = 0;
    _read(REG_DRV_STATUS, drv);
    return (drv & 0x30) != 0;
}
