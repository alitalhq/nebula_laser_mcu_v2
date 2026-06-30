#ifndef TMC2209_DRIVER_H
#define TMC2209_DRIVER_H

#include <Arduino.h>
#include "../config/HardwareConfig.h"

class TMC2209Driver {
public:
    enum DriverStatus {
        STATUS_OK = 0,
        STATUS_NOT_INITIALIZED,
        STATUS_COMM_ERROR,
        STATUS_CONFIG_ERROR
    };

    TMC2209Driver();

    bool begin(HardwareSerial &serial, uint8_t rxPin, uint8_t txPin, const char* name);
    bool testConnection();

    DriverStatus getStatus() const { return _status; }
    bool isUartOk() const { return _uartOk; }
    void printStatus();
    void printCurrentDiag();   // CS_ACTUAL (gercek akim) + sicaklik bandi + otpw/ot
    bool isOverTemperature();
    bool isOpenLoad();

private:
    HardwareSerial* _serial;
    DriverStatus     _status;
    bool             _uartOk;
    const char*      _name;

    uint8_t  _calcCRC(uint8_t *data, int len);
    bool     _read(uint8_t reg, uint32_t &value);
    void     _write(uint8_t reg, uint32_t value);

    bool configure();
    bool verifyConfiguration();
};

#endif
