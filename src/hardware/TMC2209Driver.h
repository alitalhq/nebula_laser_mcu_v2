#ifndef TMC2209_DRIVER_H
#define TMC2209_DRIVER_H

#include <Arduino.h>
#include <TMCStepper.h>
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
    bool isOverTemperature();
    bool isOpenLoad();

    TMC2209Stepper* getDriver() { return _driver; }

private:
    TMC2209Stepper* _driver;
    DriverStatus _status;
    bool _uartOk;
    const char* _name;

    bool configure();
    bool verifyConfiguration();

public:
    // UART loopback testi: TX ve RX pinlerini birbirine bagla (TMC2209'u sok)
    static bool uartLoopbackTest(HardwareSerial &serial, uint8_t rxPin, uint8_t txPin, const char* name);
};

#endif
