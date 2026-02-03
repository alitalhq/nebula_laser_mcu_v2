#ifndef TARGET_MANAGER_H
#define TARGET_MANAGER_H

#include <Arduino.h>
#include "SerialProtocol.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "../config/HardwareConfig.h"

class TargetManager {
public:

    enum Mode {
        MODE_GROUND_LOCK = 0,   // Varsayilan: Yere bak (guvenli mod)
        MODE_TRACKING = 1       // ROS2'den gelen hedefi takip et
    };

    enum CommStatus {
        COMM_OK = 0,            // Haberlesme aktif
        COMM_TIMEOUT = 1,       // Komut alinmadi (yumusak zaman asimi)
        COMM_LOST = 2           // Haberlesme kaybedildi (sert zaman asimi)
    };

    TargetManager();

    bool begin(float initialPan, float initialTilt);

    void handleCommand(const SerialProtocol::GimbalCommand &cmd);

    void setGroundLockTarget(float pan, float tilt);

    void getTargets(float &pan, float &tilt, float &ffPan, float &ffTilt);

    void setTarget(float pan, float tilt);

    void forceGroundLock();

    Mode getMode() const { return _mode; }

    CommStatus getCommStatus() const { return _commStatus; }

    bool isCommHealthy() const { return _commStatus == COMM_OK; }

    uint32_t getTimeSinceLastCommand() const;

    void setCommandTimeout(uint32_t timeout_ms) { _commandTimeoutMs = timeout_ms; }

    uint32_t getCommandCount() const { return _commandCount; }
    uint32_t getTimeoutCount() const { return _timeoutCount; }
    void resetStats();

private:
    Mode _mode;
    CommStatus _commStatus;

    float _targetPan, _targetTilt;
    float _feedforwardPan, _feedforwardTilt;
    float _groundLockPan, _groundLockTilt;

    uint32_t _lastCommandTime;
    uint32_t _lastModeChangeTime;
    uint32_t _commandTimeoutMs;

    uint32_t _commandCount;
    uint32_t _timeoutCount;

    SemaphoreHandle_t _mutex;

    struct PendingNotification {
        bool pending;
        Mode oldMode;
        Mode newMode;
        const char* reason;
    };
    PendingNotification _pendingNotif;

    void checkTimeout();
    void switchMode(Mode newMode, const char* reason);
    void processPendingNotification();
};

#endif
