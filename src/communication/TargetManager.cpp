#include "TargetManager.h"
#include "SerialProtocol.h"
#include "../utils/MathUtils.h"
#include "../hardware/BuzzerDriver.h"

extern BuzzerDriver g_buzzer;

TargetManager::TargetManager()
    : _mode(MODE_GROUND_LOCK)
    , _commStatus(COMM_OK)
    , _targetPan(0.0f), _targetTilt(0.0f)
    , _feedforwardPan(0.0f), _feedforwardTilt(0.0f)
    , _groundLockPan(0.0f), _groundLockTilt(90.0f)  // Yere bak: 90 derece asagi
    , _lastCommandTime(0)
    , _lastModeChangeTime(0)
    , _commandTimeoutMs(ROS2_COMMAND_TIMEOUT_MS)
    , _commandCount(0)
    , _timeoutCount(0)
    , _mutex(nullptr)
{
    _pendingNotif.pending = false;
    _pendingNotif.oldMode = MODE_GROUND_LOCK;
    _pendingNotif.newMode = MODE_GROUND_LOCK;
    _pendingNotif.reason = nullptr;
}

bool TargetManager::begin(float initialPan, float initialTilt) {
    _targetPan = initialPan;
    _targetTilt = initialTilt;

    // ground-lock modunda basla (guvenli varsayilan)
    _mode = MODE_GROUND_LOCK;
    _commStatus = COMM_OK;
    _groundLockPan = 0.0f;
    _groundLockTilt = 90.0f;  // Tam asagi bak (yere dogru)

    _lastCommandTime = millis();
    _lastModeChangeTime = millis();
    _commandTimeoutMs = ROS2_COMMAND_TIMEOUT_MS;

    _feedforwardPan = 0;
    _feedforwardTilt = 0;

    _commandCount = 0;
    _timeoutCount = 0;

    _pendingNotif.pending = false;

    _mutex = xSemaphoreCreateMutex();
    if (!_mutex) {
        Serial.println("TargetManager: Mutex olusturulamadi");
        return false;
    }

    Serial.printf("TargetManager: Baslatildi (zaman asimi=%ums)\n", _commandTimeoutMs);
    return true;
}

void TargetManager::handleCommand(const SerialProtocol::GimbalCommand &cmd) {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _lastCommandTime = millis();
        _commandCount++;

        _commStatus = COMM_OK;

        if (_mode == MODE_GROUND_LOCK) {
            switchMode(MODE_TRACKING, "ROS2 komutu alindi");

            _targetPan = _groundLockPan;
            _targetTilt = _groundLockTilt;
        }

        _targetPan += cmd.pan_delta;
        _targetTilt += cmd.tilt_delta;

        _targetPan = MathUtils::wrapAngle360(_targetPan);
        _targetTilt = MathUtils::wrapAngle360(_targetTilt);

        _feedforwardPan = cmd.feedforward_vel_pan;
        _feedforwardTilt = cmd.feedforward_vel_tilt;

        xSemaphoreGive(_mutex);
        processPendingNotification();
    }
}

void TargetManager::setGroundLockTarget(float pan, float tilt) {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _groundLockPan = pan;
        _groundLockTilt = tilt;
        xSemaphoreGive(_mutex);
    }
}

void TargetManager::getTargets(float &pan, float &tilt, float &ffPan, float &ffTilt) {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        checkTimeout();

        if (_mode == MODE_GROUND_LOCK) {
            pan = _groundLockPan;
            tilt = _groundLockTilt;
            ffPan = 0.0f;
            ffTilt = 0.0f;
        } else {
            pan = _targetPan;
            tilt = _targetTilt;
            ffPan = _feedforwardPan;
            ffTilt = _feedforwardTilt;
        }

        xSemaphoreGive(_mutex);

        processPendingNotification();
    }
}

void TargetManager::setTarget(float pan, float tilt) {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _targetPan = pan;
        _targetTilt = tilt;
        xSemaphoreGive(_mutex);
    }
}

void TargetManager::forceGroundLock() {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (_mode != MODE_GROUND_LOCK) {
            switchMode(MODE_GROUND_LOCK, "ACIL: Zorunlu yer-kilit");
        }
        xSemaphoreGive(_mutex);

        processPendingNotification();
    }
}

uint32_t TargetManager::getTimeSinceLastCommand() const {
    return millis() - _lastCommandTime;
}

void TargetManager::resetStats() {
    _commandCount = 0;
    _timeoutCount = 0;
}

void TargetManager::checkTimeout() {
    if (_mode != MODE_TRACKING) {
        return;
    }

    uint32_t timeSinceCommand = millis() - _lastCommandTime;

    if (timeSinceCommand > _commandTimeoutMs) {
        _timeoutCount++;

        if (timeSinceCommand > SERIAL_HEARTBEAT_TIMEOUT_MS) {
            _commStatus = COMM_LOST;
        } else {
            _commStatus = COMM_TIMEOUT;
        }

        switchMode(MODE_GROUND_LOCK, "Haberlesme zaman asimi - guvenlik modu aktif");

        _feedforwardPan = 0.0f;
        _feedforwardTilt = 0.0f;
    }
}

void TargetManager::switchMode(Mode newMode, const char* reason) {
    if (_mode == newMode) {
        return;
    }

    Mode oldMode = _mode;
    _mode = newMode;

    uint32_t now = millis();
    bool shouldNotify = (now - _lastModeChangeTime) > MODE_CHANGE_DEBOUNCE_MS;

    if (shouldNotify) {
        _lastModeChangeTime = now;

        _pendingNotif.pending = true;
        _pendingNotif.oldMode = oldMode;
        _pendingNotif.newMode = newMode;
        _pendingNotif.reason = reason;
    }
}

void TargetManager::processPendingNotification() {

    if (!_pendingNotif.pending) {
        return;
    }

    _pendingNotif.pending = false;

    const char* oldModeStr = (_pendingNotif.oldMode == MODE_GROUND_LOCK) ? "YER_KILIT" : "TAKIP";
    const char* newModeStr = (_pendingNotif.newMode == MODE_GROUND_LOCK) ? "YER_KILIT" : "TAKIP";

    Serial.printf("TargetManager: Mod %s -> %s (%s)\n", oldModeStr, newModeStr, _pendingNotif.reason);

    g_buzzer.modeChange();
}
