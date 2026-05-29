#ifndef SERIAL_PROTOCOL_H

#define SERIAL_PROTOCOL_H

#include <Arduino.h>

#include "../config/HardwareConfig.h"

class SerialProtocol {
public:

    // Komut modu sabitleri
    static constexpr uint8_t GIMBAL_MODE_GROUND_LOCK = 0; // Yer kilit (0°/0°)
    static constexpr uint8_t GIMBAL_MODE_TRACKING    = 1; // ROS2 delta takibi
    static constexpr uint8_t GIMBAL_MODE_JOYSTICK    = 2; // Joystick hiz kontrolu

    struct GimbalCommand {
        float pan_delta;            // Pan aci degisimi (derece) — TRACKING modunda kullanilir
        float tilt_delta;           // Tilt aci degisimi (derece) — TRACKING modunda kullanilir
        float feedforward_vel_pan;  // Pan hizi (derece/saniye) — JOYSTICK modunda kullanilir
        float feedforward_vel_tilt; // Tilt hizi (derece/saniye) — JOYSTICK modunda kullanilir
        bool laser_enable;          // Lazer etkin mi?
        bool laser_fire;            // Lazer ates et
        uint8_t mode;               // Gimbal modu (GIMBAL_MODE_*)
        uint32_t timestamp_ms;      // Komut alinma zamani (milisaniye)

        GimbalCommand()
            : pan_delta(0), tilt_delta(0)
            , feedforward_vel_pan(0), feedforward_vel_tilt(0)
            , laser_enable(false), laser_fire(false)
            , mode(GIMBAL_MODE_GROUND_LOCK)
            , timestamp_ms(0) {}
    };

    struct TelemetryPacket {
        float current_world_pan;    // Dunya referansinda mevcut pan (derece)
        float current_world_tilt;   // Dunya referansinda mevcut tilt (derece)
        float encoder_pan;          // Ham encoder pan okumasi (derece)
        float encoder_tilt;         // Ham encoder tilt okumasi (derece)
        float error_pan;            // Pan takip hatasi (derece)
        float error_tilt;           // Tilt takip hatasi (derece)
        uint16_t status_flags;      // Durum bayraklari
        uint32_t timestamp_ms;      // Zaman damgasi (milisaniye)

        TelemetryPacket()
            : current_world_pan(0), current_world_tilt(0)
            , encoder_pan(0), encoder_tilt(0)
            , error_pan(0), error_tilt(0)
            , status_flags(0), timestamp_ms(0) {}
    };

    enum StatusFlags {
        STATUS_OK              = 0x0000,  // Her sey normal
        STATUS_PAN_AT_LIMIT    = 0x0001,  // Pan ekseni limite ulasti
        STATUS_TILT_AT_LIMIT   = 0x0002,  // Tilt ekseni limite ulasti
        STATUS_IMU_FAULT       = 0x0004,  // IMU arizasi
        STATUS_ENCODER_FAULT   = 0x0008,  // Encoder arizasi
        STATUS_SAFE_MODE       = 0x0010,  // Guvenli mod aktif
        STATUS_CALIBRATING     = 0x0020   // Kalibrasyon devam ediyor
    };

    SerialProtocol();

    bool parsePacket(const uint8_t *buffer, size_t len, GimbalCommand &cmd);

    size_t buildTelemetry(const TelemetryPacket &telem, uint8_t *buffer);

private:
    uint16_t calculateCRC16(const uint8_t *data, size_t len);

    bool validateHeaders(const uint8_t *buffer);

    // Paket boyutlari
    static constexpr size_t COMMAND_PACKET_SIZE = 24;    // Komut paketi boyutu
    static constexpr size_t TELEMETRY_PACKET_SIZE = 32;  // Telemetri paketi boyutu
};

#endif
