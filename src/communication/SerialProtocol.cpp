#include "SerialProtocol.h"
#include <string.h>

SerialProtocol::SerialProtocol() {
}

bool SerialProtocol::parsePacket(const uint8_t *buffer, size_t len, GimbalCommand &cmd) {//gelen paketi çözümler

    if (len < COMMAND_PACKET_SIZE) {//uzunluk doğru mu
        return false;
    }

    if (!validateHeaders(buffer)) {//headerlar doğru mu
        return false;
    }

    uint16_t receivedCRC = (buffer[21] << 8) | buffer[20];

    uint16_t calculatedCRC = calculateCRC16(buffer, 20);

    if (receivedCRC != calculatedCRC) {
        Serial.printf("CRC uyusmazligi: alinan 0x%04X, hesaplanan 0x%04X\n",
                      receivedCRC, calculatedCRC);
        return false;
    }

    // Pan delta: byte 2-5
    memcpy(&cmd.pan_delta, &buffer[2], 4);

    // Tilt delta: byte 6-9
    memcpy(&cmd.tilt_delta, &buffer[6], 4);

    // Feedforward hizlar: byte 10-17
    memcpy(&cmd.feedforward_vel_pan, &buffer[10], 4);

    memcpy(&cmd.feedforward_vel_tilt, &buffer[14], 4);

    // Lazer kontrol byte'lari: byte 18-19
    cmd.laser_enable = buffer[18];

    cmd.laser_fire = buffer[19];

    // Zaman damgasi: paketin alindigi an
    cmd.timestamp_ms = millis();

    return true;
}

size_t SerialProtocol::buildTelemetry(const TelemetryPacket &telem, uint8_t *buffer) {//ros2 ye gidecek verileri paketler

    buffer[0] = PACKET_HEADER1;
    buffer[1] = PACKET_HEADER2;

    // Dunya koordinatlari (float, 4 byte her biri)
    memcpy(&buffer[2], &telem.current_world_pan, 4);

    memcpy(&buffer[6], &telem.current_world_tilt, 4);

    // Ham encoder okumasi (float, 4 byte her biri)
    memcpy(&buffer[10], &telem.encoder_pan, 4);

    memcpy(&buffer[14], &telem.encoder_tilt, 4);

    // Takip hatasi (float, 4 byte her biri)
    memcpy(&buffer[18], &telem.error_pan, 4);

    memcpy(&buffer[22], &telem.error_tilt, 4);

    // Durum bayraklari (16-bit, little-endian)
    buffer[26] = telem.status_flags & 0xFF;         // Dusuk byte

    buffer[27] = (telem.status_flags >> 8) & 0xFF;  // Yuksek byte

    // Zaman damgasi (32-bit, little-endian)
    memcpy(&buffer[28], &telem.timestamp_ms, 4);

    // CRC16 hesapla ve ekle
    uint16_t crc = calculateCRC16(buffer, 30);

    buffer[30] = crc & 0xFF;          // Dusuk byte

    buffer[31] = (crc >> 8) & 0xFF;   // Yuksek byte

    return TELEMETRY_PACKET_SIZE;
}

bool SerialProtocol::validateHeaders(const uint8_t *buffer) {//header kontrol
    return (buffer[0] == PACKET_HEADER1 && buffer[1] == PACKET_HEADER2);
}

uint16_t SerialProtocol::calculateCRC16(const uint8_t *data, size_t len) {//CRC hesaplama
    // CCITT-FALSE CRC16:
    // - Baslangic degeri: 0xFFFF
    // - Polinom: 0x1021 (x^16 + x^12 + x^5 + 1)
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}
