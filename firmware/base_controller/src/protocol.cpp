#include "protocol.h"
#include <string.h>
 
// --- CRC-8, polynomial 0x07, init 0x00 (same table-free bit-banged form both here and in JS) ---
static uint8_t crc8Update(uint8_t crc, uint8_t b) {
    crc ^= b;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 0x80) {
            crc = (uint8_t)((crc << 1) ^ 0x07);
        } else {
            crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}
 
uint8_t Protocol::crc8(uint8_t cmd, uint8_t len, const uint8_t* data) {
    uint8_t crc = 0;
    crc = crc8Update(crc, cmd);
    crc = crc8Update(crc, len);
    for (uint8_t i = 0; i < len; i++) {
        crc = crc8Update(crc, data[i]);
    }
    return crc;
}
 
void Protocol::begin(Stream& serialPort) {
    port = &serialPort;
    state = ParseState::WAIT_HEADER;
    rxIndex = 0;
    resetWatchdog();
}
 
void Protocol::onCommand(ProtocolCommandHandler cb) {
    handler = cb;
}
 
void Protocol::update() {
    if (!port) return;
    while (port->available() > 0) {
        feedByte((uint8_t)port->read());
    }
}
 
void Protocol::feedByte(uint8_t b) {
    switch (state) {
        case ParseState::WAIT_HEADER:
            if (b == PROTOCOL_HEADER_MASTER_TO_SLAVE) {
                state = ParseState::WAIT_CMD;
            }
            break;
 
        case ParseState::WAIT_CMD:
            rxCmd = b;
            state = ParseState::WAIT_LEN;
            break;
 
        case ParseState::WAIT_LEN:
            rxLen = b;
            if (rxLen > PROTOCOL_MAX_DATA_LEN) {
                // Corrupt / unsupported length: resync on the next header instead of overflowing.
                state = ParseState::WAIT_HEADER;
                break;
            }
            rxIndex = 0;
            state = (rxLen == 0) ? ParseState::WAIT_CRC : ParseState::WAIT_DATA;
            break;
 
        case ParseState::WAIT_DATA:
            rxData[rxIndex++] = b;
            if (rxIndex >= rxLen) {
                state = ParseState::WAIT_CRC;
            }
            break;
 
        case ParseState::WAIT_CRC: {
            uint8_t expected = crc8(rxCmd, rxLen, rxData);
            state = ParseState::WAIT_HEADER; // always resync, valid or not
            if (b == expected) {
                lastRxMillis = millis();
                watchdogTriggered = false; // any valid packet from master counts as "alive"
                if (handler) {
                    handler(rxCmd, rxData, rxLen);
                }
            }
            // else: CRC mismatch, silently drop the packet.
            break;
        }
    }
}
 
void Protocol::sendPacket(uint8_t cmd, const uint8_t* data, uint8_t len) {
    if (!port) return;
    uint8_t crc = crc8(cmd, len, data);
    port->write((uint8_t)PROTOCOL_HEADER_SLAVE_TO_MASTER);
    port->write(cmd);
    port->write(len);
    if (len > 0 && data != nullptr) {
        port->write(data, len);
    }
    port->write(crc);
}
 
bool Protocol::checkWatchdog(unsigned long timeoutMs) {
    if (watchdogTriggered) return false; // already reported, don't re-fire until reset by a new packet
    if (millis() - lastRxMillis > timeoutMs) {
        watchdogTriggered = true;
        return true;
    }
    return false;
}
 
void Protocol::resetWatchdog() {
    lastRxMillis = millis();
    watchdogTriggered = false;
}
 
// --- Send helpers ---
 
void Protocol::sendAckPing() {
    sendPacket(CMD_ACK_PING, nullptr, 0);
}
 
void Protocol::sendAckControl() {
    sendPacket(CMD_ACK_CONTROL, nullptr, 0);
}
 
void Protocol::sendWatchdogTriggered() {
    sendPacket(CMD_WATCHDOG, nullptr, 0);
}
 
void Protocol::sendOdometry(float x, float y, float theta) {
    uint8_t buf[12];
    memcpy(buf + 0, &x, 4);
    memcpy(buf + 4, &y, 4);
    memcpy(buf + 8, &theta, 4);
    sendPacket(CMD_ODOMETRY, buf, sizeof(buf));
}
 
void Protocol::sendVelocity(float left, float right) {
    uint8_t buf[8];
    memcpy(buf + 0, &left, 4);
    memcpy(buf + 4, &right, 4);
    sendPacket(CMD_VELOCITY, buf, sizeof(buf));
}
 
void Protocol::sendEncoderPos(int32_t left, int32_t right) {
    uint8_t buf[8];
    memcpy(buf + 0, &left, 4);
    memcpy(buf + 4, &right, 4);
    sendPacket(CMD_ENCODER_POS, buf, sizeof(buf));
}
 
void Protocol::sendStatus(float battery, float current) {
    uint8_t buf[8];
    memcpy(buf + 0, &battery, 4);
    memcpy(buf + 4, &current, 4);
    sendPacket(CMD_STATUS, buf, sizeof(buf));
}