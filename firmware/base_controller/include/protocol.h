#pragma once
 
#include <Arduino.h>
 
// =============================================================================
// Serial protocol between master (PC) and slave (this MCU).
//
// Packet format (both directions), baud 115200:
//   [HEADER][CMD][LEN][DATA...][CRC]
//     HEADER : 1 byte  - 0xAA master->slave, 0xBB slave->master
//     CMD    : 1 byte  - command id, see ProtocolCmd below
//     LEN    : 1 byte  - number of bytes in DATA (0-PROTOCOL_MAX_DATA_LEN)
//     DATA   : LEN bytes - payload, multi-byte numbers are little-endian
//     CRC    : 1 byte  - CRC-8 (poly 0x07, init 0x00) over CMD, LEN, DATA
//
// Several command ids are shared between directions: master sends them with
// LEN=0 as a "request", slave answers with the same cmd id and real data.
// =============================================================================
 
#define PROTOCOL_HEADER_MASTER_TO_SLAVE 0xAA
#define PROTOCOL_HEADER_SLAVE_TO_MASTER 0xBB
#define PROTOCOL_MAX_DATA_LEN 32
#define PROTOCOL_DEFAULT_WATCHDOG_MS 200 // master pings @50Hz (20ms); allow some margin
 
enum ProtocolCmd : uint8_t {
    // --- Communicate (master -> slave) ---
    CMD_PING            = 0x00, // master asks: are you alive?
    CMD_START           = 0x01, // start the slave
    CMD_STOP            = 0x02, // emergency stop
    CMD_RESET_ENCODER   = 0x03, // reset position encoder + velocity
    CMD_RESET_ODOMETRY  = 0x04, // reset odometry (X, Y, Theta)
 
    // --- Control (master -> slave: command; also used as a data "request") ---
    CMD_SET_VELOCITY        = 0x10, // data: 4 byte float left, 4 byte float right
    CMD_SET_SMOOTH_VELOCITY = 0x11, // data: 4 byte float left, 4 byte float right
    CMD_ODOMETRY             = 0x12, // request (len=0) / response: 4B X, 4B Y, 4B Theta (float)
    CMD_VELOCITY             = 0x13, // request (len=0) / response: 4B left, 4B right (float)
    CMD_ENCODER_POS           = 0x14, // request (len=0) / response: 4B left, 4B right (int32_t)
    CMD_STATUS                = 0x15, // request (len=0) / response: 4B battery, 4B current (float)
 
    // --- Slave -> master only ---
    CMD_ACK_PING     = 0x20, // reply to CMD_PING
    CMD_ACK_CONTROL  = 0x21, // generic reply to a control command (start/stop/reset/setVelocity/...)
    CMD_WATCHDOG     = 0x22  // slave hasn't heard from master in a while, went to emergency stop
};
 
// Called once per fully-received, CRC-valid packet.
typedef void (*ProtocolCommandHandler)(uint8_t cmd, const uint8_t* data, uint8_t len);
 
class Protocol {
private:
    enum class ParseState : uint8_t { WAIT_HEADER, WAIT_CMD, WAIT_LEN, WAIT_DATA, WAIT_CRC };
 
    Stream* port = nullptr;
 
    ParseState state = ParseState::WAIT_HEADER;
    uint8_t rxCmd = 0;
    uint8_t rxLen = 0;
    uint8_t rxIndex = 0;
    uint8_t rxData[PROTOCOL_MAX_DATA_LEN];
 
    ProtocolCommandHandler handler = nullptr;
 
    unsigned long lastRxMillis = 0;
    bool watchdogTriggered = false;
 
    void feedByte(uint8_t b);
    void sendPacket(uint8_t cmd, const uint8_t* data, uint8_t len);
 
public:
    static uint8_t crc8(uint8_t cmd, uint8_t len, const uint8_t* data);
 
    void begin(Stream& serialPort);              // e.g. protocol.begin(Serial);
    void onCommand(ProtocolCommandHandler cb);    // register the packet handler
    void update();                                // call every loop(): reads & parses incoming bytes
 
    // --- Watchdog ---
    // Call every loop(). Returns true exactly once, the moment the timeout is
    // crossed (edge-triggered) so the caller can react (stop motors, send CMD_WATCHDOG).
    bool checkWatchdog(unsigned long timeoutMs = PROTOCOL_DEFAULT_WATCHDOG_MS);
    void resetWatchdog(); // call in setup(), so it doesn't fire before the first packet arrives
 
    // --- Send helpers (slave -> master) ---
    void sendAckPing();
    void sendAckControl();
    void sendWatchdogTriggered();
    void sendOdometry(float x, float y, float theta);
    void sendVelocity(float left, float right);
    void sendEncoderPos(int32_t left, int32_t right);
    void sendStatus(float battery, float current);
};