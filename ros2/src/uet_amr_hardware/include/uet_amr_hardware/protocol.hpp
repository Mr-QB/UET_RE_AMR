// Copyright (c) 2024 UET Robotics & Electronics Club
// Licensed under the MIT License

#pragma once

#include <cstdint>
#include <vector>

// =============================================================================
// C++ port of firmware/base_controller/include/protocol.h — must stay in sync
// with that file. See it for the authoritative protocol description.
//
// Packet format (both directions), baud 115200:
//   [HEADER][CMD][LEN][DATA...][CRC]
//     HEADER : 1 byte  - 0xAA master->slave, 0xBB slave->master
//     CMD    : 1 byte  - command id, see Cmd below
//     LEN    : 1 byte  - number of bytes in DATA (0-kMaxDataLen)
//     DATA   : LEN bytes - payload, multi-byte numbers are little-endian
//     CRC    : 1 byte  - CRC-8 (poly 0x07, init 0x00) over CMD, LEN, DATA
// =============================================================================

namespace uet_amr_hardware
{
namespace protocol
{

constexpr uint8_t kHeaderMasterToSlave = 0xAA;
constexpr uint8_t kHeaderSlaveToMaster = 0xBB;
constexpr uint8_t kMaxDataLen = 32;
constexpr int kDefaultWatchdogMs = 200;

enum class Cmd : uint8_t
{
  Ping = 0x00,
  Start = 0x01,
  Stop = 0x02,
  ResetEncoder = 0x03,
  ResetOdometry = 0x04,

  SetVelocity = 0x10,
  SetSmoothVelocity = 0x11,
  Odometry = 0x12,
  Velocity = 0x13,
  EncoderPos = 0x14,
  Status = 0x15,

  AckPing = 0x20,
  AckControl = 0x21,
  Watchdog = 0x22,
};

// CRC-8, poly 0x07, init 0x00 — must match firmware's crc8Update()/crc8().
uint8_t crc8(uint8_t cmd, uint8_t len, const uint8_t * data);

// Builds a master->slave frame: [0xAA][cmd][len][data...][crc].
std::vector<uint8_t> encodeFrame(Cmd cmd, const uint8_t * data, uint8_t len);

// Byte-fed parser for slave->master frames (header 0xBB). Mirrors the
// tolerant resync behavior of firmware's Protocol::feedByte(): any bad
// length or bad CRC just drops the in-progress frame and waits for the
// next header byte, rather than erroring out.
class FrameParser
{
public:
  // Feed one received byte. Returns true exactly when a full, CRC-valid
  // frame has just been assembled; cmd()/data()/len() are valid only
  // immediately after a call that returned true.
  bool feed(uint8_t byte);

  Cmd cmd() const {return static_cast<Cmd>(rx_cmd_);}
  const uint8_t * data() const {return rx_data_;}
  uint8_t len() const {return rx_len_;}

private:
  enum class ParseState : uint8_t {WaitHeader, WaitCmd, WaitLen, WaitData, WaitCrc};

  ParseState state_{ParseState::WaitHeader};
  uint8_t rx_cmd_{0};
  uint8_t rx_len_{0};
  uint8_t rx_index_{0};
  uint8_t rx_data_[kMaxDataLen];
};

}  // namespace protocol
}  // namespace uet_amr_hardware
