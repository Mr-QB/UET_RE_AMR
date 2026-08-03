// Copyright (c) 2024 UET Robotics & Electronics Club
// Licensed under the MIT License

#pragma once

#include <cstddef>
#include <cstdint>

// =============================================================================
// C++ port of firmware/base_controller/include/serial_protocol.h /.cpp --
// must stay in sync with that file (and hoverboardSerial.h, whose
// `Odom odom(wheel_radius_cm, wheel_base_cm, ...)` constants are baked into
// the x/y odometry this protocol reports).
//
// Master (PC/ROS2) -> Slave (ESP32), fixed 8 bytes:
//   [0]     header = 0xAA
//   [1]     cmd
//   [2..3]  speed_left  int16, little-endian (only meaningful for SetVelocity)
//   [4..5]  speed_right int16, little-endian
//   [6..7]  checksum    uint16, little-endian, XOR of bytes [0..5]
//
// cmd values:
//   0x00 RequestVelocity  -- ask for last measured wheel speed
//   0x01 RequestOdometry  -- ask for onboard x/y/theta odometry
//   0x02 RequestStatus    -- ask for battery/temperature
//   0x03 EmergencyStop    -- stop immediately (firmware replies with the
//                            RequestVelocity-shaped packet, no special case)
//   anything else (this driver uses 0xFF) -- SetVelocity: speed_left/right
//     are applied as the new commanded wheel speed. Firmware sends NO reply
//     for this case.
//
// Slave -> Master reply, header 0xBB, cmd echoes the request cmd:
//   RequestVelocity (0x00) reply, 8 bytes total:
//     [2..3] measured speed_left  int16 LE
//     [4..5] measured speed_right int16 LE
//     [6..7] checksum uint16 LE (XOR of bytes [0..5])
//   RequestOdometry (0x01) reply, 14 bytes total:
//     [2..5]   x float32 LE, centimeters
//     [6..9]   y float32 LE, centimeters
//     [10..11] theta int16 LE, radians * 10 (firmware casts a possibly
//              negative float into a uint16_t field; reading it back as a
//              signed int16 recovers the value on every compiler observed
//              so far, but this is worth re-checking against real hardware)
//     [12..13] checksum uint16 LE (XOR of bytes [0..11])
//   RequestStatus (0x02) reply, 8 bytes total:
//     [2..3] battery uint16 LE
//     [4..5] temperature int16 LE
//     [6..7] checksum uint16 LE (XOR of bytes [0..5])
//
// Checksum: 16-bit accumulator, XORed one byte at a time over all bytes
// preceding the checksum field itself (matches SerialProtocol::calcChecksum).
//
// Note there is no per-wheel encoder tick feedback in this protocol (unlike
// the older protocol.h it replaced) -- only measured speed and onboard
// (x, y, theta) odometry. See amr_hardware_interface.cpp's read() for how
// wheel position is recovered from odometry deltas.
// =============================================================================

namespace uet_amr_hardware
{
namespace protocol
{

constexpr uint8_t kHeaderMasterToSlave = 0xAA;
constexpr uint8_t kHeaderSlaveToMaster = 0xBB;

enum class Cmd : uint8_t
{
  RequestVelocity = 0x00,
  RequestOdometry = 0x01,
  RequestStatus = 0x02,
  EmergencyStop = 0x03,
  SetVelocity = 0xFF,
};

// XOR checksum over data[0..len-1], matches firmware's calcChecksum().
uint16_t checksum16(const uint8_t * data, size_t len);

// Builds a master->slave command frame (8 bytes): header, cmd, speed_left,
// speed_right, checksum. speed_left/right are ignored by the firmware for
// request commands but must still be present for the checksum to line up.
struct CommandFrame
{
  uint8_t bytes[8];
};
CommandFrame encodeCommandFrame(Cmd cmd, int16_t speed_left, int16_t speed_right);

// Byte-fed parser for slave->master reply frames (header 0xBB). The reply
// length is determined entirely by the cmd byte (there is no explicit
// length field in this protocol); an unrecognized cmd drops the frame and
// resyncs on the next header byte, mirroring the firmware's own tolerant
// behavior on its receive side.
class FrameParser
{
public:
  // Feed one received byte. Returns true exactly when a full,
  // checksum-valid reply frame has just been assembled; cmd()/payload()/
  // payloadLen() are valid only immediately after a call that returned true.
  bool feed(uint8_t byte);

  Cmd cmd() const {return static_cast<Cmd>(cmd_);}
  const uint8_t * payload() const {return body_;}
  uint8_t payloadLen() const {return payload_len_;}

private:
  enum class State {WaitHeader, WaitCmd, WaitBody};

  static constexpr uint8_t kMaxPayloadLen = 10;  // RequestOdometry reply is the largest

  State state_{State::WaitHeader};
  uint8_t cmd_{0};
  uint8_t payload_len_{0};
  uint8_t body_len_{0};  // payload_len_ + 2 (checksum bytes)
  uint8_t body_index_{0};
  uint8_t body_[kMaxPayloadLen + 2]{};
};

}  // namespace protocol
}  // namespace uet_amr_hardware
