// Copyright (c) 2024 UET Robotics & Electronics Club
// Licensed under the MIT License

#pragma once

#include <cstddef>
#include <cstdint>

// =============================================================================
// C++ port of firmware/amr_uart_bridge/include/ros_protocol.h -- must
// stay in sync with that file (and hover_protocol.h, whose
// `Odom odom(wheel_radius_cm, wheel_base_cm, ticks_per_rev, encoder_max)`
// constants determine how raw encoder ticks map to wheel angle below).
//
// This is a push-style protocol: the firmware streams a feedback packet
// every time it gets a fresh reading from the hoverboard, unprompted. There
// is no request/response -- the host just keeps the most recent command
// frame flowing so the firmware's no-traffic watchdog stays fed.
//
// Slave (ESP32) -> Master (ROS2) feedback packet, 16 bytes / 128 bits,
// bit-packed. FeedbackPacket below is a bitfield struct used only as the
// *decoded value* representation (readable field names/widths) -- the wire
// bytes are never reinterpreted as this struct directly, because C++
// bitfield/struct memory layout (bit order, padding) is
// implementation-defined and differs across compilers/architectures.
// FeedbackParser decodes the wire bytes by hand with explicit shifts, the
// same way the firmware's packPacket() encodes them.
//
//   Word 1 (bytes 0..3,   bits 0..31):
//     header(8) sys_status(2) reserved1(6) speed_l(8) speed_r(8)
//   Word 2 (bytes 4..7,   bits 32..63):
//     en_tick_l(14) en_tick_r(14) reserved2(4)
//   Word 3 (bytes 8..11,  bits 64..95):
//     temp_c(7) current_a(11) battery(7) charging(1) optional_1(6)
//   Word 4 (bytes 12..15, bits 96..127):
//     optional_2(16) checksum(16)
//
//   Each word is little-endian (LSB first). checksum is a 16-bit XOR fold
//   (see checksum16()) over payload bytes [0..13].
//
//   sys_status:  0=Idle, 1=Nav, 2=Manual, 3=Fault
//   speed_l/r:   measured speed, signed, -100..100
//   en_tick_l/r: wheel encoder ticks, 0..encoder_max (wraps)
//   temp_c:      board temperature, 0..127 degC
//   current_a:   raw; divide by 100.0f -> 0.00..20.47 A
//   battery:     0..100 %
//   charging:    0=discharging, 1=charging
//   optional_1/2: expansion payload, currently unused
//
// Master (ROS2) -> Slave (ESP32) command frame, 8 bytes. CommandPacket below
// is a bitfield struct purely for readable field names/widths, the same way
// FeedbackPacket is for the feedback packet -- it is never reinterpreted
// from the raw bytes directly.
//
//   left/right are clamped to -100..100, so they're packed as 8-bit fields
//   (matching how FeedbackPacket encodes measured speed) instead of a full
//   int16 -- the bits saved go to `reserved` for future expansion (e.g.
//   enable/estop/mode flags) without growing the frame again.
//
//   [0]    header   uint8  0xAA
//   [1]    left     int8   commanded left speed (firmware clamps to
//                          -100..100)
//   [2]    right    int8   commanded right speed (firmware clamps to
//                          -100..100)
//   [3..6] reserved uint32 LE, expansion payload, currently unused (0)
//   [7]    checksum uint8  XOR of bytes [0..6]
//
// Wheel encoder ticks wrap modulo encoder_max and must be turned into a
// signed delta the same way the firmware's own Odom::wrapDelta() does
// before being accumulated into a wheel angle -- see
// AmrHardwareInterface::read().
// =============================================================================

namespace uet_amr_hardware
{
namespace protocol
{

constexpr uint8_t kHeader = 0xAA;
constexpr size_t kFeedbackPacketLen = 16;
constexpr size_t kCommandFrameLen = 8;

// XOR-fold checksum matching the firmware's checksum16(): byte i is XORed
// into the low half of the accumulator if i is even, the high half if odd.
uint16_t checksum16(const uint8_t * data, size_t len);

// Decoded master->slave command values. A bitfield struct purely for
// readable field names/widths -- mirrors the firmware's CommandPacket in
// ros_protocol.h.
struct CommandPacket
{
  int32_t speed_l : 8;
  int32_t speed_r : 8;
  uint32_t reserved : 32;
};

// Builds a master->slave command frame (8 bytes): header, left, right,
// reserved, checksum. left/right are the raw command sent as-is (truncated
// to int8 on the wire); the firmware clamps them to -100..100 on its side.
struct CommandFrame
{
  uint8_t bytes[kCommandFrameLen];
};
CommandFrame encodeCommandFrame(int16_t speed_left, int16_t speed_right);

// Decoded slave->master feedback packet. A bitfield struct purely for
// readable field names/widths -- populated field-by-field from the manual
// bit-unpack in FeedbackParser::feed(), never by reinterpreting raw bytes.
struct FeedbackPacket
{
  uint32_t sys_status : 2;
  int32_t speed_l : 8;
  int32_t speed_r : 8;
  uint32_t en_tick_l : 14;
  uint32_t en_tick_r : 14;
  uint32_t temp_c : 7;
  uint32_t current_a : 11;  // raw; divide by 100.0f for Amps
  uint32_t battery : 7;
  uint32_t charging : 1;
  uint32_t optional_1 : 6;
  uint32_t optional_2 : 16;
};

// Byte-fed parser for slave->master feedback packets (header 0xAA, fixed
// length kFeedbackPacketLen). Resyncs on the next header byte whenever a
// checksum fails, mirroring the firmware's own tolerant receive behavior.
class FeedbackParser
{
public:
  // Feed one received byte. Returns true exactly when a full,
  // checksum-valid packet has just been assembled; packet() is valid only
  // immediately after a call that returned true.
  bool feed(uint8_t byte);

  const FeedbackPacket & packet() const {return packet_;}

private:
  enum class State {WaitHeader, WaitBody};

  State state_{State::WaitHeader};
  uint8_t body_[kFeedbackPacketLen]{};
  uint8_t body_index_{0};
  FeedbackPacket packet_{};
};

}  // namespace protocol
}  // namespace uet_amr_hardware
