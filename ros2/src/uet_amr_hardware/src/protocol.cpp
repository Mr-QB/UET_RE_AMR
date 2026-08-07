// Copyright (c) 2024 UET Robotics & Electronics Club
// Licensed under the MIT License

#include "uet_amr_hardware/protocol.hpp"

#include <cstring>

namespace uet_amr_hardware
{
namespace protocol
{

uint16_t checksum16(const uint8_t * data, size_t len)
{
  uint16_t c = 0;
  for (size_t i = 0; i < len; i++) {
    c ^= static_cast<uint16_t>(data[i]) << ((i & 1u) * 8u);
  }
  return c;
}

CommandFrame encodeCommandFrame(int16_t speed_left, int16_t speed_right)
{
  CommandFrame frame{};
  frame.bytes[0] = kHeader;
  frame.bytes[1] = static_cast<uint8_t>(speed_left & 0xFF);
  frame.bytes[2] = static_cast<uint8_t>((speed_left >> 8) & 0xFF);
  frame.bytes[3] = static_cast<uint8_t>(speed_right & 0xFF);
  frame.bytes[4] = static_cast<uint8_t>((speed_right >> 8) & 0xFF);

  // Plain byte-wise XOR of bytes [0..4], matching the firmware's R_Receive()
  // in serial_protocol.cpp -- NOT checksum16(), which is the 16-bit
  // alternating fold used only for the 16-byte feedback packet.
  uint8_t checksum = 0;
  for (size_t i = 0; i < 5; ++i) {
    checksum ^= frame.bytes[i];
  }
  frame.bytes[5] = checksum;
  return frame;
}

bool FeedbackParser::feed(uint8_t b)
{
  switch (state_) {
    case State::WaitHeader:
      if (b == kHeader) {
        body_[0] = b;
        body_index_ = 1;
        state_ = State::WaitBody;
      }
      break;

    case State::WaitBody:
      body_[body_index_++] = b;
      if (body_index_ >= kFeedbackPacketLen) {
        state_ = State::WaitHeader;

        const uint16_t expected = checksum16(body_, kFeedbackPacketLen - 2);
        const uint16_t received = static_cast<uint16_t>(
          body_[kFeedbackPacketLen - 2] | (body_[kFeedbackPacketLen - 1] << 8));
        if (received != expected) {
          break;  // checksum mismatch, drop and resync on the next header
        }

        const uint32_t w1 = static_cast<uint32_t>(body_[0]) |
          (static_cast<uint32_t>(body_[1]) << 8) |
          (static_cast<uint32_t>(body_[2]) << 16) |
          (static_cast<uint32_t>(body_[3]) << 24);
        const uint32_t w2 = static_cast<uint32_t>(body_[4]) |
          (static_cast<uint32_t>(body_[5]) << 8) |
          (static_cast<uint32_t>(body_[6]) << 16) |
          (static_cast<uint32_t>(body_[7]) << 24);
        const uint32_t w3 = static_cast<uint32_t>(body_[8]) |
          (static_cast<uint32_t>(body_[9]) << 8) |
          (static_cast<uint32_t>(body_[10]) << 16) |
          (static_cast<uint32_t>(body_[11]) << 24);
        const uint32_t optional_2 = static_cast<uint32_t>(body_[12]) |
          (static_cast<uint32_t>(body_[13]) << 8);

        packet_.sys_status = (w1 >> 8) & 0x3;
        packet_.speed_l = static_cast<int8_t>((w1 >> 16) & 0xFF);
        packet_.speed_r = static_cast<int8_t>((w1 >> 24) & 0xFF);
        packet_.en_tick_l = w2 & 0x3FFF;
        packet_.en_tick_r = (w2 >> 14) & 0x3FFF;
        packet_.temp_c = w3 & 0x7F;
        packet_.current_a = (w3 >> 7) & 0x7FF;
        packet_.battery = (w3 >> 18) & 0x7F;
        packet_.charging = (w3 >> 25) & 0x1;
        packet_.optional_1 = (w3 >> 26) & 0x3F;
        packet_.optional_2 = optional_2;

        return true;
      }
      break;
  }
  return false;
}

}  // namespace protocol
}  // namespace uet_amr_hardware
