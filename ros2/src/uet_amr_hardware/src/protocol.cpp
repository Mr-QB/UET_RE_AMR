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
  uint16_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

CommandFrame encodeCommandFrame(Cmd cmd, int16_t speed_left, int16_t speed_right)
{
  CommandFrame frame{};
  frame.bytes[0] = kHeaderMasterToSlave;
  frame.bytes[1] = static_cast<uint8_t>(cmd);
  std::memcpy(frame.bytes + 2, &speed_left, 2);
  std::memcpy(frame.bytes + 4, &speed_right, 2);
  uint16_t crc = checksum16(frame.bytes, 6);
  std::memcpy(frame.bytes + 6, &crc, 2);
  return frame;
}

bool FrameParser::feed(uint8_t b)
{
  switch (state_) {
    case State::WaitHeader:
      if (b == kHeaderSlaveToMaster) {
        state_ = State::WaitCmd;
      }
      break;

    case State::WaitCmd: {
        cmd_ = b;
        switch (static_cast<Cmd>(cmd_)) {
          case Cmd::RequestVelocity:
          case Cmd::RequestStatus:
            payload_len_ = 4;
            break;
          case Cmd::RequestOdometry:
            payload_len_ = 10;
            break;
          default:
            // Unrecognized cmd for a slave->master frame -- resync.
            state_ = State::WaitHeader;
            return false;
        }
        body_len_ = static_cast<uint8_t>(payload_len_ + 2);
        body_index_ = 0;
        state_ = State::WaitBody;
        break;
      }

    case State::WaitBody:
      body_[body_index_++] = b;
      if (body_index_ >= body_len_) {
        state_ = State::WaitHeader;

        uint8_t header_and_payload[2 + kMaxPayloadLen];
        header_and_payload[0] = kHeaderSlaveToMaster;
        header_and_payload[1] = cmd_;
        std::memcpy(header_and_payload + 2, body_, payload_len_);

        uint16_t expected = checksum16(header_and_payload, 2u + payload_len_);
        uint16_t received;
        std::memcpy(&received, body_ + payload_len_, 2);

        if (received == expected) {
          return true;
        }
        // else: checksum mismatch, silently drop (already resynced above).
      }
      break;
  }
  return false;
}

}  // namespace protocol
}  // namespace uet_amr_hardware
