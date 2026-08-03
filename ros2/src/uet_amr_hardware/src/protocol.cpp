// Copyright (c) 2024 UET Robotics & Electronics Club
// Licensed under the MIT License

#include "uet_amr_hardware/protocol.hpp"

namespace uet_amr_hardware
{
namespace protocol
{

namespace
{
uint8_t crc8Update(uint8_t crc, uint8_t b)
{
  crc ^= b;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x80) {
      crc = static_cast<uint8_t>((crc << 1) ^ 0x07);
    } else {
      crc = static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}
}  // namespace

uint8_t crc8(uint8_t cmd, uint8_t len, const uint8_t * data)
{
  uint8_t crc = 0;
  crc = crc8Update(crc, cmd);
  crc = crc8Update(crc, len);
  for (uint8_t i = 0; i < len; i++) {
    crc = crc8Update(crc, data[i]);
  }
  return crc;
}

std::vector<uint8_t> encodeFrame(Cmd cmd, const uint8_t * data, uint8_t len)
{
  std::vector<uint8_t> frame;
  frame.reserve(4u + len);
  frame.push_back(kHeaderMasterToSlave);
  frame.push_back(static_cast<uint8_t>(cmd));
  frame.push_back(len);
  if (len > 0 && data != nullptr) {
    frame.insert(frame.end(), data, data + len);
  }
  frame.push_back(crc8(static_cast<uint8_t>(cmd), len, data));
  return frame;
}

bool FrameParser::feed(uint8_t b)
{
  switch (state_) {
    case ParseState::WaitHeader:
      if (b == kHeaderSlaveToMaster) {
        state_ = ParseState::WaitCmd;
      }
      break;

    case ParseState::WaitCmd:
      rx_cmd_ = b;
      state_ = ParseState::WaitLen;
      break;

    case ParseState::WaitLen:
      rx_len_ = b;
      if (rx_len_ > kMaxDataLen) {
        state_ = ParseState::WaitHeader;
        break;
      }
      rx_index_ = 0;
      state_ = (rx_len_ == 0) ? ParseState::WaitCrc : ParseState::WaitData;
      break;

    case ParseState::WaitData:
      rx_data_[rx_index_++] = b;
      if (rx_index_ >= rx_len_) {
        state_ = ParseState::WaitCrc;
      }
      break;

    case ParseState::WaitCrc: {
        uint8_t expected = crc8(rx_cmd_, rx_len_, rx_data_);
        state_ = ParseState::WaitHeader;
        if (b == expected) {
          return true;
        }
        break;
      }
  }
  return false;
}

}  // namespace protocol
}  // namespace uet_amr_hardware
