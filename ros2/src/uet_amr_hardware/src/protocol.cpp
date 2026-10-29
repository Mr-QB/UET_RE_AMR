// Copyright (c) 2026 Department of Robotics, University of Engineering and
//                    Technology, Vietnam National University, Hanoi (VNU).
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

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

    namespace
    {
        // Reads fields LSB first at a running bit position, so the calls in
        // FeedbackParser::feed() read like the field table in protocol.hpp.
        class BitReader
        {
        public:
            explicit BitReader(const uint8_t * buf) : buf_(buf) {}
            uint32_t get(unsigned width)
            {
                uint32_t v = 0;
                for (unsigned i = 0; i < width; i++, pos_++) {
                    v |= static_cast<uint32_t>((buf_[pos_ >> 3] >> (pos_ & 7u)) & 1u) << i;
                }
                return v;
            }

        private:
            const uint8_t * buf_;
            unsigned pos_{0};
        };
    }  // namespace

    CommandFrame encodeCommandFrame(int16_t speed_left, int16_t speed_right)
    {
        const uint16_t l = static_cast<uint16_t>(speed_left);
        const uint16_t r = static_cast<uint16_t>(speed_right);
        CommandFrame frame{};
        frame.bytes[0] = kHeader;
        frame.bytes[1] = static_cast<uint8_t>(l & 0xFF);
        frame.bytes[2] = static_cast<uint8_t>(l >> 8);
        frame.bytes[3] = static_cast<uint8_t>(r & 0xFF);
        frame.bytes[4] = static_cast<uint8_t>(r >> 8);
        frame.bytes[5] = 0; // reserved
        frame.bytes[6] = 0; // reserved

        // Plain byte-wise XOR of bytes [0..6], matching the firmware's
        // R_Receive() -- NOT checksum16(), which is the 16-bit alternating
        // fold used only for the 16-byte feedback packet.
        uint8_t checksum = 0;
        for (size_t i = 0; i < kCommandFrameLen - 1; ++i) {
            checksum ^= frame.bytes[i];
        }
        frame.bytes[kCommandFrameLen - 1] = checksum;
        return frame;
    }

    bool FeedbackParser::feed(uint8_t b)
    {
        if (len_ == 0 && b != kHeader) {
            return false;
        }
        buf_[len_++] = b;
        if (len_ < kFeedbackPacketLen) {
            return false;
        }
        len_ = 0;

        const uint16_t expected = checksum16(buf_, kFeedbackPacketLen - 2);
        const uint16_t received = static_cast<uint16_t>(
            buf_[kFeedbackPacketLen - 2] | (buf_[kFeedbackPacketLen - 1] << 8));
        if (received != expected) {
            // Bad packet: a real header may sit inside it, so rescan the tail.
            for (size_t i = 1; i < kFeedbackPacketLen; i++) {
                if (buf_[i] == kHeader) {
                    len_ = kFeedbackPacketLen - i;
                    std::memmove(buf_, &buf_[i], len_);
                    break;
                }
            }
            return false;
        }

        BitReader r(buf_);
        r.get(8);  // header, already matched
        packet_.seq = r.get(7);
        packet_.estop = r.get(1);
        packet_.speed_l = static_cast<int16_t>(r.get(16));
        packet_.speed_r = static_cast<int16_t>(r.get(16));
        packet_.en_tick_l = r.get(14);
        packet_.en_tick_r = r.get(14);
        packet_.temp_c = r.get(7);
        packet_.current_a = r.get(11);
        packet_.battery = r.get(7);
        packet_.stamp = r.get(11);
        return true;
    }

}  // namespace protocol
}  // namespace uet_amr_hardware
