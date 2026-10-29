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

#pragma once

#include <cstddef>
#include <cstdint>

// =============================================================================
// C++ port of firmware/amr_uart_bridge/include/ros_protocol.h -- must stay in
// sync with that file.
//
// This is a push-style protocol: the firmware streams a feedback packet every
// time it gets a fresh reading from the hoverboard, unprompted. There is no
// request/response -- the host just keeps the most recent command frame
// flowing so the firmware's no-traffic watchdog stays fed.
//
// Slave (ESP32) -> Master (ROS2) feedback packet, 16 bytes / 128 bits,
// bit-packed. FeedbackPacket below is a bitfield struct used only as the
// *decoded value* representation -- the wire bytes are never reinterpreted as
// this struct directly (bitfield layout is implementation-defined).
// FeedbackParser decodes the wire bytes by hand, the same way the firmware's
// packPacket() encodes them.
//
// The 16 bytes are one little-endian 128-bit value: bit 0 is the LSB of
// byte 0. Fields follow each other in this order, no padding:
//
//   field       bits      size  meaning
//   header      0..7       8    0xAA
//   seq         8..14      7    wraps at 128; a gap means lost packets
//   estop       15         1    0=released, 1=e-stop active
//   speed_l     16..31    16    measured speed, signed, raw hoverboard rpm
//   speed_r     32..47    16    measured speed, signed, raw hoverboard rpm
//   en_tick_l   48..61    14    wheel encoder ticks, 0..encoder_max (wraps)
//   en_tick_r   62..75    14    wheel encoder ticks, 0..encoder_max (wraps)
//   temp_c      76..82     7    board temperature, 0..127 degC
//   current_a   83..93    11    raw; divide by 100 -> 0.00..20.47 A
//   battery     94..100    7    0..100 %
//   stamp       101..111  11    sample time, 256 us units, wraps every ~524 ms
//   checksum    112..127  16    16-bit XOR fold (checksum16()) over bytes [0..13]
//
// Master (ROS2) -> Slave (ESP32) command frame, 8 bytes, byte-aligned:
//
//   [0]    header   uint8  0xAA
//   [1..2] left     int16  LE, commanded left speed (firmware clamps to
//                          +-1000)
//   [3..4] right    int16  LE, commanded right speed
//   [5..6] reserved uint16 LE, expansion payload, currently unused (0)
//   [7]    checksum uint8  XOR of bytes [0..6]
//
// Wheel encoder ticks wrap modulo encoder_max and must be turned into a
// signed delta before being accumulated into a wheel angle -- see
// AmrHardwareInterface::wrapTickDelta().
// =============================================================================

namespace uet_amr_hardware
{
namespace protocol
{

    constexpr uint8_t kHeader = 0xAA;
    constexpr size_t kFeedbackPacketLen = 16;
    constexpr size_t kCommandFrameLen = 8;
    constexpr uint32_t kStampMask = 0x7FF;       // stamp is 11 bits
    constexpr double kStampUnitSeconds = 256e-6; // one stamp count

    // XOR-fold checksum matching the firmware's checksum16(): byte i is XORed
    // into the low half of the accumulator if i is even, the high half if odd.
    uint16_t checksum16(const uint8_t * data, size_t len);

    // Builds a master->slave command frame: header, left, right, reserved,
    // checksum. Values go on the wire as-is; the firmware clamps them.
    struct CommandFrame
    {
        uint8_t bytes[kCommandFrameLen];
    };
    CommandFrame encodeCommandFrame(int16_t speed_left, int16_t speed_right);

    // Decoded slave->master feedback packet. A bitfield struct purely for
    // readable field names/widths -- populated field-by-field from the manual
    // bit-unpack in FeedbackParser::feed().
    struct FeedbackPacket
    {
        uint32_t seq : 7;
        uint32_t estop : 1;
        int32_t speed_l : 16;   // hoverboard rpm
        int32_t speed_r : 16;
        uint32_t en_tick_l : 14;
        uint32_t en_tick_r : 14;
        uint32_t temp_c : 7;
        uint32_t current_a : 11; // raw; divide by 100.0f for Amps
        uint32_t battery : 7;
        uint32_t stamp : 11;     // 256 us units, wraps (see kStampMask)
    };

    // Byte-fed parser for slave->master feedback packets (header 0xAA, fixed
    // length kFeedbackPacketLen). After a checksum failure it rescans the
    // dropped bytes for the next header so one corrupted byte cannot swallow
    // the following packet.
    class FeedbackParser
    {
    public:
        // Feed one received byte. Returns true exactly when a full,
        // checksum-valid packet has just been assembled; packet() is valid only
        // immediately after a call that returned true.
        bool feed(uint8_t byte);

        const FeedbackPacket & packet() const {return packet_;}

    private:
        uint8_t buf_[kFeedbackPacketLen]{};
        size_t len_{0};
        FeedbackPacket packet_{};
    };

}  // namespace protocol
}  // namespace uet_amr_hardware
