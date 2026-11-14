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


// Golden vectors for the ESP32 <-> ROS wire protocol. The feedback packets were produced by compiling
// firmware/amr_uart_bridge/src/ros_protocol.cpp (R_Send) on a host with an Arduino shim, so a change on either
// side that breaks the layout shows up here. The firmware is not in the container's workspace mount, which is why
// the vectors are embedded rather than generated from it at test time. If you change the protocol, regenerate them.

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "uet_amr_hardware/protocol.hpp"

namespace P = uet_amr_hardware::protocol;

namespace
{

struct Golden
{
    uint8_t bytes[P::kFeedbackPacketLen];
    unsigned seq, estop;
    int speed_l, speed_r;
    unsigned tick_l, tick_r, temp_c, current_a, battery, stamp;
};

// Firmware inputs, in order: seq increments per packet (0, 1, 2, 3).
//   R_Send(0, 0, 0, 0, 0, 0, 0, false, 0)
//   R_Send(100, -100, 1, 8999, 40, 52, 255, false, 256000)
//   R_Send(-32768, 32767, 4500, 8999, 127, 2047, 128, true, 0x7FF00)
//   R_Send(1234, -4321, 8191, 8192, 55, 20, 230, false, 123456789)
const Golden kGolden[] = {
    {{0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA, 0x00},
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {{0xAA, 0x01, 0x64, 0x00, 0x9C, 0xFF, 0x01, 0xC0, 0xC9, 0x88, 0xA2, 0x01, 0x19, 0x7D, 0x21, 0xCA},
     1, 0, 100, -100, 1, 8999, 40, 52, 100, 1000},
    {{0xAA, 0x82, 0x00, 0x80, 0xFF, 0x7F, 0x94, 0xD1, 0xC9, 0xF8, 0xFF, 0xBF, 0xEC, 0xFF, 0x1B, 0x14},
     2, 1, -32768, 32767, 4500, 8999, 127, 2047, 50, 2047},
    {{0xAA, 0x03, 0xD2, 0x04, 0x1F, 0xEF, 0xFF, 0x1F, 0x00, 0x78, 0xA3, 0x80, 0xB6, 0x79, 0x8D, 0x76},
     3, 0, 1234, -4321, 8191, 8192, 55, 20, 90, 973},
};

// Returns the number of packets completed while feeding all bytes; the last one stays in parser.packet().
size_t feedAll(P::FeedbackParser & parser, const std::vector<uint8_t> & bytes)
{
    size_t n = 0;
    for (uint8_t b : bytes) {
        if (parser.feed(b)) {
            ++n;
        }
    }
    return n;
}

std::vector<uint8_t> toVec(const uint8_t * b, size_t n) {return std::vector<uint8_t>(b, b + n);}

}  // namespace

TEST(Protocol, FeedbackGoldenVectorsDecode)
{
    for (const Golden & g : kGolden) {
        P::FeedbackParser parser;
        ASSERT_EQ(feedAll(parser, toVec(g.bytes, P::kFeedbackPacketLen)), 1u) << "seq " << g.seq;
        const auto & p = parser.packet();
        EXPECT_EQ(p.seq, g.seq);
        EXPECT_EQ(p.estop, g.estop);
        EXPECT_EQ(static_cast<int>(p.speed_l), g.speed_l);
        EXPECT_EQ(static_cast<int>(p.speed_r), g.speed_r);
        EXPECT_EQ(p.en_tick_l, g.tick_l);
        EXPECT_EQ(p.en_tick_r, g.tick_r);
        EXPECT_EQ(p.temp_c, g.temp_c);
        EXPECT_EQ(p.current_a, g.current_a);
        EXPECT_EQ(p.battery, g.battery);
        EXPECT_EQ(p.stamp, g.stamp);
    }
}

TEST(Protocol, GoldenChecksumsMatchChecksum16)
{
    for (const Golden & g : kGolden) {
        const uint16_t expected = P::checksum16(g.bytes, P::kFeedbackPacketLen - 2);
        const uint16_t wire = static_cast<uint16_t>(
            g.bytes[P::kFeedbackPacketLen - 2] | (g.bytes[P::kFeedbackPacketLen - 1] << 8));
        EXPECT_EQ(wire, expected);
    }
}

TEST(Protocol, Checksum16AlternatesLowAndHighByte)
{
    const uint8_t data[] = {0x01, 0x02, 0x04};
    // byte0 ^ byte2 in the low half, byte1 in the high half
    EXPECT_EQ(P::checksum16(data, 3), 0x0205);
    EXPECT_EQ(P::checksum16(data, 0), 0);
}

TEST(Protocol, CorruptedChecksumIsRejected)
{
    auto bytes = toVec(kGolden[1].bytes, P::kFeedbackPacketLen);
    bytes[5] ^= 0x01;
    P::FeedbackParser parser;
    EXPECT_EQ(feedAll(parser, bytes), 0u);
}

TEST(Protocol, ParserSkipsGarbageBeforeHeader)
{
    std::vector<uint8_t> bytes = {0x00, 0x13, 0x37};
    const auto good = toVec(kGolden[2].bytes, P::kFeedbackPacketLen);
    bytes.insert(bytes.end(), good.begin(), good.end());
    P::FeedbackParser parser;
    ASSERT_EQ(feedAll(parser, bytes), 1u);
    EXPECT_EQ(parser.packet().seq, 2u);
}

TEST(Protocol, ParserResyncsWhenHeaderHidesInsideCorruptPacket)
{
    // A packet that lost its first bytes: the next real packet starts inside what the parser took for one packet.
    const auto a = toVec(kGolden[1].bytes, P::kFeedbackPacketLen);
    const auto b = toVec(kGolden[3].bytes, P::kFeedbackPacketLen);
    std::vector<uint8_t> bytes = {0xAA, 0x55, 0x55, 0x55};  // false header + junk
    bytes.insert(bytes.end(), a.begin(), a.end());
    bytes.insert(bytes.end(), b.begin(), b.end());
    P::FeedbackParser parser;
    const size_t n = feedAll(parser, bytes);
    EXPECT_GE(n, 1u);
    EXPECT_EQ(parser.packet().seq, 3u);  // the last packet always gets through
}

TEST(Protocol, BackToBackPacketsAreAllDelivered)
{
    std::vector<uint8_t> bytes;
    for (const Golden & g : kGolden) {
        bytes.insert(bytes.end(), g.bytes, g.bytes + P::kFeedbackPacketLen);
    }
    P::FeedbackParser parser;
    EXPECT_EQ(feedAll(parser, bytes), 4u);
}

TEST(Protocol, StampConstants)
{
    EXPECT_EQ(P::kStampMask, 0x7FFu);
    EXPECT_DOUBLE_EQ(P::kStampUnitSeconds * 2048.0, 0.524288);  // wraps about every 524 ms
}

struct CommandGolden
{
    int16_t left, right;
    uint8_t bytes[P::kCommandFrameLen];
};

// These frames were also fed to the firmware's R_Receive(): it accepted all of them and clamped to +-1000.
const CommandGolden kCommands[] = {
    {0, 0, {0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA}},
    {100, -100, {0xAA, 0x64, 0x00, 0x9C, 0xFF, 0x00, 0x00, 0xAD}},
    {1000, -1000, {0xAA, 0xE8, 0x03, 0x18, 0xFC, 0x00, 0x00, 0xA5}},
    {1500, -1500, {0xAA, 0xDC, 0x05, 0x24, 0xFA, 0x00, 0x00, 0xAD}},
    {-32768, 32767, {0xAA, 0x00, 0x80, 0xFF, 0x7F, 0x00, 0x00, 0xAA}},
};

TEST(Protocol, CommandFrameGoldenBytes)
{
    for (const CommandGolden & c : kCommands) {
        const P::CommandFrame f = P::encodeCommandFrame(c.left, c.right);
        for (size_t i = 0; i < P::kCommandFrameLen; ++i) {
            EXPECT_EQ(f.bytes[i], c.bytes[i]) << "cmd(" << c.left << "," << c.right << ") byte " << i;
        }
    }
}

TEST(Protocol, CommandFrameChecksumIsXorOfFirstSevenBytes)
{
    const P::CommandFrame f = P::encodeCommandFrame(321, -654);
    uint8_t x = 0;
    for (size_t i = 0; i + 1 < P::kCommandFrameLen; ++i) {
        x ^= f.bytes[i];
    }
    EXPECT_EQ(f.bytes[P::kCommandFrameLen - 1], x);
    EXPECT_EQ(f.bytes[0], P::kHeader);
}
