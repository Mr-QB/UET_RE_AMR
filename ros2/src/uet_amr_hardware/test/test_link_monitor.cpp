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

#include <gtest/gtest.h>

#include "uet_amr_hardware/link_monitor.hpp"
#include "uet_amr_hardware/protocol.hpp"

using uet_amr_hardware::LinkMonitor;

namespace
{

constexpr unsigned kUnitsPerPacket = 80;                 // 80 x 256 us = 20.48 ms, about 49 Hz
constexpr double kPeriodS = kUnitsPerPacket * 256e-6;

// Feeds n packets, one read() chunk each, arriving exactly on the firmware's schedule plus extra_delay_s(i).
template <typename F>
void feed(LinkMonitor & m, int n, uint32_t & seq, uint32_t & stamp, double & host, F extra_delay_s)
{
    for (int i = 0; i < n; ++i) {
        seq = (seq + 1) & 0x7F;
        stamp = (stamp + kUnitsPerPacket) & 0x7FF;
        host += kPeriodS;
        m.onPacket(seq, stamp);
        m.onChunk(host + extra_delay_s(i));
    }
}

}  // namespace

TEST(LinkMonitor, CleanStreamHasNoLossAndNoDelay)
{
    LinkMonitor m;
    uint32_t seq = 0, stamp = 0;
    double host = 100.0;
    m.onPacket(seq, stamp);
    m.onChunk(host);
    feed(m, 200, seq, stamp, host, [](int) {return 0.0;});
    const auto s = m.takeWindow(host);
    EXPECT_EQ(s.packets_total, 201u);
    EXPECT_EQ(s.frames_lost_total, 0u);
    EXPECT_EQ(s.frames_lost_window, 0u);
    EXPECT_DOUBLE_EQ(s.loss_percent, 0.0);
    EXPECT_NEAR(s.firmware_period_ms, 20.48, 1e-6);
    EXPECT_NEAR(s.rate_hz, 1.0 / kPeriodS, 0.5);
    EXPECT_NEAR(s.delay_mean_ms, 0.0, 1e-6);
    EXPECT_NEAR(s.delay_max_ms, 0.0, 1e-6);
}

TEST(LinkMonitor, SequenceGapsCountAsLostFramesAcrossWrap)
{
    LinkMonitor m;
    m.onPacket(126, 0);
    m.onPacket(127, 80);
    m.onPacket(0, 160);      // wrap, nothing lost
    EXPECT_EQ(m.takeWindow(1.0).frames_lost_total, 0u);
    m.onPacket(3, 400);      // 1 and 2 missing
    m.onPacket(4, 480);
    const auto s = m.takeWindow(2.0);
    EXPECT_EQ(s.frames_lost_total, 2u);
    EXPECT_EQ(s.frames_lost_window, 2u);
    EXPECT_NEAR(s.loss_percent, 100.0 * 2 / 4, 1e-9);   // 2 received in the window + 2 lost
}

TEST(LinkMonitor, FirmwarePeriodSurvivesStampWrapAndSkipsGaps)
{
    LinkMonitor m;
    uint32_t seq = 0, stamp = 2040;      // wraps within the first packets
    double host = 0.0;
    m.onPacket(seq, stamp);
    m.onChunk(host);
    feed(m, 50, seq, stamp, host, [](int) {return 0.0;});
    // a lost frame doubles one spacing; it must not pollute the period mean
    seq = (seq + 2) & 0x7F;
    stamp = (stamp + 2 * kUnitsPerPacket) & 0x7FF;
    host += 2 * kPeriodS;
    m.onPacket(seq, stamp);
    m.onChunk(host);
    const auto s = m.takeWindow(host);
    EXPECT_EQ(s.frames_lost_total, 1u);
    EXPECT_NEAR(s.firmware_period_ms, 20.48, 1e-6);
}

TEST(LinkMonitor, DelayVariationIsMeasuredAgainstTheBestCaseOfTheWindow)
{
    LinkMonitor m;
    uint32_t seq = 0, stamp = 0;
    double host = 5.0;
    m.onPacket(seq, stamp);
    m.onChunk(host + 0.003);
    // every packet is 3 ms late (constant offset, must cancel) except packet 40, 18 ms late
    feed(m, 100, seq, stamp, host, [](int i) {return i == 40 ? 0.018 : 0.003;});
    const auto s = m.takeWindow(host);
    EXPECT_NEAR(s.delay_max_ms, 15.0, 1e-6);          // 18 - 3
    EXPECT_NEAR(s.delay_mean_ms, 0.15, 0.05);          // one 15 ms outlier among ~100 samples
}

TEST(LinkMonitor, PacketsInOneChunkUseOnlyTheNewestForDelay)
{
    LinkMonitor m;
    uint32_t seq = 0, stamp = 0;
    double host = 1.0;
    m.onPacket(seq, stamp);
    m.onChunk(host);
    // three packets sent 20.48 ms apart are read together at the arrival time of the newest one
    for (int i = 0; i < 3; ++i) {
        seq = (seq + 1) & 0x7F;
        stamp = (stamp + kUnitsPerPacket) & 0x7FF;
        m.onPacket(seq, stamp);
    }
    host += 3 * kPeriodS;
    m.onChunk(host);
    host += kPeriodS;
    m.onPacket((seq + 1) & 0x7F, (stamp + kUnitsPerPacket) & 0x7FF);
    m.onChunk(host);
    const auto s = m.takeWindow(host);
    EXPECT_EQ(s.packets_total, 5u);
    EXPECT_NEAR(s.delay_max_ms, 0.0, 1e-6);            // no phantom delay from the older packets in the chunk
}

TEST(LinkMonitor, ResetIsNotCountedAsLossButReconnectIsCounted)
{
    LinkMonitor m;
    m.onPacket(10, 0);
    m.onChunk(1.0);
    m.onReconnect();
    m.onPacket(90, 1000);     // unrelated sequence after the port was reopened
    m.onChunk(2.0);
    m.onTimeout();
    m.onTimeout();
    const auto s = m.takeWindow(2.0);
    EXPECT_EQ(s.frames_lost_total, 0u);
    EXPECT_EQ(s.reconnects_total, 1u);
    EXPECT_EQ(s.timeouts_total, 2u);
}

TEST(LinkMonitor, WindowsReportOnlyWhatIsNew)
{
    LinkMonitor m;
    m.onPacket(0, 0);
    m.onChunk(0.0);
    m.onParserCounters(3, 40);
    auto s1 = m.takeWindow(1.0);
    EXPECT_EQ(s1.checksum_errors_window, 3u);
    EXPECT_EQ(s1.bytes_discarded_total, 40u);
    m.onPacket(1, 80);
    m.onChunk(1.5);
    m.onParserCounters(4, 55);
    auto s2 = m.takeWindow(2.0);
    EXPECT_EQ(s2.checksum_errors_window, 1u);
    EXPECT_EQ(s2.checksum_errors_total, 4u);
    EXPECT_EQ(s2.packets_window, 1u);
    EXPECT_EQ(s2.packets_total, 2u);
    EXPECT_NEAR(s2.rate_hz, 1.0, 1e-9);
}
