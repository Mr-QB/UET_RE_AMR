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

#include <cstdint>
#include <vector>

namespace uet_amr_hardware
{

// One reporting window plus the totals since start. All fields are plain numbers so the node can copy them
// straight into uet_amr_msgs/msg/SerialLinkStatus.
struct LinkSnapshot
{
    double window_s{0.0};

    uint64_t packets_total{0};
    uint32_t frames_lost_total{0};
    uint32_t checksum_errors_total{0};
    uint32_t bytes_discarded_total{0};
    uint32_t timeouts_total{0};
    uint32_t reconnects_total{0};

    uint32_t packets_window{0};
    uint32_t frames_lost_window{0};
    uint32_t checksum_errors_window{0};
    double rate_hz{0.0};              // packets per second over the window
    double loss_percent{0.0};         // lost / (lost + received) over the window
    double firmware_period_ms{0.0};   // mean packet spacing by the firmware's own clock (needs no loss in the pair)
    double delay_mean_ms{0.0};        // arrival delay above the best case of the window, mean
    double delay_max_ms{0.0};         // ... and worst
};

// Health of the ESP32 -> ROS feedback stream. Pure bookkeeping (no ROS, no clock of its own: the caller passes
// host time), so it can be unit-tested with synthetic streams.
//
// What it can and cannot tell:
//  - Lost frames come from gaps in the 7-bit sequence number; checksum errors and discarded bytes from the parser.
//  - The firmware stamps every packet with its own 256 us clock. Comparing how far apart packets arrive on the host
//    against how far apart the firmware sent them gives the delay *variation* (queuing in the USB/serial stack,
//    scheduling): each sample is measured against the best case of the window, so a constant offset between the
//    two clocks cancels. It is NOT the absolute latency of the link; that needs the firmware to echo something
//    (e.g. a command counter in the reserved bytes of the command frame).
//  - Packets that arrive together in one read() share a host timestamp, so only the newest packet of each chunk is
//    used for the delay.
class LinkMonitor
{
public:
    // A valid feedback packet was decoded (sequence number, 11-bit firmware stamp).
    void onPacket(uint32_t seq, uint32_t stamp);

    // The serial read() that produced the packets so far returned at host time now_s (seconds, any steady origin).
    void onChunk(double now_s);

    // Cumulative parser counters (see protocol::ParserStats).
    void onParserCounters(uint64_t checksum_errors, uint64_t bytes_discarded);

    // More than the feedback timeout passed without a packet.
    void onTimeout() {++timeouts_;}

    // The port was reopened or the stream restarted: forget sequence/stamp continuity (a gap is not a loss).
    void reset();

    // reset() plus counting it as a reconnect.
    void onReconnect()
    {
        ++reconnects_;
        reset();
    }

    // Snapshot of the window since the previous call (or since the first packet), then start a new window.
    LinkSnapshot takeWindow(double now_s);

private:
    bool have_prev_{false};
    uint32_t last_seq_{0};
    uint32_t last_stamp_{0};
    double fw_time_s_{0.0};          // firmware time since the first packet after reset()

    bool pending_{false};            // a packet arrived since the last onChunk()
    bool have_base_{false};
    double base_host_s_{0.0};

    uint64_t packets_total_{0};
    uint32_t frames_lost_total_{0};
    uint32_t timeouts_{0};
    uint32_t reconnects_{0};
    uint64_t crc_total_{0};
    uint64_t discarded_total_{0};

    // Window accumulators
    double window_start_s_{-1.0};
    uint32_t win_packets_{0};
    uint32_t win_lost_{0};
    uint64_t win_crc_base_{0};
    double fw_period_sum_s_{0.0};
    uint32_t fw_period_n_{0};
    std::vector<double> win_delay_s_;
};

}  // namespace uet_amr_hardware
