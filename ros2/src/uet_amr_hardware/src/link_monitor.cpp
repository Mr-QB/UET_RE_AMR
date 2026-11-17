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

#include "uet_amr_hardware/link_monitor.hpp"

#include <algorithm>

#include "uet_amr_hardware/protocol.hpp"

namespace uet_amr_hardware
{

void LinkMonitor::onPacket(uint32_t seq, uint32_t stamp)
{
    if (have_prev_) {
        const uint32_t missed = (seq - last_seq_ - 1u) & 0x7Fu;
        frames_lost_total_ += missed;
        win_lost_ += missed;
        // 11-bit stamp: the masked difference is wrap-safe while the gap is below ~524 ms.
        const double dt = static_cast<double>((stamp - last_stamp_) & protocol::kStampMask) *
            protocol::kStampUnitSeconds;
        fw_time_s_ += dt;
        if (missed == 0) {
            fw_period_sum_s_ += dt;
            ++fw_period_n_;
        }
    } else {
        fw_time_s_ = 0.0;
    }
    last_seq_ = seq;
    last_stamp_ = stamp;
    have_prev_ = true;
    ++packets_total_;
    ++win_packets_;
    pending_ = true;
}

void LinkMonitor::onChunk(double now_s)
{
    if (window_start_s_ < 0.0) {
        window_start_s_ = now_s;
    }
    if (!pending_) {
        return;
    }
    pending_ = false;
    if (!have_base_) {
        base_host_s_ = now_s;
        have_base_ = true;
    }
    // Host time elapsed minus firmware time elapsed: constant for a perfect link, grows with queuing delay.
    win_delay_s_.push_back((now_s - base_host_s_) - fw_time_s_);
}

void LinkMonitor::onParserCounters(uint64_t checksum_errors, uint64_t bytes_discarded)
{
    crc_total_ = checksum_errors;
    discarded_total_ = bytes_discarded;
}

void LinkMonitor::reset()
{
    have_prev_ = false;
    have_base_ = false;
    pending_ = false;
}

LinkSnapshot LinkMonitor::takeWindow(double now_s)
{
    LinkSnapshot s;
    s.packets_total = packets_total_;
    s.frames_lost_total = frames_lost_total_;
    s.checksum_errors_total = static_cast<uint32_t>(crc_total_);
    s.bytes_discarded_total = static_cast<uint32_t>(discarded_total_);
    s.timeouts_total = timeouts_;
    s.reconnects_total = reconnects_;

    s.window_s = window_start_s_ < 0.0 ? 0.0 : now_s - window_start_s_;
    s.packets_window = win_packets_;
    s.frames_lost_window = win_lost_;
    s.checksum_errors_window = static_cast<uint32_t>(crc_total_ - win_crc_base_);
    if (s.window_s > 0.0) {
        s.rate_hz = win_packets_ / s.window_s;
    }
    if (win_packets_ + win_lost_ > 0) {
        s.loss_percent = 100.0 * win_lost_ / static_cast<double>(win_packets_ + win_lost_);
    }
    if (fw_period_n_ > 0) {
        s.firmware_period_ms = 1000.0 * fw_period_sum_s_ / fw_period_n_;
    }
    if (win_delay_s_.size() >= 2) {
        const double best = *std::min_element(win_delay_s_.begin(), win_delay_s_.end());
        double sum = 0.0, worst = 0.0;
        for (double d : win_delay_s_) {
            sum += d - best;
            worst = std::max(worst, d - best);
        }
        s.delay_mean_ms = 1000.0 * sum / win_delay_s_.size();
        s.delay_max_ms = 1000.0 * worst;
    }

    window_start_s_ = now_s;
    win_packets_ = 0;
    win_lost_ = 0;
    win_crc_base_ = crc_total_;
    fw_period_sum_s_ = 0.0;
    fw_period_n_ = 0;
    win_delay_s_.clear();
    return s;
}

}  // namespace uet_amr_hardware
