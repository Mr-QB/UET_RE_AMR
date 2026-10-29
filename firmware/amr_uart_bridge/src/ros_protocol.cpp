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


/**
 * @file ros_protocol.cpp
 * @brief Implements ros_protocol.h.
 */
#include "ros_protocol.h"

static uint8_t FB[FB_LEN];
static uint8_t FbSeq = 0;

static uint16_t checksum16(const uint8_t *data, uint8_t len) {
    uint16_t c = 0;
    for (uint8_t i = 0; i < len; i++) c ^= (uint16_t)data[i] << ((i & 1) * 8);
    return c;
}

// Appends fields LSB first at a running bit position, so the calls below read
// like the field table in ros_protocol.h.
struct BitWriter {
    uint8_t *buf;
    uint8_t pos = 0;
    void put(uint32_t value, uint8_t width) {
        for (uint8_t i = 0; i < width; i++, pos++) {
            if ((value >> i) & 1) buf[pos >> 3] |= (uint8_t)(1u << (pos & 7));
        }
    }
};

static void packPacket(const FeedbackPacket &p, uint8_t out[FB_LEN]) {
    memset(out, 0, FB_LEN);
    BitWriter w{out};
    w.put(HEAD,                    8);
    w.put(p.seq,                   7);
    w.put(p.estop,                 1);
    w.put((uint16_t)p.speed_l,    16);
    w.put((uint16_t)p.speed_r,    16);
    w.put(p.en_tick_l,            14);
    w.put(p.en_tick_r,            14);
    w.put(p.temp_c,                7);
    w.put(p.current_a,            11);
    w.put(p.battery,               7);
    w.put(p.stamp,                11);
    w.put(checksum16(out, FB_LEN - 2), 16);   // bytes 0..13 are already filled
}

void R_Send(int16_t spL, int16_t spR, int16_t enL, int16_t enR, uint8_t temp, int16_t cur, uint8_t bat, bool estop, uint32_t stamp_us) {
    FeedbackPacket p{};
    p.seq       = FbSeq++;
    p.estop     = estop ? 1 : 0;
    p.speed_l   = spL;
    p.speed_r   = spR;
    p.en_tick_l = (uint32_t)enL;
    p.en_tick_r = (uint32_t)enR;
    p.temp_c    = temp;
    p.current_a = (uint32_t)cur;
    p.battery   = (uint32_t)(bat * 100 / 255);
    p.stamp     = (stamp_us >> 8) & 0x7FF;

    packPacket(p, FB);
    Serial.write(FB, FB_LEN);
}

bool R_Receive(int16_t &left, int16_t &right) {
    static uint8_t frame[CMD_LEN];
    static uint8_t idx = 0;
    bool updated = false;

    while (Serial.available() > 0) {
        const uint8_t b = Serial.read();
        if (idx == 0 && b != HEAD) continue;
        frame[idx++] = b;
        if (idx < CMD_LEN) continue;
        idx = 0;

        uint8_t checksum = 0;
        for (uint8_t i = 0; i < CMD_LEN - 1; i++) checksum ^= frame[i];
        if (checksum != frame[CMD_LEN - 1]) {
            // Bad frame: a real header may sit inside it, so rescan the tail.
            for (uint8_t i = 1; i < CMD_LEN; i++) {
                if (frame[i] == HEAD) {
                    idx = CMD_LEN - i;
                    memmove(frame, &frame[i], idx);
                    break;
                }
            }
            continue;
        }
        left  = constrain((int16_t)((uint16_t)frame[1] | ((uint16_t)frame[2] << 8)), -CMD_LIMIT, CMD_LIMIT);
        right = constrain((int16_t)((uint16_t)frame[3] | ((uint16_t)frame[4] << 8)), -CMD_LIMIT, CMD_LIMIT);
        updated = true;
    }
    return updated;
}
