// Copyright (c) 2026 UET Robotics Club, University of Engineering and
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
 * @author Phuc Nguyen
 */
#include "ros_protocol.h"

static uint8_t FB[FB_LEN];

static uint16_t checksum16(const uint8_t *data, uint8_t len) {
    uint16_t c = 0;
    for (uint8_t i = 0; i < len; i++) c ^= (uint16_t)data[i] << ((i & 1) * 8);
    return c;
}

static void packPacket(const FeedbackPacket &p, uint8_t out[FB_LEN]) {
    out[0]  = HEAD;
    out[1]  = (uint8_t)(p.sys_status & 0x3);
    out[2]  = (uint8_t)p.speed_l;
    out[3]  = (uint8_t)p.speed_r;

    out[4]  = (uint8_t)(p.en_tick_l & 0xFF);
    out[5]  = (uint8_t)(((p.en_tick_l >> 8) & 0x3F) | ((p.en_tick_r & 0x3) << 6));
    out[6]  = (uint8_t)((p.en_tick_r >> 2) & 0xFF);
    out[7]  = (uint8_t)((p.en_tick_r >> 10) & 0xF);

    out[8]  = (uint8_t)((p.temp_c & 0x7F) | ((p.current_a & 0x1) << 7));
    out[9]  = (uint8_t)((p.current_a >> 1) & 0xFF);
    out[10] = (uint8_t)(((p.current_a >> 9) & 0x3) | ((p.battery & 0x3F) << 2));
    out[11] = (uint8_t)(((p.battery >> 6) & 0x1) | ((p.charging & 0x1) << 1) | ((p.optional_1 & 0x3F) << 2));

    out[12] = (uint8_t)(p.optional_2 & 0xFF);
    out[13] = (uint8_t)((p.optional_2 >> 8) & 0xFF);
    const uint16_t crc = checksum16(out, FB_LEN - 2);
    out[14] = (uint8_t)(crc & 0xFF);
    out[15] = (uint8_t)((crc >> 8) & 0xFF);
}

void R_Send(int8_t sys, int8_t spL, int8_t spR, int16_t enL, int16_t enR, uint8_t temp, int16_t cur, uint8_t bat, bool charge) {
    FeedbackPacket p{};
    p.sys_status = (uint32_t)sys;
    p.speed_l    = spL;
    p.speed_r    = spR;
    p.en_tick_l  = (uint32_t)enL;
    p.en_tick_r  = (uint32_t)enR;
    p.temp_c     = temp;
    p.current_a  = (uint32_t)cur;
    p.battery    = (uint32_t)(bat * 100 / 255);
    p.charging   = charge ? 1 : 0;
    p.optional_1 = 0;
    p.optional_2 = 0;

    packPacket(p, FB);
    Serial.write(FB, FB_LEN);
}

static CommandPacket unpackCommand(const uint8_t frame[CMD_LEN]) {
    CommandPacket p{};
    p.speed_l  = (int8_t)frame[1];
    p.speed_r  = (int8_t)frame[2];
    p.reserved = (uint32_t)frame[3] | ((uint32_t)frame[4] << 8) |
                 ((uint32_t)frame[5] << 16) | ((uint32_t)frame[6] << 24);
    return p;
}

bool R_Receive(int16_t &left, int16_t &right) {
    while (Serial.available() >= CMD_LEN){      // Read only when the required number of bytes is available.
        uint8_t header = Serial.read();
        if (header != HEAD) continue;

        uint8_t frame[CMD_LEN];
        frame[0] = header;
        for (uint8_t i = 1; i < CMD_LEN; i++) frame[i] = Serial.read();

        uint8_t checksum = 0;
        for (uint8_t i = 0; i < CMD_LEN - 1; i++) checksum ^= frame[i];
        if (checksum == frame[CMD_LEN - 1]) {
            const CommandPacket p = unpackCommand(frame);
            left  = constrain((int16_t)p.speed_l, -100, 100);
            right = constrain((int16_t)p.speed_r, -100, 100);
            return true;
        }
    }

    return false;
}
