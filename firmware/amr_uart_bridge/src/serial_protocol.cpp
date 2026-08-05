#include "serial_protocol.h"

static uint8_t FB[FB_LEN];

static uint16_t checksum16(const uint8_t *data, uint8_t len) {
    uint16_t c = 0;
    for (uint8_t i = 0; i < len; i++) c ^= (uint16_t)data[i] << ((i & 1) * 8);
    return c;
}

static void packPacket(const SlaveToMasterPacket &p, uint8_t out[FB_LEN]) {
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
    SlaveToMasterPacket p{};
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

bool R_Receive(int16_t &left, int16_t &right) {
    while (Serial.available() >= 6){   //Read only when the required number of bytes is available.
        uint8_t header = Serial.read();
        if (header != HEAD) continue;

        uint8_t frame[6];
        frame[0] = header;
        frame[1] = Serial.read();
        frame[2] = Serial.read();
        frame[3] = Serial.read();
        frame[4] = Serial.read();
        frame[5] = Serial.read();

        uint8_t checksum = frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4];
        if (checksum == frame[5]) {
            left  = (int16_t)((frame[2] << 8) | frame[1]);
            right = (int16_t)((frame[4] << 8) | frame[3]);
            left  = constrain(left, -100, 100);
            right = constrain(right, -100, 100);
            return true;
        }
    }

    return false;
}