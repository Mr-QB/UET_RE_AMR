#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H
 
#include <Arduino.h>

#define HEAD 0xAA


struct __attribute__((packed)) SlaveToMasterPacket16 {
    // Word 1 (Bits 0..31): Header, Mode, Alignment, Signed Speeds (-100..100)
    uint32_t header      : 8;  // Bits 0..7   (Sync byte 0xAA)
    uint32_t sys_status  : 2;  // Bits 8..9   (0=Idle, 1=Nav, 2=Manual, 3=Fault)
    uint32_t reserved1   : 6;  // Bits 10..15 (Spare alignment bits)
    int32_t  speed_l     : 8;  // Bits 16..23 (Clamped -100 to +100)
    int32_t  speed_r     : 8;  // Bits 24..31 (Clamped -100 to +100)

    // Word 2 (Bits 32..63): Wheel Encoders & Spare Bits
    uint32_t en_tick_l   : 14; // Bits 32..45 (0-9000 ticks)
    uint32_t en_tick_r   : 14; // Bits 46..59 (0-9000 ticks)
    uint32_t reserved2   : 4;  // Bits 60..63 (Spare alignment bits)

    // Word 3 (Bits 64..95): 7-bit Temp (0..127°C), 11-bit Current, Battery & Power
    uint32_t temp_c      : 7;  // Bits 64..70 (0-127°C direct integer)
    uint32_t current_a   : 11; // Bits 71..81 (0-2047 raw value, divide by 100 on read -> 0.00 to 20.47 A)
    uint32_t battery     : 7;  // Bits 82..88 (0-100%)
    uint32_t charging    : 1;  // Bit 89      (0=Discharging, 1=Charging)
    uint32_t optional_1  : 6;  // Bits 90..95 (Expansion payload 1)

    // Word 4 (Bits 96..127): Optional Expansion 2 & CRC Checksum
    uint32_t optional_2  : 16; // Bits 96..111 (Expansion payload 2)
    uint32_t crc         : 16; // Bits 112..127 (CRC-16 Checksum)
};                             // EXACTLY 16 Bytes (128 Bits)

SlaveToMasterPacket16 FB;

void R_Send(int8_t sys, int8_t spL, int8_t spR, int16_t enL, int16_t enR, int8_t temp, int16_t cur, int8_t bat, bool charge){
    FB.header = HEAD;
    FB.sys_status = sys;
    FB.speed_l = spL;
    FB.speed_r = spR;
    FB.en_tick_l = enL;
    FB.en_tick_r = enR;
    FB.temp_c = temp;
    FB.current_a = cur;
    FB.battery = bat * 100 / 255;
    FB.charging = charge;



    FB.crc = FB.header ^ FB.sys_status ^ FB.reserved1 ^ FB.speed_l ^ FB.speed_r ^
             FB.en_tick_l ^ FB.en_tick_r ^ FB.reserved2 ^
             FB.temp_c ^ FB.current_a ^ FB.battery ^ FB.charging ^ FB.optional_1 ^
             FB.optional_2;
    Serial.write((uint8_t *)&FB, sizeof(FB));
}

bool R_Receive(int16_t &left, int16_t &right){
    while (Serial.available()){
        uint8_t header = Serial.read();
        if(header == HEAD){
            uint8_t b0 = Serial.read();
            uint8_t b1 = Serial.read();
            left = (int16_t)((b1 << 8) | b0);

            b0 = Serial.read();
            b1 = Serial.read();
            right = (int16_t)((b1 << 8) | b0);

            uint8_t checksum = Serial.read();
            if(HEAD ^ left ^ right == checksum){
                left = constrain(left, -100, 100);
                right = constrain(right, -100, 100);
                return true;
            }
        }
    }
    
    return false;
}

#endif // SERIAL_PROTOCOL_H