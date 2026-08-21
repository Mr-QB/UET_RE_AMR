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
 * @file ros_protocol.h
 * @brief Wire protocol for ROS2 <-> ESP32 communication (feedback/command packet packing).
 * @author Phuc Nguyen
 */
#ifndef ROS_PROTOCOL_H
#define ROS_PROTOCOL_H

#include <Arduino.h>

#define HEAD 0xAA

// =============================================================================
// Slave (ESP32) -> Master (ROS2) feedback packet, 16 bytes / 128 bits,
// bit-packed. FeedbackPacket below is a bitfield struct used only as
// the *local* in-code representation (readable field names/widths) -- it is
// never memcpy'd onto the wire, because C++ bitfield/struct memory layout
// (bit order, padding) is implementation-defined and differs across
// compilers/architectures. packPacket() in ros_protocol.cpp packs each
// field onto the wire by hand with explicit shifts, so the wire bytes are
// fixed regardless of compiler. Must stay in sync with the C++ port in
// ros2/src/uet_amr_hardware/include/uet_amr_hardware/protocol.hpp.
//
//   Word 1 (bytes 0..3,   bits 0..31):
//     header(8) sys_status(2) reserved1(6) speed_l(8) speed_r(8)
//   Word 2 (bytes 4..7,   bits 32..63):
//     en_tick_l(14) en_tick_r(14) reserved2(4)
//   Word 3 (bytes 8..11,  bits 64..95):
//     temp_c(7) current_a(11) battery(7) charging(1) optional_1(6)
//   Word 4 (bytes 12..15, bits 96..127):
//     optional_2(16) checksum(16)
//
//   Each word is written little-endian (LSB first). checksum is a 16-bit
//   XOR fold (see checksum16() in ros_protocol.cpp) over payload bytes
//   [0..13].
//
//   sys_status:  0=Idle, 1=Nav, 2=Manual, 3=Fault
//   speed_l/r:   measured speed, signed, -100..100
//   en_tick_l/r: wheel encoder ticks, 0..encoder_max (wraps)
//   temp_c:      board temperature, 0..127 degC
//   current_a:   raw; divide by 100 on read -> 0.00..20.47 A
//   battery:     0..100 %
//   charging:    0=discharging, 1=charging
//   optional_1/2: expansion payload, currently unused (0)
// =============================================================================
#define FB_LEN 16

struct __attribute__((packed)) FeedbackPacket {
    uint32_t sys_status : 2;
    int32_t  speed_l    : 8;
    int32_t  speed_r    : 8;
    uint32_t en_tick_l  : 14;
    uint32_t en_tick_r  : 14;
    uint32_t temp_c     : 7;
    uint32_t current_a  : 11;
    uint32_t battery    : 7;
    uint32_t charging   : 1;
    uint32_t optional_1 : 6;
    uint32_t optional_2 : 16;
};

void R_Send(int8_t sys, int8_t spL, int8_t spR, int16_t enL, int16_t enR, uint8_t temp, int16_t cur, uint8_t bat, bool charge);

// =============================================================================
// Master (ROS2) -> Slave (ESP32) command frame, 8 bytes. CommandPacket below
// is a bitfield struct used only as the *local* in-code representation
// (readable field names/widths) -- it is never memcpy'd onto the wire, same
// rationale as FeedbackPacket above. Must stay in sync with the C++ port in
// ros2/src/uet_amr_hardware/include/uet_amr_hardware/protocol.hpp.
//
//   speed_l/r are clamped to -100..100, so they're packed as 8-bit fields
//   (matching how FeedbackPacket encodes measured speed) instead of a full
//   int16 -- the bits saved go to `reserved` for future expansion (e.g.
//   enable/estop/mode flags) without growing the frame again.
//
//   [0]    header   uint8  0xAA
//   [1]    left     int8   commanded left speed (clamped to -100..100)
//   [2]    right    int8   commanded right speed (clamped to -100..100)
//   [3..6] reserved uint32 LE, expansion payload, currently unused (0)
//   [7]    checksum uint8  XOR of bytes [0..6]
// =============================================================================
#define CMD_LEN 8

struct __attribute__((packed)) CommandPacket {
    int32_t  speed_l  : 8;
    int32_t  speed_r  : 8;
    uint32_t reserved : 32;
};

bool R_Receive(int16_t &left, int16_t &right);

#endif // ROS_PROTOCOL_H
