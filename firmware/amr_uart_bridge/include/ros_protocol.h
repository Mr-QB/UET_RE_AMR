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
 * @file ros_protocol.h
 * @brief Wire protocol between the ESP32 bridge and ROS2. Must stay in sync with
 *        ros2/src/uet_amr_hardware/include/uet_amr_hardware/protocol.hpp.
 */
#ifndef ROS_PROTOCOL_H
#define ROS_PROTOCOL_H

#include <Arduino.h>

#define HEAD 0xAA

// =============================================================================
// Slave (ESP32) -> Master (ROS2) feedback packet, 16 bytes / 128 bits,
// bit-packed. FeedbackPacket below is a bitfield struct used only as the
// *local* in-code representation (readable field names/widths) -- it is
// never memcpy'd onto the wire, because C++ bitfield/struct memory layout
// (bit order, padding) is implementation-defined and differs across
// compilers/architectures. packPacket() in ros_protocol.cpp packs each
// field onto the wire by hand, so the wire bytes are fixed regardless of
// compiler. Must stay in sync with the C++ port in
// ros2/src/uet_amr_hardware/include/uet_amr_hardware/protocol.hpp.
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
// =============================================================================
#define FB_LEN 16

struct __attribute__((packed)) FeedbackPacket {
  uint32_t seq        : 7;
  uint32_t estop      : 1;
  int32_t  speed_l    : 16;
  int32_t  speed_r    : 16;
  uint32_t en_tick_l  : 14;
  uint32_t en_tick_r  : 14;
  uint32_t temp_c     : 7;
  uint32_t current_a  : 11;
  uint32_t battery    : 7;
  uint32_t stamp      : 11;
};

// bat is the raw hoverboard reading (0..255), sent as 0..100 %. stamp_us is
// the bridge micros() when the hoverboard frame arrived.
void R_Send(int16_t spL, int16_t spR, int16_t enL, int16_t enR, uint8_t temp, int16_t cur, uint8_t bat, bool estop, uint32_t stamp_us);

// =============================================================================
// Master (ROS2) -> Slave (ESP32) command frame, 8 bytes, byte-aligned.
//
//   [0]    header   uint8  0xAA
//   [1..2] left     int16  LE, commanded left speed (clamped to +-CMD_LIMIT)
//   [3..4] right    int16  LE, commanded right speed (clamped to +-CMD_LIMIT)
//   [5..6] reserved uint16 LE, expansion payload, currently unused (0)
//   [7]    checksum uint8  XOR of bytes [0..6]
// =============================================================================
#define CMD_LEN 8
#define CMD_LIMIT 1000  // hoverboard INPUT_MAX/INPUT_MIN

// Drains the serial buffer byte by byte. Returns true when a checksum-valid
// command frame completed; left/right are then updated.
bool R_Receive(int16_t &left, int16_t &right);

#endif // ROS_PROTOCOL_H
