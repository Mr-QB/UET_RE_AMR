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
 * @file hover_protocol.h
 * @brief Serial protocol for communicating with the hoverboard (command/feedback structs, send/receive).
 * @author Phuc Nguyen
 */
#ifndef HOVER_PROTOCOL_H
#define HOVER_PROTOCOL_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "odom.h"

extern HardwareSerial HoverSerial;

#if ENABLE_ODOM
extern Odom odom;

extern float x_odom;
extern float y_odom;
extern float theta;
#endif

typedef struct __attribute__((packed)) { // do not touch here
    uint16_t start;
    int16_t  steer;
    int16_t  speed;
    uint16_t checksum;
} HoverCommand;

typedef struct __attribute__((packed)) { // do not touch here
    uint16_t start;
    int8_t   speedR_meas;
    int8_t   speedL_meas;
    int16_t  wheelR_cnt;
    int16_t  wheelL_cnt;
    uint8_t  batVoltage;
    uint8_t  boardTemp;
    int16_t  cur;
    uint16_t checksum;
} HoverFeedback;

extern HoverFeedback Feedback;

void H_Init();
void H_Send(int16_t L, int16_t R);
// Drains any bytes waiting from the hoverboard. Returns true exactly when a
// new checksum-valid feedback packet was decoded into Feedback/odom (the
// caller is then expected to forward it via R_Send).
bool H_Receive();

#endif // HOVER_PROTOCOL_H
