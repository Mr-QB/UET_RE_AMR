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
 * @file main.cpp
 * @brief Entry point bridging ROS2 and the hoverboard over serial.
 * @author Phuc Nguyen
 */
#include <Arduino.h>
#include "hover_protocol.h"
#include "ros_protocol.h"

int16_t left, right;

void setup() {
    Serial.begin(921600);
    H_Init();
}

void loop(void) {
    if (H_Receive()) {
        R_Send(1, Feedback.speedL_meas, Feedback.speedR_meas, Feedback.wheelL_cnt, Feedback.wheelR_cnt, Feedback.boardTemp, Feedback.cur, Feedback.batVoltage, 0);
    }
    if(R_Receive(left, right)) H_Send(left, right);
}
