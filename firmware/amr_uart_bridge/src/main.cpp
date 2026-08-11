/**
 * @file main.cpp
 * @brief Entry point bridging ROS2 and the hoverboard over serial.
 * @author Phuc Nguyen
 * @copyright Copyright (c) 2026 Department of Robotics, University of Engineering and Technology, Vietnam National University, Hanoi
 */

#include <Arduino.h>
#include "hover_protocol.h"
#include "ros_protocol.h"

int16_t left, right;

void setup() {
  Serial.begin(921600);
  H_Init();
}

void loop(void){
  if (H_Receive()) {
    R_Send(1, Feedback.speedL_meas, Feedback.speedR_meas, Feedback.wheelL_cnt, Feedback.wheelR_cnt, Feedback.boardTemp, Feedback.cur, Feedback.batVoltage, 0);
  }
  if(R_Receive(left, right)) H_Send(left, right);
}
