#include <Arduino.h>
#include "hover_protocol.h"
#include "ros_protocol.h"

// The ESP32 acts as a bridge between ROS 2 and the hoverboard via serial communication.
// ROS 2 <-> ESP32 <-> HoverBoard

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
