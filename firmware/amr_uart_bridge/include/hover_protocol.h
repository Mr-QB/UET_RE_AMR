/**
 * @file hover_protocol.h
 * @brief Serial protocol for communicating with the hoverboard (command/feedback structs, send/receive).
 * @author Phuc Nguyen
 * @copyright Copyright (c) 2026 Department of Robotics, University of Engineering and Technology, Vietnam National University, Hanoi
 */

#ifndef HOVER_PROTOCOL_H
#define HOVER_PROTOCOL_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "odom.h"

extern HardwareSerial HoverSerial;
extern Odom odom;

extern float x_odom;
extern float y_odom;
extern float theta;

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
