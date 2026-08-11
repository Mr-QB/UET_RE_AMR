/**
 * @file hover_protocol.cpp
 * @brief Implements hover_protocol.h.
 * @author Phuc Nguyen
 * @copyright Copyright (c) 2026 Department of Robotics, University of Engineering and Technology, Vietnam National University, Hanoi
 */

#include "hover_protocol.h"

#define START_FRAME 0xABCD

HardwareSerial HoverSerial(1);
Odom odom(8.5f, 45.0f, 90, 9000);

float x_odom = 0.0f;
float y_odom = 0.0f;
float theta  = 0.0f;

static HoverCommand Command;
HoverFeedback Feedback;
static HoverFeedback NewFeedback;

static uint8_t idx = 0;
static byte incomingByte;
static byte incomingBytePrev;

void H_Init() {
  HoverSerial.begin(921600, SERIAL_8N1, 16, 17);
  odom.reset(0, 0, PI/2);
}

// ########################## SEND ##########################
void H_Send(int16_t L, int16_t R){ // do not touch here
  L = -L;
  R = -R;
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (R - L) / 2;
  Command.speed    = (L + R) / 2;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  HoverSerial.write((uint8_t *)&Command, sizeof(Command));
}

// ########################## RECEIVE ##########################
bool H_Receive(){
  bool updated = false;
  while (HoverSerial.available() >= 14) {  // Read only when the required number of bytes is available.
    incomingByte = HoverSerial.read();
    uint16_t bufStartFrame = ((uint16_t)incomingByte << 8) | incomingBytePrev;
    if (bufStartFrame == START_FRAME) {
      byte *p = (byte *)&NewFeedback;
      *p++ = incomingBytePrev;
      *p++ = incomingByte;
      idx = 2;
    }
    else if (idx >= 2 && idx < sizeof(HoverFeedback)) {
      byte *p = (byte *)&NewFeedback;
      p[idx] = incomingByte;
      idx++;
      if (idx == sizeof(HoverFeedback)) {
        idx = 0;

        // cal Checksum
        uint16_t checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^ NewFeedback.wheelR_cnt ^ NewFeedback.wheelL_cnt ^ NewFeedback.cur ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp);

        // Check Checksum
        if (checksum == NewFeedback.checksum) {
          memcpy(&Feedback, &NewFeedback, sizeof(HoverFeedback));
          Feedback.speedL_meas = -Feedback.speedL_meas;
          Feedback.speedR_meas = Feedback.speedR_meas;

          odom.updateFromEncoder(Feedback.wheelL_cnt, Feedback.wheelR_cnt);
          x_odom = odom.getX();
          y_odom = odom.getY();
          theta = odom.getTheta();

          updated = true;
        }
      }
    }
    incomingBytePrev = incomingByte;
  }
  return updated;
}
