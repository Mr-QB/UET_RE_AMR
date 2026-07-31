#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>
HardwareSerial HoverSerial(1);

#define START_FRAME 0xABCD  

#include "odom.h"
Odom odom(8.5f, 45.0f, 90, 9000);

float x_odom = 0.0f;
float y_odom = 0.0f;
float theta  = 0.0f;

typedef struct __attribute__((packed)) { // do not touch here
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;

typedef struct __attribute__((packed)) { // do not touch here
   uint16_t start;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t   wheelR_cnt;
   int16_t   wheelL_cnt;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t checksum;
} SerialFeedback;

SerialCommand Command;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

uint8_t idx = 0;
byte incomingByte;
byte incomingBytePrev;

// ########################## SEND ##########################
void Send(int16_t L, int16_t R){ // do not touch here
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (R - L) / 2;
  Command.speed    = (L + R) / 2;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  HoverSerial.write((uint8_t *)&Command, sizeof(Command)); 
}

// ########################## RECEIVE ##########################
void Receive(){
  while (HoverSerial.available()) {
    incomingByte = HoverSerial.read();
    uint16_t bufStartFrame = ((uint16_t)incomingByte << 8) | incomingBytePrev;
    if (bufStartFrame == START_FRAME) { 
      byte *p = (byte *)&NewFeedback;
      *p++ = incomingBytePrev;
      *p++ = incomingByte;
      idx = 2; 
    } 
    else if (idx >= 2 && idx < sizeof(SerialFeedback)) { 
      byte *p = (byte *)&NewFeedback;
      p[idx] = incomingByte;
      idx++;
      if (idx == sizeof(SerialFeedback)) {
        idx = 0;

        // cal Checksum
        uint16_t checksum = (uint16_t)(NewFeedback.start ^  NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^ NewFeedback.wheelR_cnt ^ NewFeedback.wheelL_cnt ^  NewFeedback.batVoltage ^ NewFeedback.boardTemp);

        //Check Checksum
        if (checksum == NewFeedback.checksum) {
          memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
          odom.updateFromEncoder(Feedback.wheelL_cnt, Feedback.wheelR_cnt);
          x_odom = odom.getX();
          y_odom = odom.getY();
          theta = odom.getTheta();
        }
      }
    }
    incomingBytePrev = incomingByte;
  }
}