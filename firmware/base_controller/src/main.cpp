#include <Arduino.h>
#include <HardwareSerial.h>


#define HOVER_SERIAL_BAUD   115200      
#define SERIAL_BAUD         115200      
#define START_FRAME         0xABCD      
#define TIME_SEND           100         

#include "odom.h"

Odom odom(8.5f, 45.0f, 90, 9000);

HardwareSerial HoverSerial(1);

typedef struct __attribute__((packed)) { // do not touch here
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

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
SerialFeedback Feedback;
SerialFeedback NewFeedback;


uint8_t idx = 0;
byte incomingByte;
byte incomingBytePrev;

void setup() {
  Serial.begin(SERIAL_BAUD);
  HoverSerial.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, 16, 17);
  odom.reset(0, 0, 0);
  
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed){ // do not touch here
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = uSteer;
  Command.speed    = uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  HoverSerial.write((uint8_t *)&Command, sizeof(Command)); 
}

int pre = 0;
float x_odom = 0.0f;
float y_odom = 0.0f;
float theta  = 0.0f;


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
        uint16_t checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^ NewFeedback.wheelR_cnt ^ NewFeedback.wheelL_cnt ^  NewFeedback.batVoltage ^ NewFeedback.boardTemp);

        //Check Checksum
        if (checksum == NewFeedback.checksum) {
          memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

          // here for your logic

          odom.updateFromEncoder(Feedback.wheelL_cnt, Feedback.wheelR_cnt);
          x_odom = odom.getX();
          y_odom = odom.getY();
          theta = odom.getTheta();
          // if(pre != Feedback.wheelL_cnt){
          //   Serial.println(Feedback.wheelL_cnt);
          //   pre = Feedback.wheelL_cnt;
          // }

          // debug
          // Serial.print("1: "); Serial.print(Feedback.cmd1);
          // Serial.print(" | 2: "); Serial.print(Feedback.cmd2);
          // Serial.print(" | R_Spd: "); Serial.print(Feedback.speedR_meas);
          // Serial.print(" | L_Spd: "); Serial.print(Feedback.speedL_meas);
          // Serial.print(" | L_Odom: "); 
          
          // Serial.print(" | Bat: "); Serial.print(Feedback.batVoltage);
          // Serial.print(" | Temp: "); Serial.print(Feedback.boardTemp);
          // Serial.print(" | Led: "); Serial.println(Feedback.cmdLed);


        } else {
          Serial.println("Checksum err!");
        }
      }
    }
    incomingBytePrev = incomingByte;
  }
}

// ########################## LOOP ##########################
unsigned long iTimeSend = 0;

int control = 500;

void loop(void){ 
  unsigned long timeNow = millis();
  Receive();

  if (Serial.available() > 0) { // debug
    String str = Serial.readStringUntil('\n');
    str.trim();
    if (str.length() > 0) {
      control = str.toInt();

      if (control < 0) control = 0;
      if (control > 1000) control = 1000;
    }
  }

  if (timeNow - iTimeSend >= TIME_SEND) { // keep freq
    iTimeSend = timeNow;

    Send(0, control - 500);

    Serial.print(x_odom, 2);
    Serial.print(" ");
    Serial.print(y_odom, 2);
    Serial.print(" ");
    Serial.println(theta * 180 / PI, 2);

  }

}