#include <Arduino.h>
#include "hoverboardSerial.h"


//The ESP32 acts as a bridge between ROS 2 and the hoverboard via serial communication.

// ROS 2 <-> ESP32 <-> HoverBoard


int16_t left, right;

void setup() {
  Serial.begin(921600);
  HoverSerial.begin(921600, SERIAL_8N1, 16, 17);
  odom.reset(0, 0, PI/2);
}

void loop(void){ 
  H_Receive();
  if(R_Receive(left, right)) H_Send(left, right);
}
