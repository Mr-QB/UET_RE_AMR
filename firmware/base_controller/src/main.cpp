#include <Arduino.h>

#define TIME_SEND 12500 // 80Hz

#include "hoverboardSerial.h"
       
// Control types - choose one, if use wifi, plz choose type in wificontrol.h
// #define CMD_HAND     // Use Serial monitor to control (Quick check)
// #define CMD_COMPUTE  // Use compute/ROS2 to communicate 
#define CMD_WIFI


 

#ifdef CMD_WIFI
  #include "wificontrol.h"
#endif

#ifdef CMD_COMPUTE
  #include "serial_protocol.h"
  SerialProtocol SP; 
#endif

#ifdef CMD_HAND 
  int control = 500;
#endif

#ifdef CMD_COMPUTE 
  int16_t left, right;
  bool iscmd = true;
#endif



void setup() {
  Serial.begin(115200);
  
  #ifdef CMD_WIFI
    initWifi();
  #endif

  HoverSerial.begin(115200, SERIAL_8N1, 16, 17);
  odom.reset(0, 0, 0);
}

unsigned long long iTimeSend = 0;

void loop(void){ 
  unsigned long long timeNow = micros();
  Receive();

  #ifdef CMD_COMPUTE 
    iscmd = SP.isNewCMD(&left, &right);
    SP.updateData(Feedback.speedL_meas, Feedback.speedR_meas, x_odom, y_odom, theta * 10, Feedback.batVoltage, Feedback.boardTemp);
  #endif

  #ifdef CMD_HAND
    if (Serial.available() > 0) { // debug
      String str = Serial.readStringUntil('\n');
      str.trim();
      if (str.length() > 0) {
        control = str.toInt();

        if (control < 0) control = 0;
        if (control > 1000) control = 1000;
      }
    }
  #endif
    
  #if defined(WEB) && defined(CMD_WIFI)
    runWifi();
    Send(leftSpeed, rightSpeed);
  #endif

  if (timeNow - iTimeSend >= TIME_SEND) {  // main loop on const frequency
    iTimeSend = timeNow;

    #if defined(ESPNOW) && defined(CMD_WIFI)
      tx.x = x_odom;
      tx.y = y_odom;
      tx.theta = (int16_t)(10 * theta * 180 / PI);
      Send(rx.left, rx.right);
      esp_now_send(peerMAC, (uint8_t *) &tx, sizeof(tx));
    #endif

    #ifdef CMD_HAND
      Send(control - 500, 0);
    #endif

    #ifdef CMD_COMPUTE 
      if (SP.isConnected()) {
        if(iscmd){
          Send(left, right);
        }else{
          Send(0, 0);
        }
      }else{
        Send(0, 0);
      }
    #endif
  }
}