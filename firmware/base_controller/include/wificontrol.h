#ifndef WIFICONTROL
#define WIFICONTROL

    // choose one
    // if you have another esp32  and have switch to control, you can use this feature for low latency
    #define ESPNOW


    // uncomment to use, in your device, find wifi name : AGV
    // connect and access web address 192.168.4.1
    // however this method has high latency
    
    // #define WEB


    // DO NOT TOUCH!
    #ifdef ESPNOW
        #include <WiFi.h>
        #include <esp_now.h>
        uint8_t peerMAC[] = {0x30, 0xAE, 0xA4, 0x00, 0xE4, 0x5C};
        struct rx_packet{
            int left;
            int right;
        };
        struct tx_packet{
            int16_t x;
            int16_t y;
            int16_t theta;
        };
        tx_packet tx;
        rx_packet rx;

        void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
            memcpy(&rx, incomingData, sizeof(rx));
        }
        void initWifi(){
            WiFi.mode(WIFI_STA);
            esp_now_init();
            esp_now_register_recv_cb(OnDataRecv);
            esp_now_peer_info_t peerInfo = {};
            memcpy(peerInfo.peer_addr, peerMAC, 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;
            esp_now_add_peer(&peerInfo);
        }
    #endif

    #ifdef WEB
        #include <WiFi.h>
        #include <WebServer.h>
        const char* ssid     = "AGV";
        const char* password = "";
        WebServer server(80);
        #include "web.h"
        int leftSpeed  = 0;
        int rightSpeed = 0;
        void handleRoot() {
            server.send_P(20, "text/html", HTML_PAGE);
        }
        void handleSet() {
            if (server.hasArg("left"))  leftSpeed  = constrain(server.arg("left").toInt(),  -100, 100);
            if (server.hasArg("right")) rightSpeed = constrain(server.arg("right").toInt(), -100, 100);
            server.send(20, "text/plain", "OK");
        }
        void initWifi() {
            WiFi.mode(WIFI_AP);
            WiFi.softAP(ssid, password);

            Serial.print("AP IP: ");
            Serial.println(WiFi.softAPIP());

            server.on("/", handleRoot);
            server.on("/set", handleSet);
            server.begin();
        }
        void runWifi() {
            server.handleClient();
        }
    #endif


#endif