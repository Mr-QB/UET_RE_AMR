#include "serial_protocol.h"
 
SerialProtocol::SerialProtocol(): last_left_(0), last_right_(0), new_cmd_(false),  emergency_stop_(false), last_receive_time_(0){}
 
bool SerialProtocol::isNewCMD(int16_t* left, int16_t* right){
    processSerial();
    
    if (emergency_stop_) {
        emergency_stop_ = false;
        return false;
    }
    
    if (new_cmd_) {
        new_cmd_ = false;
        if (left) *left = last_left_;
        if (right) *right = last_right_;
    }
    
    return true;
}
 
void SerialProtocol::updateData(int16_t speedL, int16_t speedR, float x, float y, uint16_t theta, uint16_t battery, int16_t temp){
    //  packet:
    // 1. Velocity (cmd 0x00): [0xBB][0x00][vel_L][vel_R][checksum]
    // 2. Odometry (cmd 0x01): [0xBB][0x01][x][y][theta][checksum]
    // 3. Status (cmd 0x02): [0xBB][0x02][battery][temp][checksum]
    
    // velocity
    vel_packet[0] = HEADER_SLAVE;
    vel_packet[1] = 0x00;  // cmd velocity
    memcpy(vel_packet + 2, &speedL, 2);
    memcpy(vel_packet + 4, &speedR, 2);
    uint16_t vel_checksum = calcChecksum(vel_packet, 6);
    memcpy(vel_packet + 6, &vel_checksum, 2);
    
    // odometry
    odom_packet[0] = HEADER_SLAVE;
    odom_packet[1] = 0x01;  // cmd odometry
    memcpy(odom_packet + 2, &x, 4);
    memcpy(odom_packet + 6, &y, 4);
    memcpy(odom_packet + 10, &theta, 2);
    uint16_t odom_checksum = calcChecksum(odom_packet, 12);
    memcpy(odom_packet + 12, &odom_checksum, 2);
    
    // status
    status_packet[0] = HEADER_SLAVE;
    status_packet[1] = 0x02;  // cmd status
    memcpy(status_packet + 2, &battery, 2);
    memcpy(status_packet + 4, &temp, 2);
    uint16_t status_checksum = calcChecksum(status_packet, 6);
    memcpy(status_packet + 6, &status_checksum, 2);
}
 
bool SerialProtocol::isConnected() const{
    return (millis() - last_receive_time_) < TIMEOUT;
}
 
void SerialProtocol::processSerial(){
    // Read packet from computer: [0xAA][cmd][spd_L][spd_R][checksum]
    // 8 bytes total
    
    while (Serial.available() >= 8) {
        // find header 0xAA
        uint8_t header = Serial.read();
        if (header != HEADER_MASTER) {
            continue;
        }
        
        uint8_t packet[8];
        packet[0] = header;
        Serial.readBytes(packet + 1, 7);
        
        uint8_t cmd = packet[1];
        int16_t left, right;
        memcpy(&left, packet + 2, 2);
        memcpy(&right, packet + 4, 2);
        
        //checksum:
        uint16_t received_checksum;
        memcpy(&received_checksum, packet + 6, 2);
        
        uint16_t calc_checksum = calcChecksum(packet, 6);
        
        if (calc_checksum == received_checksum) {
            last_receive_time_ = millis();
            
            if(cmd == 0x00){
                Serial.write(vel_packet, 8);
            }else if(cmd == 0x01){
                Serial.write(odom_packet, 14);
            }else if(cmd == 0x02){
                Serial.write(status_packet, 8);
            }else if (cmd == 0x03) {
                emergency_stop_ = true;
                Serial.write(vel_packet, 8);
                new_cmd_ = false;
            } else {
                last_left_ = left;
                last_right_ = right;
                new_cmd_ = true;
                emergency_stop_ = false;
            }
        }
        
        break;
    }
}
 
uint16_t SerialProtocol::calcChecksum(uint8_t* data, int len){
    uint16_t checksum = 0;
    for (int i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}