#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H
 
#include <Arduino.h>
 
/*
 * Serial Protocol - ESP32 ↔ Computer (UART0)
 * Master (Computer) → Slave (ESP32): 8 bytes
 *   [Header] [Cmd] [SpeedL] [SpeedR] [Checksum]
 *    0xAA     1B     2B      2B        2B
 *
 * Slave (ESP32) → Master (Computer): 8/14 bytes
 *   [Header] [Cmd] [Data...] [Checksum]
 *    0xBB     1B    varies    2B
 *
 * 3 hàm chính:
 *   - isNewCMD(&left, &right)      : lấy lệnh, true nếu không phải stop
 *   - updateData(...)              : gửi dữ liệu tới computer
 *   - isConnected()                : kiểm tra kết nối
 */
 
class SerialProtocol{
public:
    SerialProtocol();
    
    bool isNewCMD(int16_t* left, int16_t* right);
    
    void updateData(int16_t speedL, int16_t speedR, float x, float y, uint16_t theta, uint16_t battery, int16_t temp);
    

    bool isConnected() const;
 
private:
    int16_t last_left_;
    int16_t last_right_;
    bool new_cmd_;
    bool emergency_stop_;
    unsigned long last_receive_time_;
    
    static const unsigned long TIMEOUT = 1000;
    static const uint8_t HEADER_MASTER = 0xAA;
    static const uint8_t HEADER_SLAVE = 0xBB;

    uint8_t vel_packet[8];
    uint8_t odom_packet[14];
    uint8_t status_packet[8];
    
    void processSerial();
    uint16_t calcChecksum(uint8_t* data, int len);
};
 
#endif // SERIAL_PROTOCOL_H