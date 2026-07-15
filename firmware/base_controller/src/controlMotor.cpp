#include "controlMotor.h"
#include <Wire.h>

// This project uses MCP4725 DAC 12bit to control speed of motor.
// The ESC (Electronic Speed Controller) is controlled by analog voltage(1.1V to 4.2V converted to DAC value is 900 to 3400)
// In this project, motor is limited speed, so the DAC value is limited 900 to 1900.
// However, the 5V power supply for MCP4725 DAC is not stable, at 900 the voltage may be lower or higher than 1.1V. Therefore, you need to calibrate to know when motor runs with DAC value.
// You also need to calculate the dead band of the control value. For example, when the motor is off, setting the value to 10 does not make it run, but setting it to 15 starts the motor. After that, if you lower the value back to 10, the motor keeps running, and you have to decrease it to 5 for the motor to stop.

void ControlMotor::Init(uint8_t dirPin, uint8_t addr, uint8_t stopPin) {
    this->dirPin = dirPin;
    this->addr = addr;
    this->stopPin = stopPin;
    pinMode(dirPin, OUTPUT);
    digitalWrite(dirPin, HIGH); // set LOW to reverse, HIGH to forward, depending on your motor wiring
    if(stopPin != 255){  // Assuming 255 is used to indicate no stop pin
        pinMode(stopPin, OUTPUT);
        digitalWrite(stopPin, HIGH); // Assuming HIGH is the default state for stop pin
        delay(200);
    }
}

void ControlMotor::setSpeed(int speed) {
    int16_t dacValue = 0;
    if (speed > 200) {
        speed = 200;
    } else if (speed < -200) {
        speed = -200;
    }

    if (speed < 0) {
        if (cw) {
            if(lastSpeed == 0) {
                digitalWrite(dirPin, LOW);
                cw = false;
                delay(50); // wait for 50 milliseconds to allow the motor to change direction
            }else{
                speed = 0;
            } 
        }
        dacValue = map(-speed, 1, 200, offset_dac, offset_dac + 1000);

    }else if (speed > 0) {
        if (!cw) {         
            if(lastSpeed == 0) {
                digitalWrite(dirPin, HIGH);
                cw = true;
                delay(50); // wait for 50 milliseconds to allow the motor to change direction
            }else{
                speed = 0;
            } 
        }
        dacValue = map(speed, 1, 200, offset_dac, offset_dac + 1000);

    }
    
    if(abs(speed) == 0) {
        dacValue = offset_dac - deadband_dac; // Set to offset if within deadband
    }
    setDAC(addr, dacValue);
    lastSpeed = speed;
}

void ControlMotor::setVelocity(float velocity, float currentVelocity) {
    float error = velocity - currentVelocity;
    integral += error * dt;
    float derivative = (error - prevError) / dt;
    prevError = error;
    float output = Kp * error + Ki * integral + Kd * derivative;
    setSpeed((int)output);
}

void ControlMotor::smoothVelocity(float targetVelocity, float currentVelocity, float acceleration) {
    float step = acceleration * dt;
    const float tolerance = step * 1.0f;

    bool reached =
        (currentVelocity >= lastSpeed - tolerance) &&(currentVelocity <= lastSpeed + tolerance);

    if (reached){
        if (lastSpeed < targetVelocity){
            lastSpeed += step;
            if (lastSpeed > targetVelocity)
                lastSpeed = targetVelocity;
        }else if (lastSpeed > targetVelocity){
            lastSpeed -= step;
            if (lastSpeed < targetVelocity)
                lastSpeed = targetVelocity;
        }
    }

    setSpeed(lastSpeed);
}

void ControlMotor::start() {
    digitalWrite(stopPin, HIGH); // Assuming HIGH allows the motor to run
}

void ControlMotor::stop() {
    setSpeed(0);
    digitalWrite(stopPin, LOW); // Assuming LOW stops the motor
}

void setDAC(uint8_t addr, uint16_t value) {
    value &= 0x0FFF; 
    Wire.beginTransmission(addr);
    Wire.write(0x40);                 // Fast Mode Write
    Wire.write(value >> 4);           // 8 bit cao
    Wire.write((value & 0x0F) << 4);  // 4 bit thấp
    Wire.endTransmission();
}