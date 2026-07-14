/**
 * @file motor_controller.h
 * @brief Differential drive motor controller with PID velocity control
 */

#pragma once

#include <Arduino.h>


class ControlMotor {
private:
    uint8_t dirPin; // Direction pin for motor direction control
    uint8_t addr; // I2C address for motor controller
    uint8_t stopPin; // Pin to stop the motor (if applicable)
    uint16_t offset_dac = 900;
    uint16_t deadband_dac = 60; // Deadband for motor speed control
    bool cw = true;
    int16_t lastSpeed = 0; // Last speed set for the motor

    float Kp = 1.0; // Proportional gain for velocity control
    float Ki = 0.1; // Integral gain for velocity control
    float Kd = 0.01; // Derivative gain for velocity control
    float integral = 0; // Integral term for PID control
    float prevError = 0; // Previous error for PID control
    float prevDerivative = 0; // Previous derivative for PID control

    float dt = 0.002; // Time step for PID control (in seconds)
public:
    void Init(uint8_t dirPin, uint8_t addr, uint8_t stopPin); // Initialize motor control with direction pin and I2C address
    void setSpeed(int speed); // Set motor speed (-200 to 200)
    void setVelocity(float velocity); // Set motor velocity (alternative to setSpeed)
    void smoothVelocity(float targetVelocity, float currentVelocity, float acceleration); // Smoothly adjust motor velocity to the target value
    void start(); // Start the motor
    void stop(); // Stop the motor
};

void setDAC(uint8_t addr, uint16_t value); // Set DAC value for motor speed control

