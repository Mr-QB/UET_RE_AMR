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
    int16_t lastSpeed = 0; // Last speed value reported to setSpeed()

    // --- Non-blocking direction change (replaces old delay(50)) ---
    bool changingDirection = false;
    unsigned long dirChangeStartTime = 0;
    const uint16_t dirChangeDelay = 50; // ms to wait for the direction relay/H-bridge to settle

    // --- Start-from-rest hysteresis / kickstart ---
    // Real motor behavior: e.g. speed=40 will not start the motor from rest, but speed=50 will;
    // once running, speed can be lowered back down to 40 and the motor keeps spinning.
    // To start at a low target speed, we briefly command startThreshold, then drop to the target.
    bool isRunning = false;      // whether the motor is currently spinning (not at rest)
    bool kickstarting = false;
    unsigned long kickstartStartTime = 0;
    int16_t kickstartTargetSpeed = 0;
    const uint8_t startThreshold = 50;      // minimum |speed| (0-200 units) needed to start from rest
    const uint16_t kickstartDuration = 100; // ms to hold startThreshold before dropping to target

    float smoothedSpeed = 0; // ramping accumulator used by smoothVelocity() (float, avoids int truncation)

    float Kp = 0.1; // Proportional gain for velocity control
    float Ki = 0.01; // Integral gain for velocity control
    float Kd = 0.01; // Derivative gain for velocity control
    float integral = 0; // Integral term for PID control
    float prevError = 0; // Previous error for PID control
    float prevDerivative = 0; // Previous derivative for PID control
    float prevMeasurement = 0; // Previous currentVelocity, used for derivative-on-measurement
    const float integralLimit = 100.0f; // anti-windup clamp on |integral| (in speed units)

    float dt = 0.002; // Time step for PID control (in seconds) - nominal fallback, actually measured each call
    unsigned long lastUpdateMicros = 0; // timestamp of the previous setVelocity()/smoothVelocity() call

    float computeDt(); // measures real elapsed time since the last call and updates dt

    float getBaseSpeed(float targetVelocity); // Feed-forward base speed per target velocity (to be filled in later)
public:
    void Init(uint8_t dirPin, uint8_t addr, uint8_t stopPin); // Initialize motor control with direction pin and I2C address
    void setSpeed(int speed); // Set motor speed (-200 to 200)
    void setVelocity(float velocity,  float currentVelocity); // Set motor velocity (alternative to setSpeed)
    void smoothVelocity(float targetVelocity, float currentVelocity, float acceleration); // Smoothly adjust motor velocity to the target value
    void start(); // Start the motor
    void stop(); // Stop the motor
    bool isBusy() const { return changingDirection || kickstarting; } // true while direction-change or kickstart is in progress (PID should hold, not accumulate)
};

void setDAC(uint8_t addr, uint16_t value); // Set DAC value for motor speed control