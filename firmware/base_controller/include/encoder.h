/**
 * @file encoder.h
 * @brief Quadrature encoder interface for wheel odometry
 */

#pragma once

#include <Arduino.h>

class Encoder {
private:
    uint8_t A, B, C; // Encoder pin numbers
    uint8_t currState; // Current state of the encoder
    uint8_t prevState; // Previous state of the encoder
    uint16_t stepsPerRotation; // Number of steps per rotation
    float radius = 0; // Radius of the wheel
    int position = 0; // Current position of the encoder
    float velocity = 0; // Current velocity of the encoder
    uint32_t lastTime = 0; // Last time the encoder was updated
    double lastPosition = 0; // Last position of the encoder
    uint32_t dt = 0; // Time difference between updates
    bool clockwise; // Direction of rotation
    int cw = 1; // Clockwise direction
public:
    void Init(uint8_t A, uint8_t B, uint8_t C, uint16_t stepsPerRotation, float radius, bool clockwise);
    void readEncoder(int32_t *position, double *velocity);
    void reset();
};

int position_detect(byte curr, byte* prev);