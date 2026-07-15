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
    float velocity = 0; // Final velocity (combined from both methods)
    float velocity_inst = 0; // Instantaneous velocity (between 2 consecutive pulses) - fast response, noisier
    float velocity_avg = 0;  // Averaged velocity over a batch of steps - smoother, less noise, but lags
    float prev_velocity = 0;
    uint32_t lastTime = 0; // Last time the encoder was updated (per-instance, NOT a static local anymore)
    double lastPosition = 0; // Last position of the encoder
    uint32_t dt = 0; // Time difference between updates
    bool clockwise; // Direction of rotation
    int cw = 1; // Clockwise direction
 
    // Variables used by method 2 (averaging over a batch of steps).
    // IMPORTANT: these must be MEMBER variables (not static locals inside the function),
    // otherwise the left/right encoder objects would share a single counter -> wrong results.
    uint16_t stepAccum = 0;      // total encoder steps accumulated in the current batch
    uint32_t stepStartTime = 0;  // time when the current step batch started
    bool avgReady = false;       // whether the averaged velocity has been computed at least once
 
public:
    void Init(uint8_t A, uint8_t B, uint8_t C, uint16_t stepsPerRotation, float radius, bool clockwise);
    void readEncoder(int32_t *position, double *velocity);
    void reset();
};
 
int position_detect(byte curr, byte* prev);