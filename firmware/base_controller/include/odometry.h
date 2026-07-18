#pragma once
 
#include <Arduino.h>
 
// Simple differential-drive odometry (dead reckoning).
// Feed it the cumulative encoder step counts of the left and right wheels
// every loop, and it integrates them into a relative pose (x, y, theta)
// starting from wherever the robot was when Init()/reset() was called.
//
// NOTE: this is *relative* position estimation only - it drifts over time
// due to wheel slip and encoder noise, like any dead-reckoning odometry.
class Odometry {
private:
    float wheelRadius;       // wheel radius (same unit as the Encoder class uses)
    float wheelBase;         // distance between the left and right wheel contact points
                              // (i.e. track width), same unit as wheelRadius
    uint16_t stepsPerRotation; // encoder steps per full wheel rotation
 
    double x = 0;       // current X position (same unit as wheelRadius)
    double y = 0;       // current Y position (same unit as wheelRadius)
    double theta = 0;   // current heading, in radians, normalized to [-PI, PI]
 
    int32_t lastLeftPos = 0;
    int32_t lastRightPos = 0;
    bool initialized = false; // true once we have a first reading to diff against
 
public:
    // wheelRadius_ and wheelBase_ must use the same unit (e.g. both in mm or both in m).
    void Init(float wheelRadius_, float wheelBase_, uint16_t stepsPerRotation_);
 
    // Call this once per control loop with the latest cumulative encoder positions
    // (the same leftPos/rightPos values produced by Encoder::readEncoder).
    void update(int32_t leftPos, int32_t rightPos);
 
    // Retrieve the current pose. theta is in radians, normalized to [-PI, PI].
    void getPose(double *x, double *y, double *theta);
 
    // Convenience getters if you only need one value at a time.
    double getX();
    double getY();
    double getTheta();       // radians
    double getThetaDegrees(); // degrees, for easier debugging/printing
 
    // Resets the pose back to (0, 0, 0) and re-syncs the last known encoder counts,
    // so the next update() starts fresh from the current wheel positions.
    void reset(int32_t leftPos = 0, int32_t rightPos = 0);
};