#include "odometry.h"
 
void Odometry::Init(float wheelRadius_, float wheelBase_, uint16_t stepsPerRotation_) {
    wheelRadius = wheelRadius_;
    wheelBase = wheelBase_;
    stepsPerRotation = stepsPerRotation_;
 
    x = 0;
    y = 0;
    theta = 0;
    lastLeftPos = 0;
    lastRightPos = 0;
    initialized = false;
}
 
void Odometry::reset(int32_t leftPos, int32_t rightPos) {
    x = 0;
    y = 0;
    theta = 0;
    lastLeftPos = leftPos;
    lastRightPos = rightPos;
    initialized = true;
}
 
void Odometry::update(int32_t leftPos, int32_t rightPos) {
    // First call after Init()/reset(): just sync the reference counts, nothing to integrate yet.
    if (!initialized) {
        lastLeftPos = leftPos;
        lastRightPos = rightPos;
        initialized = true;
        return;
    }
 
    int32_t deltaStepsLeft = leftPos - lastLeftPos;
    int32_t deltaStepsRight = rightPos - lastRightPos;
    lastLeftPos = leftPos;
    lastRightPos = rightPos;
 
    if (deltaStepsLeft == 0 && deltaStepsRight == 0) {
        return; // no movement, pose unchanged
    }
 
    // Convert encoder step deltas to linear distance traveled by each wheel.
    float distPerStep = 2.0f * PI * wheelRadius / stepsPerRotation;
    double dLeft = deltaStepsLeft * distPerStep;
    double dRight = deltaStepsRight * distPerStep;
 
    // Standard differential-drive dead-reckoning:
    double dCenter = (dLeft + dRight) / 2.0;   // distance traveled by the robot's center
    double dTheta = (dRight - dLeft) / wheelBase; // change in heading
 
    // Use the midpoint heading (theta + dTheta/2) for better accuracy over the arc
    // traveled during this update, instead of only using the heading before or after.
    double midTheta = theta + dTheta / 2.0;
    x += dCenter * cos(midTheta);
    y += dCenter * sin(midTheta);
    theta += dTheta;
 
    // Normalize theta to [-PI, PI] so it doesn't grow unbounded after many rotations.
    theta = atan2(sin(theta), cos(theta));
}
 
void Odometry::getPose(double *xOut, double *yOut, double *thetaOut) {
    *xOut = x;
    *yOut = y;
    *thetaOut = theta;
}
 
double Odometry::getX() {
    return x;
}
 
double Odometry::getY() {
    return y;
}
 
double Odometry::getTheta() {
    return theta;
}
 
double Odometry::getThetaDegrees() {
    return theta * 180.0 / PI;
}