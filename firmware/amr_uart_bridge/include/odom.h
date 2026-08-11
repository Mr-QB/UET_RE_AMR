/**
 * @file odom.h
 * @brief Odometry class computing robot pose from wheel encoder ticks.
 * @author Phuc Nguyen
 * @copyright Copyright (c) 2026 Department of Robotics, University of Engineering and Technology, Vietnam National University, Hanoi
 */

#ifndef ODOM_H
#define ODOM_H

class Odom
{
public:
    Odom(float wheel_radius, float wheel_base,
         int ticks_per_rev = 90, int encoder_max = 9000);
 
    void reset(float x = 0.0f, float y = 0.0f, float theta = 0.0f);
 
    void setPose(float x, float y, float theta);
 
    void updateFromEncoder(int left_encoder, int right_encoder);
 
    void updateFromVelocity(float left_rpm, float right_rpm, float dt);
 
    float getX() const;
    float getY() const;
    float getTheta() const; // radian
 
    float getLastLinearDisplacement() const;
    float getLastAngularDisplacement() const;
 
private:
    float wheel_radius_;
    float wheel_base_;
    int   ticks_per_rev_;
    int   encoder_max_;
    float dist_per_tick_; 
 
    float x_;
    float y_;
    float theta_;
 
    int  last_left_enc_;
    int  last_right_enc_;
    bool first_update_;
 
    float last_dc_;  
    float last_dtheta_;
 
    int wrapDelta(int current, int previous) const;
    float normalizeAngle(float angle) const;
    void integrate(float dl, float dr);
};
 
#endif // ODOM_H