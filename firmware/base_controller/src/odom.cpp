#include "odom.h"
#include <math.h>
 
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
 
Odom::Odom(float wheel_radius, float wheel_base,
           int ticks_per_rev, int encoder_max)
    : wheel_radius_(wheel_radius),
      wheel_base_(wheel_base),
      ticks_per_rev_(ticks_per_rev),
      encoder_max_(encoder_max),
      x_(0.0f), y_(0.0f), theta_(0.0f),
      last_left_enc_(0), last_right_enc_(0),
      first_update_(true),
      last_dc_(0.0f), last_dtheta_(0.0f)
{
    dist_per_tick_ = (2.0f * M_PI * wheel_radius_) / (float)ticks_per_rev_;
}
 
void Odom::reset(float x, float y, float theta)
{
    x_ = x;
    y_ = y;
    theta_ = normalizeAngle(theta);
    last_left_enc_ = 0;
    last_right_enc_ = 0;
    first_update_ = true;
    last_dc_ = 0.0f;
    last_dtheta_ = 0.0f;
}
 
void Odom::setPose(float x, float y, float theta)
{
    x_ = x;
    y_ = y;
    theta_ = normalizeAngle(theta);
}
 
int Odom::wrapDelta(int current, int previous) const
{
    int delta = current - previous;
    int half = encoder_max_ / 2;
 
    if (delta > half)
    {
        delta -= encoder_max_;
    }
    else if (delta < -half)
    {
        delta += encoder_max_;
    }
    return delta;
}
 
float Odom::normalizeAngle(float angle) const
{
    while (angle > M_PI)  angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}
 
void Odom::integrate(float dl, float dr)
{
    float dc = (dl + dr) / 2.0f;             
    float dtheta = (dr - dl) / wheel_base_;  

    float theta_mid = theta_ + dtheta / 2.0f;
 
    x_ += dc * cosf(theta_mid);
    y_ += dc * sinf(theta_mid);
    theta_ = normalizeAngle(theta_ + dtheta);
 
    last_dc_ = dc;
    last_dtheta_ = dtheta;
}
 
void Odom::updateFromEncoder(int left_encoder, int right_encoder)
{
    if (first_update_)
    {
        last_left_enc_ = left_encoder;
        last_right_enc_ = right_encoder;
        first_update_ = false;
        last_dc_ = 0.0f;
        last_dtheta_ = 0.0f;
        return;
    }
 
    int delta_left_ticks  = wrapDelta(left_encoder, last_left_enc_);
    int delta_right_ticks = wrapDelta(right_encoder, last_right_enc_);
 
    last_left_enc_  = left_encoder;
    last_right_enc_ = right_encoder;
 
    float dl = delta_left_ticks  * dist_per_tick_;
    float dr = delta_right_ticks * dist_per_tick_;
 
    integrate(dl, dr);
}
 
void Odom::updateFromVelocity(float left_rpm, float right_rpm, float dt)
{
    float v_left  = (left_rpm  / 60.0f) * (2.0f * M_PI * wheel_radius_);
    float v_right = (right_rpm / 60.0f) * (2.0f * M_PI * wheel_radius_);
 
    float dl = v_left  * dt;
    float dr = v_right * dt;
 
    integrate(dl, dr);
}
 
float Odom::getX() const
{
    return x_;
}
 
float Odom::getY() const
{
    return y_;
}
 
float Odom::getTheta() const
{
    return theta_;
}
 
float Odom::getLastLinearDisplacement() const
{
    return last_dc_;
}
 
float Odom::getLastAngularDisplacement() const
{
    return last_dtheta_;
}