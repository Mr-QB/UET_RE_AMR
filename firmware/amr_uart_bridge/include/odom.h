// Copyright (c) 2026 UET Robotics Club, University of Engineering and
//                    Technology, Vietnam National University, Hanoi (VNU).
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

/**
 * @file odom.h
 * @brief Odometry class computing robot pose from wheel encoder ticks.
 * @author Phuc Nguyen
 */
#ifndef ODOM_H
#define ODOM_H

/// Enable or disable odometry calculations through firmware.
#ifndef ENABLE_ODOM
#define ENABLE_ODOM 0
#endif

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
