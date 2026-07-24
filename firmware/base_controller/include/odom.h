#ifndef ODOM_H
#define ODOM_H
 
/*
 * Thu vien tinh Odometry (vi tri tuong doi) cho xe 2 banh vi sai
 * (Differential Drive Robot).
 *
 * Ho tro 2 cach cap nhat:
 *   1) updateFromEncoder(): dua tren gia tri encoder TUYET DOI
 *      (0 .. encoder_max-1), tu dong xu ly tran so (wrap-around).
 *   2) updateFromVelocity(): dua tren van toc banh xe (rpm) va
 *      thoi gian trich mau dt (giay).
 *
 * Don vi:
 *   - wheel_radius, wheel_base do nguoi dung tu chon don vi (m, cm, mm...)
 *   - x, y tra ve CUNG DON VI voi wheel_radius / wheel_base
 *   - theta tra ve don vi radian, chuan hoa trong khoang [-pi, pi]
 */
class Odom
{
public:
    // wheel_radius : ban kinh banh xe
    // wheel_base   : khoang cach giua 2 tam banh xe
    // ticks_per_rev: so buoc encoder tren 1 vong quay banh xe (mac dinh 90)
    // encoder_max  : so gia tri encoder (mac dinh 9000 -> encoder chay 0..8999)
    Odom(float wheel_radius, float wheel_base,
         int ticks_per_rev = 90, int encoder_max = 9000);
 
    // Dat lai vi tri (x, y, theta) va trang thai encoder ban dau
    void reset(float x = 0.0f, float y = 0.0f, float theta = 0.0f);
 
    // Ep lai pose hien tai (khong reset encoder truoc do)
    void setPose(float x, float y, float theta);
 
    // Cap nhat odometry tu gia tri encoder TUYET DOI hien tai
    // left_encoder, right_encoder: gia tri trong khoang [0, encoder_max-1]
    void updateFromEncoder(int left_encoder, int right_encoder);
 
    // Cap nhat odometry tu van toc banh xe (rpm) va thoi gian dt (giay)
    void updateFromVelocity(float left_rpm, float right_rpm, float dt);
 
    float getX() const;
    float getY() const;
    float getTheta() const; // radian
 
    // Khoang cach da di duoc cua tam xe trong lan cap nhat gan nhat
    float getLastLinearDisplacement() const;
    // Goc quay duoc trong lan cap nhat gan nhat (radian)
    float getLastAngularDisplacement() const;
 
private:
    float wheel_radius_;
    float wheel_base_;
    int   ticks_per_rev_;
    int   encoder_max_;
    float dist_per_tick_; // quang duong ung voi 1 buoc encoder
 
    float x_;
    float y_;
    float theta_;
 
    int  last_left_enc_;
    int  last_right_enc_;
    bool first_update_;
 
    float last_dc_;     // quang duong tam xe di duoc lan cap nhat gan nhat
    float last_dtheta_; // goc quay lan cap nhat gan nhat
 
    // Tinh do lech encoder co xu ly tran so (encoder chay vong 0..encoder_max-1)
    int wrapDelta(int current, int previous) const;
 
    // Chuan hoa goc ve khoang [-pi, pi]
    float normalizeAngle(float angle) const;
 
    // Cap nhat pose (x, y, theta) tu quang duong 2 banh di duoc (dl, dr)
    void integrate(float dl, float dr);
};
 
#endif // ODOM_H