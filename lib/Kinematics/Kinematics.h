#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "Arduino.h"

typedef struct {
    float x;
    float y;
    float angle;
    float linear_speed;
    float angular_speed;
} odom_t;

/**
 * 1. 运动学正逆解（两个轮子的实时速度->当前实时的角速度和线速度 / 当前目标的角速度和线速度->两个轮子的目标速度）
 *
 */
class Kinematics
{
private:
    float wheel_distance = 0.0;    // 两个轮子之间的距离
    odom_t odom; // 用于存储里程计信息

public:
    Kinematics(/* args */);
    ~Kinematics() = default;

    void set_wheel_distance(float distance); // 设置轮子的间距
    // 运动学正解,将左右轮的速度转换成线速度和角速度
    void kinematics_forward(float left_speed,float right_speed,float* out_linear_speed,float* out_angular_speed);
    // 运动学逆解，将线速度和角速度转换成左右轮的速度
    void kinematics_inverse(float linear_speed,float angular_speed,float* out_left_speed,float* out_right_speed);
    void TransAngleInPI(float angle,float& out_angle);
    void update_odom(uint16_t dt_ms, float left_speed, float right_speed);
    odom_t& get_odom();
};

#endif // __KINEMATICS_H__