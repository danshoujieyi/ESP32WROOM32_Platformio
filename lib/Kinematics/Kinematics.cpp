#include "Kinematics.h"

Kinematics::Kinematics(){
    odom.x = 0;
    odom.y = 0;
    odom.angle = 0;
    odom.linear_speed = 0;
    odom.angular_speed = 0;
}

// 设置轮子的间距
void Kinematics::set_wheel_distance(float distance)
{
    wheel_distance = distance;
}

// 运动学正解,将左右轮的速度转换成线速度和角速度
void Kinematics::kinematics_forward(float left_speed, float right_speed, float *out_linear_speed, float *out_angular_speed)
{
    *out_linear_speed = (left_speed + right_speed) / 2;
    *out_angular_speed = (right_speed - left_speed) / wheel_distance;   
}


// 运动学逆解，将线速度和角速度转换成左右轮的速度
void Kinematics::kinematics_inverse(float linear_speed, float angular_speed, float *out_left_speed, float *out_right_speed)
{
    *out_left_speed = linear_speed - angular_speed * wheel_distance / 2;
    *out_right_speed = linear_speed + angular_speed * wheel_distance / 2;
}

void Kinematics::TransAngleInPI(float angle, float& out_angle)
{
    if(angle>PI)
    {
        out_angle -= 2*PI;
    }else if (angle<-PI)
    {
        out_angle += 2*PI;
    }
}

void Kinematics::update_odom(uint16_t dt_ms, float left_speed, float right_speed)
{
    float dt_s = float(dt_ms)/1000.0f; // ms -> s
    // 获取实时的角速度和线速度呢？我们拿左右轮实时的速度，进行运动学正解
    this->kinematics_forward(left_speed, right_speed, &odom.linear_speed, &odom.angular_speed);
    // 计算里程计信息
    odom.linear_speed = odom.linear_speed/1000.0f; // 转换成米每秒

    // 角度积分
    odom.angle += odom.angular_speed*dt_s; 
    TransAngleInPI(odom.angle, odom.angle);
    // 计算机器人行走的距离（沿自身前进方向的）
    float delta_distance = odom.linear_speed * dt_s;
    // 分解到X轴和Y轴
    odom.x += delta_distance * std::cos(odom.angle);
    odom.y += delta_distance * std::sin(odom.angle);
}

odom_t& Kinematics::get_odom()
{
    return odom;
}
