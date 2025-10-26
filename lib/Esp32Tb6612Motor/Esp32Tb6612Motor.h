#ifndef ESP32_TB6612_MOTOR_H
#define ESP32_TB6612_MOTOR_H

#include "Arduino.h"

#include <stdio.h>

#include "esp_system.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

class Esp32Tb6612Motor {
private:
    bool mMotorAttached[6] = {false};  // 电机是否已初始化
    uint8_t mIn1[6];                  // 方向控制引脚IN1
    uint8_t mIn2[6];                  // 方向控制引脚IN2
    mcpwm_unit_t mMcpwmUnit[6];       // 存储每个电机使用的MCPWM单元
    mcpwm_timer_t mMcpwmTimer[6];     // 存储每个电机使用的MCPWM定时器
    // 新增：每个电机的死区阈值（默认5.0f，可外部修改）
    float mMotorDeadBand[6] = {6.25f, 6.25f, 6.25f, 6.25f, 6.25f, 6.25f};

public:
    Esp32Tb6612Motor();  // 构造函数（无STBY参数）
    void attachMotor(uint8_t id, uint8_t in1, uint8_t in2, uint8_t pwmPin);  // 初始化电机
    void stopMotor(int8_t motorId);  // 停止电机
    void updateMotorSpeed(int8_t id, float pwmValue);  // 更新速度和方向
    // 新增：设置单个电机的死区阈值（关键）
    bool setMotorDeadBand(int8_t id, float deadBand);
};

#endif
