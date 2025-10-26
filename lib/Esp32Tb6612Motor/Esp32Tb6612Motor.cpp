#include "Esp32Tb6612Motor.h"
/*
MOTORID PCNTID(ID/3+ID)      IOA((ID%3)*2)     IOB(ID*2+1)    TIMER
0       MCPWM_UNIT_0          0                1              0
1       MCPWM_UNIT_0          2                3              1
2       MCPWM_UNIT_0          4                5              2

3       MCPWM_UNIT_1          0                1              0
4       MCPWM_UNIT_1          2                3              1
5       MCPWM_UNIT_1          4                5              2
*/
// 构造函数：无需STBY引脚参数
Esp32Tb6612Motor::Esp32Tb6612Motor() {
    // 初始化电机状态数组
    for (int i = 0; i < 6; i++) {
        mMotorAttached[i] = false;
    }
}

// 初始化电机：绑定IN1、IN2（方向引脚）和PWM（速度引脚）
void Esp32Tb6612Motor::attachMotor(uint8_t id, uint8_t in1, uint8_t in2, uint8_t pwmPin) {
    if (id >= 6){
        Serial.println("Motor ID out of range Only support 0~5 motor");
        return;  // 仅支持0~5号电机
    } 
    
    // 存储方向控制引脚（IN1/IN2），配置为普通GPIO输出
    mIn1[id] = in1;
    mIn2[id] = in2;
    pinMode(mIn1[id], OUTPUT);
    pinMode(mIn2[id], OUTPUT);
    
    mcpwm_unit_t mcpwm_num = MCPWM_UNIT_0;
    // 根据ID分配MCPWM定时器通道
    // MCPWM0A = 0, MCPWM0B = 1, MCPWM1A = 2, MCPWM1B = 3, MCPWM2A = 4, MCPWM2B = 5
    mcpwm_io_signals_t io_signal_a = mcpwm_io_signals_t((id % 3) * 2);
    mcpwm_io_signals_t io_signal_b = mcpwm_io_signals_t((id % 3) * 2 + 1);
    // ESP32WROOM32-EVB默认有两个MCPWM，分别为MCPWM_UNIT_0和MCPWM_UNIT_1
    // 每一个PWM控制单元有三个定时器，分别为MCPWM_TIMER_0、MCPWM_TIMER_1、MCPWM_TIMER_2
    // 每个定时器有两个通道，分别为MCPWM_OPR_A、MCPWM_OPR_B
    // 因此一共有12个PWM通道
    // 根据ID分配MCPWM资源，整数除法0、1、2全部分配MCPWM_UNIT_0控制器，3、4、5全部分配MCPWM_UNIT_1控制器
    if (id / 3 == 1) {
        mcpwm_num = MCPWM_UNIT_1;
    }
    // 根据ID分配定时器
    mcpwm_timer_t mcpwm_timer = MCPWM_TIMER_0;
    if (id % 3 == 1) {
        mcpwm_timer = MCPWM_TIMER_1;
    } else if (id % 3 == 2) {
        mcpwm_timer = MCPWM_TIMER_2;
    }
    
    // 记录当前电机使用的MCPWM资源（用于后续速度控制）
    mMcpwmUnit[id] = mcpwm_num;
    mMcpwmTimer[id] = mcpwm_timer;
    
    // 将PWM引脚映射到MCPWM功能（使用通道A）
    mcpwm_gpio_init(mcpwm_num, io_signal_a , pwmPin);
    
    // 初始化MCPWM配置（频率5000Hz，初始占空比0）
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 10000;  // TB6612推荐频率1~100kHz
    pwm_config.cmpr_a = 0;        // 初始占空比0（停止）
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(mcpwm_num, mcpwm_timer, &pwm_config);
    
    mMotorAttached[id] = true;
    stopMotor(id);  // 初始状态停止电机
}

// 停止电机（IN1/IN2均置低，PWM占空比0）
void Esp32Tb6612Motor::stopMotor(int8_t motorId) {
    if (motorId < 0 || motorId >= 6 || !mMotorAttached[motorId]) return;
    
    // 方向引脚均置低（TB6612的停止状态）
    digitalWrite(mIn1[motorId], LOW);
    digitalWrite(mIn2[motorId], LOW);
    
    // PWM占空比设为0
    mcpwm_set_duty(mMcpwmUnit[motorId], mMcpwmTimer[motorId], MCPWM_OPR_A, 0);
    mcpwm_set_duty(mMcpwmUnit[motorId], mMcpwmTimer[motorId], MCPWM_OPR_B, 0);
}

/**
 * @brief 设置单个电机的死区阈值
 * @param id 电机ID（0-5）
 * @param deadBand 死区阈值（0.0f~100.0f，低于此值的PWM不驱动电机）
 * @return 配置成功返回true，失败返回false（ID非法或阈值超范围）
 */
bool Esp32Tb6612Motor::setMotorDeadBand(int8_t id, float deadBand) {
    // 1. 检查电机ID合法性
    if (id < 0 || id >= 6) {
        Serial.println("Error: Motor ID out of range (0-5)");
        return false;
    }
    // 2. 检查死区阈值合法性（0~100，避免负数或超PWM范围）
    if (deadBand < 0.0f || deadBand > 100.0f) {
        Serial.println("Error: DeadBand must be between 0.0f and 100.0f");
        return false;
    }
    // 3. 配置死区阈值
    mMotorDeadBand[id] = deadBand;
    return true;
}


// 更新电机速度和方向
void Esp32Tb6612Motor::updateMotorSpeed(int8_t id, float pwmValue) {
    if (id < 0 || id >= 6 || !mMotorAttached[id]) return;

        // 限制PID输出范围（-100 ~ 100）
    if (pwmValue > 100.0f) pwmValue = 100.0f;
    if (pwmValue < -100.0f) pwmValue = -100.0f;

    float deadZone = mMotorDeadBand[id];
    float compensatedPwm = 0.0f;

    // === 死区补偿逻辑 ===
    if (pwmValue > 0) {
        // 正方向补偿：把[0~100]映射到[deadZone~100]
        compensatedPwm = deadZone + (100.0f - deadZone) * pwmValue / 100.0f;
    } else if (pwmValue < 0) {
        // 反方向补偿：把[-100~0]映射到[-100~-deadZone]
        compensatedPwm = -deadZone + (100.0f - deadZone) * pwmValue / 100.0f;
    } else {
        // 停止
        stopMotor(id);
        return;
    }

    float absPwm = fabs(compensatedPwm);

    // 根据PWM正负设置方向（IN1/IN2电平组合）
    if (compensatedPwm > 0) {
        digitalWrite(mIn1[id], HIGH);
        digitalWrite(mIn2[id], LOW);
    } else if (compensatedPwm < 0) {
        digitalWrite(mIn1[id], LOW);
        digitalWrite(mIn2[id], HIGH);
    } else {
        stopMotor(id);
        return;
    }
    // 设置PWM占空比模式，高电平有效
    mcpwm_set_duty_type(mMcpwmUnit[id], mMcpwmTimer[id], MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    // 设置PWM占空比（仅控制速度）
    mcpwm_set_duty(mMcpwmUnit[id], mMcpwmTimer[id], MCPWM_OPR_A, absPwm);

    
    // // 限制PWM范围（0~100，超出则截断）
    // float absPwm = abs(pwmValue);
    // // 若PWM绝对值低于当前电机的死区阈值，直接停止电机
    // if (absPwm < mMotorDeadBand[id]) {
    //     stopMotor(id);
    //     return;
    // }
    // if (absPwm > 100.0f) absPwm = 100.0f;
    
    // // 根据PWM正负设置方向（IN1/IN2电平组合）
    // if (pwmValue > 0) {
    //     // 正转：IN1=高，IN2=低
    //     digitalWrite(mIn1[id], HIGH);
    //     digitalWrite(mIn2[id], LOW);
    // } else if (pwmValue < 0) {
    //     // 反转：IN1=低，IN2=高
    //     digitalWrite(mIn1[id], LOW);
    //     digitalWrite(mIn2[id], HIGH);
    // } else {
    //     // 停止：IN1=低，IN2=低
    //     stopMotor(id);
    //     return;
    // }

    // // 设置PWM占空比（仅控制速度）
    // mcpwm_set_duty(mMcpwmUnit[id], mMcpwmTimer[id], MCPWM_OPR_A, absPwm);
    // // 设置PWM占空比模式，高电平有效
    // mcpwm_set_duty_type(mMcpwmUnit[id], mMcpwmTimer[id], MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}
