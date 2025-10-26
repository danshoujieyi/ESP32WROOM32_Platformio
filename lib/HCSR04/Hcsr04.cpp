#include "Hcsr04.h"

// 构造函数实现
Hcsr04::Hcsr04(uint8_t trigPin, uint8_t echoPin) {
    _trigPin = trigPin;
    _echoPin = echoPin;
}

// 初始化函数实现
void Hcsr04::begin() {
    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);
}

// 测量距离函数实现
float Hcsr04::getDistance() {
    // 发送10us的高电平触发信号
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);
    // TRIG引脚产生一个大于10us的高电平信号，才能激发HC-SR04超声波传感器发射超声波
    // 当TRIG变低后，HC-SR04芯片发送一段持续约 8 个 40kHz 周期的方波信号，即 200μs 左右

    // 检测高电平持续时间(微秒)
    double delta_time = pulseIn(_echoPin, HIGH);
    
    // 计算距离(单位：cm)，声速取0.0343cm/us
    float detect_distance = delta_time * 0.0343 / 2;
    
    return detect_distance;
}

// 测量并打印距离
void Hcsr04::printDistance() {
    float distance = getDistance();
    Serial.printf("distance=%.2f cm\n", distance);
}
