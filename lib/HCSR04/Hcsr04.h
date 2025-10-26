#ifndef HCSR04_H
#define HCSR04_H

#include "Arduino.h"

class Hcsr04 {
private:
    uint8_t _trigPin;  // TRIG引脚
    uint8_t _echoPin;  // ECHO引脚

public:
    // 构造函数，接收TRIG和ECHO引脚号
    Hcsr04(uint8_t trigPin, uint8_t echoPin);
    
    // 初始化函数，设置引脚模式
    void begin();
    
    // 测量距离，返回距离值(单位：cm)
    float getDistance();
    
    // 测量并打印距离
    void printDistance();
};

#endif // HCSR04_H

