#ifndef __ESP32PCNTENCODER_H__
#define __ESP32PCNTENCODER_H__
#include "Arduino.h"
#include <driver/pcnt.h>
#include <hal/pcnt_hal.h>
#include <soc/pcnt_struct.h>

class Esp32PcntEncoder
{
public:
	Esp32PcntEncoder()  : _lastFilteredSpeed(0.0f),  
                          _lastFilteredRpm(0.0f),  // 初始化rpm滤波值
                          _isFirstSpeedUpdate(true),
                          _isFirstRpmUpdate(true)   // 初始化rpm首次标记
    {
        
    }

	bool init(int pcntUnit, int pinA, int pinB);
	
	bool setGlitchFilter(uint32_t max_glitch_us);
	bool start();
	bool reset();
	bool stop();

	int32_t getTicks();
	// 速度计算
	bool setPeerPcntDistance(float peerPcntDistance);
	float getSpeed();

	// 转速计算
	bool setPulsesPerRevolution(float pulsesPerRevolution);  // 设置每转脉冲数
	float getRpm();  // 获取转速(rpm)

private:
	int _pinA;
	int _pinB;

	int32_t _ticks{0};
	float _peerPcntDistance{0.1};
	float _lastFilteredSpeed;  // 存储上一次的滤波结果
    bool _isFirstSpeedUpdate;  // 标记是否是第一次计算速度
	uint32_t _lastUpdateSpeedTime;
	int32_t _lastUpdateSpeedTick;

	// 转速相关变量
	float _pulsesPerRevolution{10};  // 每转脉冲数
	float _lastFilteredRpm;         // rpm滤波值
	bool _isFirstRpmUpdate;         // rpm首次计算标记
	uint32_t _lastUpdateRpmTime;    // rpm专用的上一次时间
	int32_t _lastUpdateRpmTick;     // rpm专用的上一次脉冲数

public:
	pcnt_unit_t pcntUnit;
	int accumu_count{0};
};

#endif // __ESP32PCNTENCODER_H__
