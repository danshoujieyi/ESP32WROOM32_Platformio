#include "Arduino.h"
#include <Esp32PcntEncoder.h>

// 日志标签，用于调试输出
static const char *TAG = "rotary_encoder";
// PCNT计数器默认上限值（达到此值触发高限溢出）
#define EC11_PCNT_DEFAULT_HIGH_LIMIT (100)
// PCNT计数器默认下限值（达到此值触发低限溢出）
#define EC11_PCNT_DEFAULT_LOW_LIMIT (-100)

// 错误检查宏：简化错误处理，条件不满足时打印日志并返回false
#define ROTARY_CHECK(a, msg, tag, ret, ...)                                       \
	do                                                                            \
	{                                                                             \
		if (unlikely(!(a)))                                                       \
		{                                                                         \
			ESP_LOGE(TAG, "%s(%d): " msg, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
			return false;                                                         \
		}                                                                         \
	} while (0)

/**
 * @brief PCNT计数器溢出中断处理函数
 * @param arg 传入的编码器对象指针
 * 功能：当计数器达到上限或下限的，累加溢出值以扩展计数范围
 */
static void pcnt_overflow_handler(void *arg)
{
	// 将传入的参数转换为编码器对象指针
	Esp32PcntEncoder *encoder = (Esp32PcntEncoder *)arg;
	uint32_t status = 0;
	// 获取当前PCNT单元的事件状态
	pcnt_get_event_status(encoder->pcntUnit, &status);

	// 若触发高限溢出事件（达到上限100）
	if (status & PCNT_EVT_H_LIM)
	{
		encoder->accumu_count += EC11_PCNT_DEFAULT_HIGH_LIMIT;
	}
	// 若触发低限溢出事件（达到下限-100）
	else if (status & PCNT_EVT_L_LIM)
	{
		encoder->accumu_count += EC11_PCNT_DEFAULT_LOW_LIMIT;
	}
}

/**
 * @brief 初始化编码器
 * @param _pcntUnit PCNT单元编号（0或1，ESP32支持2个PCNT单元）
 * @param pinA 编码器A相引脚
 * @param pinB 编码器B相引脚
 * @return 初始化成功返回true，失败返回false
 */
bool Esp32PcntEncoder::init(int _pcntUnit, int pinA, int pinB)
{
	this->_pinA = pinA;       // 保存A相引脚
	this->_pinB = pinB;       // 保存B相引脚
	this->pcntUnit = (pcnt_unit_t)_pcntUnit;  // 转换为PCNT单元枚举类型

	// 1. 配置引脚模式为上拉输入（避免引脚悬空产生噪声）
	pinMode(_pinA, INPUT_PULLUP);
	pinMode(_pinB, INPUT_PULLUP);

	// 2. 配置PCNT通道0（对应A相脉冲）
	pcnt_config_t dev_config = {
		.pulse_gpio_num = (gpio_num_t)_pinA,  // 脉冲输入引脚（A相）
		.ctrl_gpio_num = (gpio_num_t)_pinB,   // 控制引脚（B相，用于判断方向）
		.lctrl_mode = PCNT_MODE_REVERSE,      // 控制引脚为低电平时，反转计数方向
		.hctrl_mode = PCNT_MODE_KEEP,         // 控制引脚为高电平时，保持当前计数方向
		.pos_mode = PCNT_COUNT_DEC,           // 脉冲正跳变时计数减
		.neg_mode = PCNT_COUNT_INC,           // 脉冲负跳变时计数加
		.counter_h_lim = EC11_PCNT_DEFAULT_HIGH_LIMIT,  // 计数器上限
		.counter_l_lim = EC11_PCNT_DEFAULT_LOW_LIMIT,  // 计数器下限
		.unit = pcntUnit,                     // 所属PCNT单元
		.channel = PCNT_CHANNEL_0,            // 使用通道0
	};
	// 检查通道0配置是否成功
	ROTARY_CHECK(pcnt_unit_config(&dev_config) == ESP_OK, "配置PCNT通道0失败", err, ESP_FAIL);

	// 配置PCNT通道1（对应B相脉冲，与通道0配合实现方向识别）
	dev_config.pulse_gpio_num = (gpio_num_t)_pinB;  // 脉冲输入引脚（B相）
	dev_config.ctrl_gpio_num = (gpio_num_t)_pinA;   // 控制引脚（A相）
	dev_config.channel = PCNT_CHANNEL_1;            // 使用通道1
	dev_config.pos_mode = PCNT_COUNT_INC;           // 脉冲正跳变时计数加
	dev_config.neg_mode = PCNT_COUNT_DEC;           // 脉冲负跳变时计数减
	// 检查通道1配置是否成功
	ROTARY_CHECK(pcnt_unit_config(&dev_config) == ESP_OK, "配置PCNT通道1失败", err, ESP_FAIL);

	// 3. 暂停计数器并清零初始值
	pcnt_counter_pause(pcntUnit);
	pcnt_counter_clear(pcntUnit);

	// 若使用PCNT单元0，安装中断服务（只需安装一次）
	if (pcntUnit == PCNT_UNIT_0)
	{
		ROTARY_CHECK(pcnt_isr_service_install(0) == ESP_OK, "安装中断服务失败", err, ESP_FAIL);
	}

	// 为当前PCNT单元添加溢出中断处理函数，并传入编码器对象作为参数
	pcnt_isr_handler_add(pcntUnit, pcnt_overflow_handler, this);

	// 使能高限和低限溢出事件（触发时会调用中断函数）
	pcnt_event_enable(pcntUnit, PCNT_EVT_H_LIM);
	pcnt_event_enable(pcntUnit, PCNT_EVT_L_LIM);

	// 设置防抖动滤波器（过滤1000us以下的高频噪声）
	setGlitchFilter(1000);
	// 启动计数器
	start();

	return true;
}

/**
 * @brief 设置防抖动滤波器
 * @param max_glitch_us 最大滤波时间（微秒），短于此时间的脉冲会被过滤
 * @return 配置成功返回true，失败返回false
 */
bool Esp32PcntEncoder::setGlitchFilter(uint32_t max_glitch_us)
{
	// 设置滤波值（PCNT的滤波单位为系统时钟周期，这里简化为1:1映射）
	ROTARY_CHECK(pcnt_set_filter_value(pcntUnit, max_glitch_us * 1) == ESP_OK, "设置滤波器失败", err, ESP_FAIL);
	if (max_glitch_us)
	{
		pcnt_filter_enable(pcntUnit);  // 启用滤波器
	}
	else
	{
		pcnt_filter_disable(pcntUnit); // 禁用滤波器
	}
	return true;
}

/**
 * @brief 启动计数器
 * @return 成功返回true
 */
bool Esp32PcntEncoder::start()
{
	pcnt_counter_resume(pcntUnit);  // 恢复计数器运行
	pcnt_counter_clear(pcntUnit);   // 清零当前计数值
	return true;
}

/**
 * @brief 重置计数值（软件层面，不影响硬件计数器）
 * @return 成功返回true
 */
bool Esp32PcntEncoder::reset()
{
	_ticks = 0;  // 重置累计脉冲数
	return true;
}

/**
 * @brief 停止计数器
 * @return 成功返回true
 */
bool Esp32PcntEncoder::stop()
{
	pcnt_counter_pause(pcntUnit);   // 暂停计数器
	pcnt_counter_clear(pcntUnit);   // 清零当前计数值
	return true;
}

/**
 * @brief 获取累计脉冲数（总 ticks）
 * @return 包含溢出补偿的总脉冲数
 */
int32_t Esp32PcntEncoder::getTicks()
{
	int16_t val = 0;
	// 读取硬件计数器当前值（范围：-100~100）
	pcnt_get_counter_value(pcntUnit, &val);
	// 总脉冲数 = 硬件当前值 + 溢出累加值（突破硬件计数范围限制）
	_ticks = val + accumu_count;
	return _ticks;
}

/**
 * @brief 计算旋转速度
 * @return 速度值（单位由setPeerPcntDistance设置），速度单位输出目前为mm/s
 */
float Esp32PcntEncoder::getSpeed()
{
	// 计算当前与上一次更新的时间差（微秒） 
	// millis（）毫秒级延时
	uint64_t dt = micros() - _lastUpdateSpeedTime;
	// 速度公式：(脉冲变化量 × 每脉冲距离) / 时间差
	float speed = (float)(getTicks() - _lastUpdateSpeedTick) * _peerPcntDistance / dt;
	float filteredSpeed;
	// 首次调用时直接使用原始值作为初始值
    if (_isFirstSpeedUpdate) {
        filteredSpeed = speed;
        _isFirstSpeedUpdate = false;
    } else {
        // 滤波公式：当前滤波值 = 系数×当前原始值 + (1-系数)×上一次滤波值
        filteredSpeed = 0.3 * speed + 0.7 * _lastFilteredSpeed;
    }
	_lastFilteredSpeed = filteredSpeed;
	// 更新上一次的时间和脉冲数
	_lastUpdateSpeedTick = getTicks();
	_lastUpdateSpeedTime = micros();

	return filteredSpeed;
}

/**
 * @brief 设置每脉冲对应的物理距离（用于速度计算）
 * @param peerPcntDistance 每脉冲的物理距离（如：1脉冲=0.1963495f毫米）
 * @return 配置成功返回true
 */
bool Esp32PcntEncoder::setPeerPcntDistance(float peerPcntDistance)
{
	// 乘以1e6是因为getSpeed中时间单位为微秒（1秒=1e6微秒），统一单位
	_peerPcntDistance = peerPcntDistance * 1000 * 1000;
	return true;
}

/**
 * @brief 设置每转对应的脉冲数（用于转速计算）
 * @param pulsesPerRevolution 每转脉冲数(mg310电机为1040脉冲一转)
 * @return 配置成功返回true
 */
bool Esp32PcntEncoder::setPulsesPerRevolution(float pulsesPerRevolution)
{
	if (pulsesPerRevolution <= 0) {
		return false;  // 无效值
	}
	_pulsesPerRevolution = pulsesPerRevolution;
	return true;
}

/**
 * @brief 计算转速（rpm）
 */
float Esp32PcntEncoder::getRpm()
{
	// 使用rpm专用的历史变量计算差值
	uint64_t dt = micros() - _lastUpdateRpmTime;
	if (dt == 0) {
		return _lastFilteredRpm;  // 避免除零
	}
	
	int32_t tickDiff = getTicks() - _lastUpdateRpmTick;
	// 转速公式：(脉冲变化量 / 每转脉冲数) / (时间(秒)) * 60秒 = 转/分钟
	float rpm = (float)tickDiff * 60.0f * 1000000.0f / (_pulsesPerRevolution * dt);
	
	float filteredRpm;
	if (_isFirstRpmUpdate) {
		filteredRpm = rpm;
		_isFirstRpmUpdate = false;
	} else {
		filteredRpm = 0.3 * rpm + 0.7 * _lastFilteredRpm;
	}
	
	// 更新rpm专用的历史变量
	_lastFilteredRpm = filteredRpm;
	_lastUpdateRpmTick = getTicks();
	_lastUpdateRpmTime = micros();
	
	return filteredRpm;
}
