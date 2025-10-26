#pragma once
#include <Arduino.h>

// 需实现的接口类（LDS激光雷达抽象接口）
class LDS {
  public:
    // LDS引脚类型枚举
    enum lds_pin_t {
      LDS_MOTOR_EN_PIN,  // LDS电机使能引脚
      LDS_MOTOR_PWM_PIN, // LDS电机PWM控制引脚
    };

    // 操作结果状态码枚举
    enum result_t {
      RESULT_OK = 0,              // 操作成功
      ERROR_TIMEOUT = -1,         // 超时错误
      ERROR_INVALID_PACKET = -2,  // 无效数据包错误
      ERROR_CHECKSUM = -3,        // 校验和错误
      ERROR_NOT_READY = -4,       // 设备未就绪错误
      ERROR_NOT_IMPLEMENTED = -5, // 功能未实现错误
      ERROR_NOT_CONFIGURED = -6,  // 未配置错误
      ERROR_MOTOR_DISABLED = -7,  // 电机已禁用错误
      ERROR_INVALID_MODEL = -8,   // 无效模型错误
      ERROR_DEVICE_INFO = -9,     // 设备信息获取错误
      ERROR_DEVICE_HEALTH = -10,  // 设备健康状态错误
      ERROR_START_SCAN = -11,     // 启动扫描错误
      ERROR_INVALID_VALUE = -12,  // 无效值错误
      ERROR_UNKNOWN = -13,        // 未知错误
      ERROR_INVALID_DATA = -14,   // 无效数据错误
      ERROR_UNAVAILABLE = -15,    // 不可用错误
    };

    // 设备信息类型枚举
    enum info_t {
      INFO_MODEL,                     // 设备型号
      INFO_FIRMWARE_VERSION,          // 固件版本
      INFO_HARDWARE_VERSION,          // 硬件版本
      INFO_SERIAL_NUMBER,             // 序列号
      INFO_DEVICE_HEALTH,             // 设备健康状态
      INFO_SAMPLING_RATE,             // 采样率
      INFO_DEFAULT_TARGET_SCAN_FREQ_HZ,// 默认目标扫描频率（Hz）
      INFO_OTHER,                     // 其他信息
    };

    // 扫描点回调函数类型
    // 参数：角度（度）、距离（毫米）、质量值、扫描是否完成
    typedef void (*ScanPointCallback)(float, float, float, bool);
    // 数据包回调函数类型
    // 参数：数据缓冲区、数据长度、扫描是否完成
    typedef void (*PacketCallback)(uint8_t*, uint16_t, bool);
    // 电机引脚控制回调函数类型
    // 参数：控制值、引脚类型
    typedef void (*MotorPinCallback)(float, lds_pin_t);
    // 串口写入回调函数类型
    // 参数：写入数据缓冲区、数据长度，返回实际写入长度
    typedef size_t (*SerialWriteCallback)(const uint8_t *, size_t);
    // 串口读取回调函数类型
    // 返回读取到的字节（-1表示无数据）
    typedef int (*SerialReadCallback)(void);
    // 信息回调函数类型
    // 参数：信息类型、信息内容
    typedef void (*InfoCallback)(info_t, String);
    // 错误回调函数类型
    // 参数：错误码、辅助错误信息
    typedef void (*ErrorCallback)(result_t, String);

    // 默认值常量
    const int8_t DEFAULT_VALUE = 0;

    // 引脚状态枚举
    enum lds_pin_state_t {
      VALUE_PWM = 0,         // PWM值（具体值为PWM占空比）
      VALUE_LOW = -1,        // 低电平
      VALUE_HIGH = -2,       // 高电平
      DIR_INPUT = -3,        // 输入方向
      DIR_OUTPUT_CONST = -4, // 输出方向（恒定电平）
      DIR_OUTPUT_PWM = -5,   // 输出方向（PWM模式）
    };
  public:
    LDS();
    virtual void init() = 0;                  // 初始化（子类实现）
    virtual result_t start() = 0;             // 初始化并启动（从Arduino setup()中调用）
    virtual result_t stop() = 0;              // 停止电机和扫描
    virtual void loop() = 0;                  // 循环处理（从Arduino loop()中频繁调用）

    virtual uint32_t getSerialBaudRate() = 0; // 获取串口波特率
    virtual float getCurrentScanFreqHz() = 0; // 获取当前扫描频率（Hz）
    virtual float getTargetScanFreqHz() = 0;  // 获取目标扫描频率（Hz）
    virtual int getSamplingRateHz() = 0;      // 获取采样率（Hz）
    virtual bool isActive() = 0;              // 判断设备是否处于活动状态
    virtual const char* getModelName() = 0;   // 获取设备型号名称

    virtual result_t setScanTargetFreqHz(float freq) = 0; // 设置目标扫描频率（Hz）
    virtual result_t setScanPIDCoeffs(float Kp, float Ki, float Kd); // 设置扫描PID控制参数
    virtual result_t setScanPIDSamplePeriodMs(uint32_t sample_period_ms); // 设置扫描PID采样周期（毫秒）

    void setScanPointCallback(ScanPointCallback scan_callback);    // 设置扫描点回调函数
    void setMotorPinCallback(MotorPinCallback motor_pin_callback); // 设置电机引脚控制回调函数
    void setPacketCallback(PacketCallback packet_callback);        // 设置数据包回调函数
    void setSerialReadCallback(SerialReadCallback serial_read_callback); // 设置串口读取回调函数
    void setSerialWriteCallback(SerialWriteCallback serial_write_callback); // 设置串口写入回调函数
    void setInfoCallback(InfoCallback info_callback);              // 设置信息回调函数
    void setErrorCallback(ErrorCallback error_callback);           // 设置错误回调函数

    virtual String resultCodeToString(result_t code);  // 将结果码转换为字符串描述
    virtual String infoCodeToString(info_t code);      // 将信息类型码转换为字符串描述
    virtual String pinIDToString(lds_pin_t pin);       // 将引脚ID转换为字符串描述
    virtual String pinStateToString(lds_pin_state_t state); // 将引脚状态转换为字符串描述

  protected:
    // 发送扫描点数据到回调函数
    void postScanPoint(float angle_deg, float dist_mm, float quality, bool scan_completed);
    // 控制电机引脚（通过回调函数）
    void setMotorPin(float value, lds_pin_t pin);
    // 发送数据包到回调函数
    void postPacket(uint8_t* data, uint16_t length, bool scan_completed);
    // 读取串口数据（通过回调函数）
    int readSerial();
    // 写入数据到串口（通过回调函数）
    size_t writeSerial(const uint8_t * buffer, size_t length);
    // 发送设备信息到回调函数
    void postInfo(info_t code, String info);
    // 发送错误信息到回调函数
    void postError(result_t code, String aux_info);
    // void enableMotor(bool enable);  // （预留）使能电机

  protected:
    ScanPointCallback scan_point_callback;    // 扫描点回调函数指针
    PacketCallback packet_callback;           // 数据包回调函数指针
    SerialWriteCallback serial_write_callback;// 串口写入回调函数指针
    SerialReadCallback serial_read_callback;  // 串口读取回调函数指针
    MotorPinCallback motor_pin_callback;      // 电机引脚控制回调函数指针
    InfoCallback info_callback;               // 信息回调函数指针
    ErrorCallback error_callback;             // 错误回调函数指针
};