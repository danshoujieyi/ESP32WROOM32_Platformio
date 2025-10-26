#pragma once
#include "LDS.h"

// YDLIDAR X4激光雷达设备类，继承自LDS基类
class LDS_YDLIDAR_X4 : public LDS {
  public:
    // 初始化激光雷达（重写基类方法）
    virtual void init() override;

    // 启动激光雷达扫描（重写基类方法）
    virtual result_t start() override;
    // 停止激光雷达扫描（重写基类方法）
    virtual result_t stop() override;
    // 主循环处理函数（需从Arduino的loop()中频繁调用，重写基类方法）
    virtual void loop() override;

    // 获取串口通信波特率（重写基类方法）
    virtual uint32_t getSerialBaudRate() override;
    // 获取当前扫描频率（Hz，重写基类方法）
    virtual float getCurrentScanFreqHz() override;
    // 获取目标扫描频率（Hz，重写基类方法）
    virtual float getTargetScanFreqHz() override;
    // 获取采样率（Hz，重写基类方法）
    virtual int getSamplingRateHz() override;
    // 检查设备是否处于活跃状态（重写基类方法）
    virtual bool isActive() override;
    // 获取设备型号名称（重写基类方法）
    virtual const char* getModelName() override;

    // 设置目标扫描频率（Hz，重写基类方法）
    virtual result_t setScanTargetFreqHz(float freq) override;

  protected:
    bool motor_enabled; // 电机使能状态（true为开启，false为关闭）
    unsigned long int ring_start_ms[2]; // 用于计算扫描周期的时间戳数组（存储上一次和当前扫描开始时间）

  protected:
    static const uint32_t DEFAULT_TIMEOUT_MS = 500; // 默认超时时间（毫秒）
    static const uint8_t PACKAGE_SAMPLE_MAX_LENGTH = 40; // 数据包中最大采样点数

    // 数据包类型枚举
    typedef enum {
      CT_NORMAL = 0,      // 普通数据包
      CT_RING_START = 1,  // 扫描圈起始数据包
      CT_TAIL,            // 尾部数据包
    } CT;
    
    // 单个激光点信息结构体（紧凑打包，无字节对齐）
    struct node_info_t {
      uint8_t    sync_quality;       // 同步质量信息
      uint16_t   angle_q6_checkbit;  // 角度信息（q6格式）及校验位
      uint16_t   distance_q2;        // 距离信息（q2格式）
    } __attribute__((packed)) ;
    
    // 设备信息结构体（紧凑打包，无字节对齐）
    struct device_info_t{
      uint8_t   model;               // 设备型号
      uint16_t  firmware_version;    // 固件版本
      uint8_t   hardware_version;    // 硬件版本
      uint8_t   serialnum[16];       // 设备序列号（16字节）
    } __attribute__((packed)) ;
    
    // 设备健康状态结构体（紧凑打包，无字节对齐）
    struct device_health_t {
      uint8_t   status;              // 健康状态（0为正常，其他为异常）
      uint16_t  error_code;          // 错误代码（状态异常时有效）
    } __attribute__((packed))  ;
    
    // 命令数据包结构体（紧凑打包，无字节对齐）
    struct cmd_packet_t {
      uint8_t syncByte;              // 同步字节（固定为0xA5）
      uint8_t cmd_flag;              // 命令标志（包含命令类型及是否带 payload）
      uint8_t size;                  // payload 大小
      uint8_t data;                  // payload 数据起始位
    } __attribute__((packed)) ;
    
    // 响应头结构体（紧凑打包，无字节对齐）
    struct ans_header_t {
      uint8_t  syncByte1;            // 同步字节1（固定为0xA5）
      uint8_t  syncByte2;            // 同步字节2（固定为0x5A）
      uint32_t size:30;              // 数据包大小（30位）
      uint32_t subType:2;            // 子类型（2位）
      uint8_t  type;                 // 数据包类型
    } __attribute__((packed));
    
    // 节点数据包结构体（紧凑打包，无字节对齐）
    struct node_package_t {
      uint16_t  package_Head;                // 数据包头部标识（固定为0x55AA）
      uint8_t   package_CT;                  // 数据包类型（对应CT枚举）
      uint8_t   nowPackageNum;               // 当前数据包中的采样点数
      uint16_t  packageFirstSampleAngle;     // 数据包中第一个采样点的角度
      uint16_t  packageLastSampleAngle;      // 数据包中最后一个采样点的角度
      uint16_t  checkSum;                    // 校验和
      uint16_t  packageSampleDistance[PACKAGE_SAMPLE_MAX_LENGTH]; // 采样点距离数组（最大40个点）
    } __attribute__((packed)) ;

  protected:
    // 使能/禁用电机
    virtual void enableMotor(bool enable);
    // 获取设备健康状态（超时时间默认为DEFAULT_TIMEOUT_MS）
    LDS::result_t getHealth(device_health_t & health, uint32_t timeout = DEFAULT_TIMEOUT_MS);
    // 获取设备信息（超时时间默认为DEFAULT_TIMEOUT_MS）
    LDS::result_t getDeviceInfo(device_info_t & info, uint32_t timeout = DEFAULT_TIMEOUT_MS);
    // 终止扫描操作
    LDS::result_t abort();
    // 启动扫描操作（force为true时强制启动，超时时间默认为DEFAULT_TIMEOUT_MS*2）
    LDS::result_t startScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT_MS*2);
    // 等待一个采样数据包到达
    virtual LDS::result_t waitScanDot();
    // 发送命令到设备（cmd为命令类型，payload为附加数据，payloadsize为数据大小）
    LDS::result_t sendCommand(uint8_t cmd, const void * payload = NULL, size_t payloadsize = 0);
    // 等待响应头（header用于存储接收的响应头，超时时间默认为DEFAULT_TIMEOUT_MS）
    LDS::result_t waitResponseHeader(ans_header_t * header, uint32_t timeout = DEFAULT_TIMEOUT_MS);
    // 标记扫描开始时间（用于计算扫描频率）
    void markScanTime();
    // 检查设备信息（解析当前字节并拼接设备信息字符串）
    void checkInfo(int currentByte);

  protected:
    // 通信命令常量定义
    static const uint8_t CMD_STOP = 0x65;          // 停止扫描命令
    static const uint8_t CMD_SCAN = 0x60;          // 启动扫描命令
    static const uint8_t CMD_FORCE_SCAN = 0x61;    // 强制启动扫描命令
    static const uint8_t CMD_FORCE_STOP = 0x00;    // 强制停止命令
    static const uint8_t CMD_GET_DEV_INFO = 0x90;  // 获取设备信息命令
    static const uint8_t CMD_GET_DEV_HEALTH = 0x92;// 获取设备健康状态命令
    static const uint8_t CMD_SYNC_BYTE = 0xA5;     // 命令同步字节
    static const uint16_t CMDFLAG_HAS_PAYLOAD = 0x8000; // 命令带payload标志

    // 响应类型常量定义
    static const uint8_t ANS_TYPE_DEV_INFO = 0x4;   // 设备信息响应类型
    static const uint8_t ANS_TYPE_DEV_HEALTH = 0x6; // 设备健康状态响应类型
    static const uint8_t ANS_SYNC_BYTE1 = 0xA5;     // 响应同步字节1
    static const uint8_t ANS_SYNC_BYTE2 = 0x5A;     // 响应同步字节2
    static const uint8_t ANS_TYPE_MEAS = 0x81;      // 测量数据响应类型

    // 测量数据相关位定义
    static const uint8_t RESP_MEAS_SYNCBIT = (0x1<<0);         // 同步位
    static const uint8_t RESP_MEAS_CHECKBIT = (0x1<<0);        // 校验位
    static const uint8_t RESP_MEAS_ANGLE_SHIFT = 1;            // 角度值移位量
    static const uint8_t RESP_MEAS_ANGLE_SAMPLE_SHIFT = 8;     // 角度采样移位量
    
    // 数据包相关常量
    static const uint8_t PACKAGE_SAMPLE_BYTES = 2;     // 每个采样点占用的字节数
    static const uint16_t NODE_DEFAULT_QUALITY = 10;   // 默认采样质量（10<<2）
    static const uint8_t NODE_SYNC = 0x01;             // 同步节点标识
    static const uint8_t NODE_NOT_SYNC = 2;            // 非同步节点标识
    static const uint8_t PACKAGE_PAID_BYTES = 10;      // 数据包头部字节数（不含采样数据）
    static const uint16_t PH = 0x55AA;                 // 数据包头部标识

    // 接收与解析相关变量
    int recvPos = 0;                                  // 当前接收位置
    uint8_t package_Sample_Num = 0;                   // 数据包中的采样点数
    int package_recvPos = 0;                          // 数据包接收位置
    int package_sample_sum = 0;                       // 采样数据总字节数

    node_package_t package;                           // 当前处理的数据包

    uint16_t package_Sample_Index = 0;                // 当前处理的采样点索引
    float IntervalSampleAngle = 0;                    // 采样点角度间隔
    float IntervalSampleAngle_LastPackage = 0;        // 上一个数据包的角度间隔
    uint16_t FirstSampleAngle = 0;                    // 第一个采样点角度
    uint16_t LastSampleAngle = 0;                     // 最后一个采样点角度
    uint16_t CheckSum = 0;                            // 接收的校验和

    uint16_t CheckSumCal = 0;                         // 计算的校验和
    uint16_t SampleNumlAndCTCal = 0;                  // 采样数与类型的计算值
    uint16_t LastSampleAngleCal = 0;                  // 最后一个采样点角度的计算值
    bool CheckSumResult = true;                       // 校验和结果（true为通过）
    uint16_t Valu8Tou16 = 0;                          // 字节转16位整数的临时变量

    uint8_t state = 0;                                // 接收状态机状态

    float scan_freq_hz = 0;                           // 扫描频率（Hz）
    bool scan_completed = false;                      // 扫描圈完成标志
};