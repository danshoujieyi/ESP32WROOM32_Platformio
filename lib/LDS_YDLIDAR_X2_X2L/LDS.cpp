#include "LDS.h"

// 构造函数：初始化所有回调函数指针为NULL
LDS::LDS() {
  scan_point_callback = NULL;
  packet_callback = NULL;
  serial_write_callback = NULL;
  serial_read_callback = NULL;
  motor_pin_callback = NULL;
  info_callback = NULL;
  error_callback = NULL;
}

// 设置扫描点回调函数
// 参数：scan_point_callback - 外部定义的扫描点回调函数
void LDS::setScanPointCallback(ScanPointCallback scan_point_callback) {
  this->scan_point_callback = scan_point_callback; 
}

// 设置电机引脚控制回调函数
// 参数：motor_pin_callback - 外部定义的电机引脚控制回调函数
void LDS::setMotorPinCallback(MotorPinCallback motor_pin_callback) {
  this->motor_pin_callback = motor_pin_callback;
}

// 设置数据包回调函数
// 参数：packet_callback - 外部定义的数据包回调函数
void LDS::setPacketCallback(PacketCallback packet_callback) {
  this->packet_callback = packet_callback; 
}

// 设置串口读取回调函数
// 参数：serial_read_callback - 外部定义的串口读取回调函数
void LDS::setSerialReadCallback(SerialReadCallback serial_read_callback) {
  this->serial_read_callback = serial_read_callback; 
}

// 设置串口写入回调函数
// 参数：serial_write_callback - 外部定义的串口写入回调函数
void LDS::setSerialWriteCallback(SerialWriteCallback serial_write_callback) {
  this->serial_write_callback = serial_write_callback; 
}

// 设置信息回调函数
// 参数：info_callback - 外部定义的信息回调函数
void LDS::setInfoCallback(InfoCallback info_callback) {
  this->info_callback = info_callback; 
}

// 设置错误回调函数
// 参数：error_callback - 外部定义的错误回调函数
void LDS::setErrorCallback(ErrorCallback error_callback) {
  this->error_callback = error_callback; 
}

// 发送扫描点数据到注册的回调函数
// 参数：
//   angle_deg - 角度（度）
//   dist_mm - 距离（毫米，<=0表示无效点）
//   quality - 质量值
//   scan_completed - 扫描是否完成标志
void LDS::postScanPoint(float angle_deg, float dist_mm, float quality,
  bool scan_completed) {
  // dist_mm <=0 表示无效点
  if (scan_point_callback)
    scan_point_callback(angle_deg, dist_mm, quality, scan_completed);
}

// 发送数据包到注册的回调函数
// 参数：
//   data - 数据包缓冲区
//   length - 数据包长度
//   scan_completed - 扫描是否完成标志
void LDS::postPacket(uint8_t* data, uint16_t length, bool scan_completed) {
  if (packet_callback)
    packet_callback(data, length, scan_completed);
}

// 通过注册的回调函数控制电机引脚
// 参数：
//   value - 控制值（PWM值或高低电平）
//   pin - 引脚类型
void LDS::setMotorPin(float value, lds_pin_t pin) {
  if (motor_pin_callback)
    motor_pin_callback(value, pin);
}

// 通过注册的回调函数写入串口数据
// 参数：
//   buffer - 写入数据缓冲区
//   length - 写入数据长度
// 返回：实际写入的字节数（未注册回调则返回0）
size_t LDS::writeSerial(const uint8_t * buffer, size_t length) {
  return (serial_write_callback) ? serial_write_callback(buffer, length) : 0;
}

// 通过注册的回调函数读取串口数据
// 返回：读取到的字节（未注册回调则返回ERROR_NOT_CONFIGURED）
int LDS::readSerial() {
  return (serial_read_callback) ? serial_read_callback() : ERROR_NOT_CONFIGURED;
}

// 发送设备信息到注册的回调函数
// 参数：
//   code - 信息类型
//   info - 信息内容
void LDS::postInfo(info_t code, String info) {
  if (info_callback)
    info_callback(code, info);
}

// 发送错误信息到注册的回调函数
// 参数：
//   code - 错误码
//   aux_info - 辅助错误信息
void LDS::postError(result_t code, String aux_info) {
  if (error_callback)
    error_callback(code, aux_info);
}

// 设置扫描PID采样周期（默认未实现）
// 参数：sample_period_ms - 采样周期（毫秒）
// 返回：操作结果（默认返回ERROR_NOT_IMPLEMENTED）
LDS::result_t LDS::setScanPIDSamplePeriodMs(uint32_t sample_period_ms) {
  return ERROR_NOT_IMPLEMENTED;
}

// 设置扫描PID控制参数（默认未实现）
// 参数：
//   Kp - 比例系数
//   Ki - 积分系数
//   Kd - 微分系数
// 返回：操作结果（默认返回ERROR_NOT_IMPLEMENTED）
LDS::result_t LDS::setScanPIDCoeffs(float Kp, float Ki, float Kd) {
  return ERROR_NOT_IMPLEMENTED;
}

// 将信息类型码转换为字符串描述
// 参数：code - 信息类型码
// 返回：对应的字符串描述
String LDS::infoCodeToString(info_t code) {
  switch (code) {
    case INFO_MODEL:
      return "设备型号";
    case INFO_FIRMWARE_VERSION:
      return "固件版本";
    case INFO_HARDWARE_VERSION:
      return "硬件版本";
    case INFO_SERIAL_NUMBER:
      return "序列号";
    case INFO_DEVICE_HEALTH:
      return "设备健康状态";
    case INFO_SAMPLING_RATE:
      return "采样率（Hz）";
    case INFO_DEFAULT_TARGET_SCAN_FREQ_HZ:
      return "默认目标扫描频率（Hz）";
    case INFO_OTHER:
      return "其他信息";
    default:
      return "未知信息类型";
  }
}

// 将结果码转换为字符串描述
// 参数：code - 结果码
// 返回：对应的字符串描述
String LDS::resultCodeToString(result_t code) {
  switch (code) {
    case LDS::RESULT_OK:
      return "成功";
    case ERROR_TIMEOUT:
      return "超时错误";
    case ERROR_INVALID_PACKET:
      return "无效数据包错误";
    case ERROR_CHECKSUM:
      return "校验和错误";
    case ERROR_NOT_READY:
      return "设备未就绪错误";
    case ERROR_NOT_IMPLEMENTED:
      return "功能未实现错误";
    case ERROR_NOT_CONFIGURED:
      return "未配置错误";
    case ERROR_MOTOR_DISABLED:
      return "电机已禁用错误";
    case ERROR_INVALID_MODEL:
      return "无效模型错误";
    case ERROR_DEVICE_INFO:
      return "设备信息获取错误";
    case ERROR_DEVICE_HEALTH:
      return "设备健康状态错误";
    case ERROR_START_SCAN:
      return "启动扫描错误";
    case ERROR_INVALID_VALUE:
      return "无效值错误";
    default:
      return "未知结果码";
  }
}

// 将引脚ID转换为字符串描述
// 参数：pin - 引脚ID
// 返回：对应的字符串描述
String LDS::pinIDToString(lds_pin_t pin) {
  switch (pin) {
    case LDS_MOTOR_EN_PIN:
      return "电机使能引脚";
    case LDS_MOTOR_PWM_PIN:
      return "电机PWM引脚";
    default:
      return "未知引脚";
  }
}

// 将引脚状态转换为字符串描述
// 参数：state - 引脚状态
// 返回：对应的字符串描述
String LDS::pinStateToString(lds_pin_state_t state) {
  switch (state) {
    case VALUE_PWM:
      return "PWM模式";
    case VALUE_LOW:
      return "低电平";
    case VALUE_HIGH:
      return "高电平";
    case DIR_INPUT:
      return "输入方向";
    case DIR_OUTPUT_CONST:
      return "输出方向（恒定电平）";
    case DIR_OUTPUT_PWM:
      return "输出方向（PWM模式）";
    default:
      return "未知状态";
  }
}