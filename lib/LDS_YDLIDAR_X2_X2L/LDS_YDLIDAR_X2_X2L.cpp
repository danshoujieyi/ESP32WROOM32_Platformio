#include "LDS_YDLIDAR_X2_X2L.h"

/**
 * @brief 实现获取激光雷达型号名称的接口
 * @return 型号名称"YDLIDAR X2/X2L"
 */
const char* LDS_YDLIDAR_X2_X2L::getModelName() { return "YDLIDAR X2/X2L"; }

/**
 * @brief 实现启动激光雷达的接口
 * 启动流程：先使能电机，再发布型号信息
 * @return 操作结果（固定返回成功）
 */
LDS::result_t LDS_YDLIDAR_X2_X2L::start() {
  enableMotor(true);  // 开启电机
  postInfo(INFO_MODEL, getModelName());  // 发布型号信息到信息系统
  return LDS::RESULT_OK;  // 返回成功状态
}

/**
 * @brief 实现获取串口波特率的接口
 * X2/X2L型号固定使用115200波特率
 * @return 波特率115200
 */
uint32_t LDS_YDLIDAR_X2_X2L::getSerialBaudRate() {
  return 115200;
}

/**
 * @brief 实现获取目标扫描频率的接口
 * 返回默认设定的目标扫描频率
 * @return 目标扫描频率（默认值）
 */
float LDS_YDLIDAR_X2_X2L::getTargetScanFreqHz() {
  return DEFAULT_VALUE;
}

/**
 * @brief 实现获取采样率的接口
 * X2/X2L型号固定采样率为3000Hz
 * @return 采样率3000
 */
int LDS_YDLIDAR_X2_X2L::getSamplingRateHz() {
  return 3000;
}

/**
 * @brief 实现获取当前扫描频率的接口
 * 复用父类X4的当前扫描频率获取逻辑
 * @return 当前扫描频率值
 */
float LDS_YDLIDAR_X2_X2L::getCurrentScanFreqHz() {
  return LDS_YDLIDAR_X4::getCurrentScanFreqHz();  // 调用父类实现
}

/**
 * @brief 实现停止激光雷达的接口
 * 停止流程：禁用电机
 * @return 操作结果（固定返回成功）
 */
LDS::result_t LDS_YDLIDAR_X2_X2L::stop() {
  enableMotor(false);  // 关闭电机
  return RESULT_OK;  // 返回成功状态
}

/**
 * @brief 实现电机使能/禁用的具体逻辑
 *  - 使能时：设置电机引脚为输入模式（由外部控制）
 *  - 禁用时：设置电机引脚为固定输出模式并置高电平（X2L型号无法完全停止电机）
 * @param enable 电机使能状态
 */
void LDS_YDLIDAR_X2_X2L::enableMotor(bool enable) {
  motor_enabled = enable;  // 更新电机使能状态标记

  if (enable) {
    // 使能电机：设置引脚为输入模式
    setMotorPin(DIR_OUTPUT_PWM, LDS_MOTOR_PWM_PIN);
  } else {
    // 禁用电机：X2L无法完全停止，设置引脚为固定输出并置高
    setMotorPin(DIR_OUTPUT_CONST, LDS_MOTOR_PWM_PIN);
    setMotorPin(VALUE_HIGH, LDS_MOTOR_PWM_PIN);
  }

  // 以下为预留的整体激光雷达开关控制逻辑（当前未使用）
  // setMotorPin(enable ? VALUE_HIGH : VALUE_LOW, LDS_MOTOR_PWM_PIN);
}