#pragma once
#include "LDS_YDLIDAR_X4.h"

/**
 * @brief YDLIDAR X2/X2L激光雷达设备类，继承自X4型号的实现
 * 用于实现X2/X2L型号激光雷达的特定功能，重写了父类的部分接口
 */
class LDS_YDLIDAR_X2_X2L : public LDS_YDLIDAR_X4 {
  public:
    /**
     * @brief 启动激光雷达
     * @return 操作结果（成功/失败等状态）
     */
    virtual result_t start() override;
    
    /**
     * @brief 停止激光雷达
     * @return 操作结果（成功/失败等状态）
     */
    virtual result_t stop() override;
    
    /**
     * @brief 获取激光雷达型号名称
     * @return 型号名称字符串
     */
    virtual const char* getModelName() override;

    /**
     * @brief 获取当前扫描频率（Hz）
     * @return 当前扫描频率值
     */
    virtual float getCurrentScanFreqHz() override;
    
    /**
     * @brief 获取串口通信波特率
     * @return 波特率数值
     */
    virtual uint32_t getSerialBaudRate() override;
    
    /**
     * @brief 获取目标扫描频率（Hz）
     * @return 目标扫描频率值
     */
    virtual float getTargetScanFreqHz() override;
    
    /**
     * @brief 获取采样率（Hz）
     * @return 采样率数值
     */
    virtual int getSamplingRateHz() override;
  
  protected:
    /**
     * @brief 使能或禁用电机
     * @param enable 电机使能状态（true为使能，false为禁用）
     */
    virtual void enableMotor(bool enable) override;
};