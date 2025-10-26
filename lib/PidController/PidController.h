#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#include "Arduino.h"

#include <stdio.h>

#include "esp_system.h"
#include "esp_attr.h"
#include "math.h"

#define PID_NUM_MAX 30      // 最大PID实例数

#ifndef usr_abs
#define usr_abs(x) ((x > 0) ? x : -x)
#endif

// 新宏定义：按 pid_config_t 成员声明顺序初始化（兼容所有C++版本）
// 参数顺序必须与结构体成员声明顺序一致！
#define INIT_PID_CONFIG( \
    Kp_val,    /* 1. 比例系数 */ \
    Ki_val,    /* 2. 积分系数 */ \
    Kd_val,    /* 3. 微分系数 */ \
    IntegralLimit_val, /* 5. 积分限幅 */ \
    MaxOut_val, /* 6. 输出限幅 */ \
    Improve_val /* 7. 优化功能开关 */ \
) { \
    Kp_val,    /* 对应 Kp */ \
    Ki_val,    /* 对应 Ki */ \
    Kd_val,    /* 对应 Kd */ \
    IntegralLimit_val, /* 对应 IntegralLimit */ \
    MaxOut_val, /* 对应 MaxOut */ \
    (pid_improvement_e)Improve_val /* 对应 Improve */ \
}

/* PID 优化环节使能标志位,通过位与可以判断启用的优化环节 */
typedef enum
{
    PID_IMPROVE_NONE = 0X00,                // 0000 0000
    PID_Integral_Limit = 0x01,              // 0000 0001 积分限幅
    PID_Derivative_On_Measurement = 0x02,   // 0000 0010 微分先行
    PID_Trapezoid_Intergral = 0x04,         // 0000 0100 梯形积分
    PID_Proportional_On_Measurement = 0x08, // 0000 1000
    PID_OutputFilter = 0x10,                // 0001 0000 输出滤波
    PID_ChangingIntegrationRate = 0x20,     // 0010 0000 变速积分
    PID_DerivativeFilter = 0x40,            // 0100 0000 微分滤波器
    PID_ErrorHandle = 0x80,                 // 1000 0000
} pid_improvement_e;

/* PID 报错类型枚举*/
typedef enum error_type_e
{
    PID_ERROR_NONE = 0x00U,
    PID_MOTOR_BLOCKED_ERROR = 0x01U
} error_type_e;

typedef struct
{
    uint64_t error_count;
    error_type_e error_type;
} pid_ErrorHandler_t;

/* PID结构体 */
typedef struct
{
    //---------------------------------- init config block
    // config parameter
    float Kp;
    float Ki;
    float Kd;
    float IntegralLimit;     // 积分限幅
    float MaxOut;

    // improve parameter
    pid_improvement_e Improve;
    float DeadBand;
    float CoefA;             // 变速积分 For Changing Integral
    float CoefB;             // 变速积分 ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC;     // 输出滤波器 RC = 1/omegac
    float Derivative_LPF_RC; // 微分滤波器系数

    //-----------------------------------
    // for calculating
    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;
    float Last_ITerm;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;
    float Last_Dout;

    float Ref;

    uint64_t DWT_CNT;
    float dt;

    pid_ErrorHandler_t ERRORHandler;
} pid_obj_t;

/* 用于PID初始化的结构体*/
typedef struct // config parameter
{
    // basic parameter
    float Kp;
    float Ki;
    float Kd;
    float IntegralLimit; // 积分限幅
    float MaxOut;   // 输出限幅

    // improve parameter
    pid_improvement_e Improve;
    float DeadBand; // 死区
    float CoefA;         // AB为变速积分参数,变速积分实际上就引入了积分分离
    float CoefB;         // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC; // RC = 1/omegac
    float Derivative_LPF_RC;
} pid_config_t;

/**
 * @brief 初始化PID实例,并返回PID实例指针
 * @param config PID初始化配置
 */
pid_obj_t *pid_register(pid_config_t *config);

/**
 * @brief 计算PID输出
 *
 * @param pid     PID实例指针
 * @param measure 反馈值
 * @param ref     设定值
 * @return float  PID计算输出
 */
float pid_calculate(pid_obj_t *pid, float measure, float ref);

/**
 * @brief 清空一个pid的历史数据
 *
 * @param pid    PID实例
 */
void pid_clear(pid_obj_t *pid);



#endif // PIDCONTROLLER_H
