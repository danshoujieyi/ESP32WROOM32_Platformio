#include "Dwt.h"

/**
 * @brief 获取两次调用的时间间隔（单位：秒）
 * @param last_time 指向存储上一次时间戳的变量指针
 * @retval 两次调用的时间间隔（秒，float类型）
 */
float dwt_get_delta(uint64_t *last_time)
{
    // 获取当前微秒时间戳（ESP32的micros()返回uint64_t）
    uint64_t current_micros = micros();
    
    // 计算时间差（微秒），处理溢出情况（micros()约70分钟溢出一次）
    uint64_t delta_micros;
    if (current_micros >= *last_time) {
        delta_micros = current_micros - *last_time;
    } else {
        // 处理溢出：假设溢出后从0重新计数
        delta_micros = (UINT64_MAX - *last_time) + current_micros + 1;
    }
    
    // 更新上一次时间戳为当前时间
    *last_time = current_micros;
    
    // 转换为秒（微秒 -> 秒：除以1e6）
    return (float)delta_micros / 1000000.0f;
}
