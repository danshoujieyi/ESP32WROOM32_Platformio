#ifndef __DWT_H__
#define __DWT_H__

#include "Arduino.h"
#include <stdio.h>

/**
 * @brief 获取两次调用的时间间隔（单位：秒）
 * @param last_time 指向存储上一次时间戳的变量指针
 * @retval 两次调用的时间间隔（秒，float类型）
 */
float dwt_get_delta(uint64_t *last_time);

#endif /* DWT_H_ */