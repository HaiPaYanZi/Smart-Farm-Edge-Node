#ifndef __MYSENSOR_H__
#define __MYSENSOR_H__

#include <stdint.h>
#include "rule_engine.h"        /* 复用 SENSOR_* 编号与 SENSOR_INVALID 约定 */

/* 传感器快照刷新:每个控制周期开始调一次,统一读一遍所有传感器
   (规则引擎评估时回调只查快照,不直接碰 I2C,避免重复读取) */
void MySensor_Update(void);

/* 规则引擎取值回调:按 sensor_id 返回对应传感器快照值
   读取失败返回 SENSOR_INVALID → 对应规则自动降级禁用(文档 §4.2) */
float MySensor_GetValue(uint16_t sensor_id);

#endif
