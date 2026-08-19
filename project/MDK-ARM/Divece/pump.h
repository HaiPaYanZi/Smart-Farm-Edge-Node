#ifndef __PUMP_H__
#define __PUMP_H__

#include <stdint.h>

/* 水泵控制:on=1 开泵 / on=0 关泵(继电器通断)
   开/关语义对应规则引擎 ACT_PUMP 输出 */
void Pump_Control(uint8_t on);

/* 读水泵实际状态:1=开 0=关(读引脚实际电平——手动模式线圈直接操作
   执行器时,OLED 显示不会被规则引擎状态表误导) */
uint8_t Pump_IsOn(void);

#endif
