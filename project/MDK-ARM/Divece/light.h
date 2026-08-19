#ifndef __LIGHT_H__
#define __LIGHT_H__

#include <stdint.h>

/* 补光灯控制:on=1 开(80% 亮度)/ on=0 关
   开/关语义对应规则引擎 ACT_LIGHT 输出(文档 §3.3) */
void Light_Control(uint8_t on);

/* 读补光灯实际状态:1=开 0=关(占空比 >0 视为开,读 PWM 实际输出——
   手动模式线圈直接操作执行器时,OLED 显示不会被规则引擎状态表误导) */
uint8_t Light_IsOn(void);

#endif
