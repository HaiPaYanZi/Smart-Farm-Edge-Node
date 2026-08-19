#ifndef __BUZZER_H__
#define __BUZZER_H__

#include <stdint.h>

/* 蜂鸣器控制:on=1 响 / on=0 停(有源蜂鸣器,GPIO 电平直接驱动)
   用途:堵转/故障报警(文档 §4.1 故障管理) */
void Buzzer_Control(uint8_t on);

#endif
