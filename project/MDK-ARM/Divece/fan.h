#ifndef __FAN_H__
#define __FAN_H__

#include <stdint.h>

/* 风扇控制:percent 0~100(转速百分比)
   内部映射:BSP 层 TIM3_CH1(PA6) → 占空比 = 转速 */
void Fan_Control(uint8_t percent);

/* 转速更新:反馈侧每 500ms 调一次,内部差分换算 RPM(对应文档 fan_get_rpm) */
void Fan_UpdateRpm(void);

/* 读当前转速(RPM) */
uint16_t Fan_GetRpm(void);

/* 读当前实际输出占空比(显示/调试用) */
uint8_t Fan_GetDuty(void);

#endif
