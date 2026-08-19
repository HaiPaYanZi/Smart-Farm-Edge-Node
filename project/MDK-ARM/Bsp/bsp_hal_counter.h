#ifndef __BSP_HAL_COUNTER_H__
#define __BSP_HAL_COUNTER_H__

#include <stdint.h>

/* 霍尔脉冲计数(BSP 层:芯片资源封装)
   引脚:PB1(EXTI1)上升沿中断计数,CubeMX 已配置(NVIC 已使能)
   职责:只管"数脉冲",不知道也不关心脉冲代表什么转速——换算在器件层(fan.c) */

/* 初始化:清零累计脉冲(上电调一次) */
void HAL_Counter_Init(void);

/* 读累计脉冲数(ISR 累加,任务侧做差分换算) */
uint32_t HAL_Counter_GetPulses(void);

/* 清零累计脉冲 */
void HAL_Counter_Reset(void);

#endif
