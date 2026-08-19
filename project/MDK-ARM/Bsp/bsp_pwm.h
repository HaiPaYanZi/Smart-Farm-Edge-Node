#ifndef __BSP_PWM_H__
#define __BSP_PWM_H__

#include <stdint.h>

/* PWM 通道号(自定义编号,与 HAL 解耦——头文件不依赖 HAL 库,
   外部只知道"风机=0、补光=1",TIM 通道细节在 bsp_pwm.c 内部映射) */
#define PWM_CH_FAN      0   /* 风机 → TIM3_CH1(PA6) */
#define PWM_CH_LIGHT    1   /* 补光 → TIM3_CH2(PA7) */

/* PWM 初始化:启动 TIM3 两通道,初始占空比 0(上电调用一次) */
void PWM_Init(void);

/* 设置指定通道占空比:percent 0~100(超界自动钳位) */
void PWM_SetDuty(uint32_t channel, uint8_t percent);

/* 读指定通道当前占空比:%(从比较寄存器反算,显示"执行器实际状态"用) */
uint8_t PWM_GetDuty(uint32_t channel);

#endif
