#ifndef __BSP_GPIO_H__
#define __BSP_GPIO_H__

#include <stdint.h>
#include "stm32f1xx_hal.h"

/* 板级输出口定义(芯片资源层:只提供"电平控制",不管用途;用途由 Divece 层决定)
   引脚来源:《04-引脚分配表》执行器组 */
#define GPIO_OUT_PUMP       GPIOB, GPIO_PIN_0    /* 水泵继电器 */
#define GPIO_OUT_BUZZER     GPIOB, GPIO_PIN_4    /* 蜂鸣器(报警) */

/* 初始化:输出口配为推挽输出,初始全部低电平(器件默认关,上电调一次) */
void GPIO_OutInit(void);

/* 输出电平:on=1 高电平 / on=0 低电平
   注:继电器模块有"低触发/高触发"跳线,默认按高电平触发吸合写,实物不符改这里 */
void GPIO_OutWrite(GPIO_TypeDef *port, uint16_t pin, uint8_t on);

/* 读引脚当前电平:1=高 0=低(显示"执行器实际状态"用——读的是引脚实际电平,
   手动模式线圈直接写执行器后,显示不会被规则引擎状态表误导) */
uint8_t GPIO_OutRead(GPIO_TypeDef *port, uint16_t pin);

#endif
