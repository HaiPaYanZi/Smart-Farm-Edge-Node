#include "bsp_pwm.h"
#include "tim.h"

/*
  板级 PWM 封装(风机/补光共用 TIM3 双通道)

  TIM3 配置(CubeMX 已生成):72MHz / (3+1) / (899+1) = 20kHz
  通道号映射:PWM_CH_FAN(0) → TIM_CHANNEL_1, PWM_CH_LIGHT(1) → TIM_CHANNEL_2

  占空比换算:
    定时器计数 0~899,比较值 = 900 * percent / 100
    例如 50% → 比较值 450 → 一半周期输出高电平
*/

/* 通道号 → HAL 通道映射(唯一的硬件相关点,换通道只改这里) */
static uint32_t channel_map(uint32_t channel)
{
    return (channel == PWM_CH_FAN) ? TIM_CHANNEL_1 : TIM_CHANNEL_2;
}

/* PWM 初始化:启动两通道,占空比默认 0(配置已设 Pulse=0) */
void PWM_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

/* 设置占空比:percent 0~100,换算成比较值写入对应通道 */
void PWM_SetDuty(uint32_t channel, uint8_t percent)
{
    uint16_t compare;

    if (percent > 100)
    {
        percent = 100;              /* 超界钳位 */
    }

    /* 比较值 = 周期 × 占空比(Period=899,故 900 × percent / 100) */
    compare = (uint16_t)((htim3.Init.Period + 1) * percent / 100);

    __HAL_TIM_SET_COMPARE(&htim3, channel_map(channel), compare);
}

/* 读当前占空比:把比较寄存器的值反算回百分比 */
uint8_t PWM_GetDuty(uint32_t channel)
{
    uint16_t compare = __HAL_TIM_GET_COMPARE(&htim3, channel_map(channel));   /* 读比较寄存器 */
    uint32_t percent = (uint32_t)compare * 100 / (htim3.Init.Period + 1);     /* 反算:比较值/周期×100 */
    return (uint8_t)percent;
}
