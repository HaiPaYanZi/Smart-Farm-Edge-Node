#include "light.h"
#include "bsp_pwm.h"

/*
  补光灯器件驱动(Divece 层)

  规则引擎只输出"开/关",补光开 = 80% 占空比(文档 §3.3 执行器输出映射)。
  亮度调节需求出现时,可在此扩展 Light_Control(percent) 类接口。
*/

#define LIGHT_DUTY_ON   80      /* 补光开启占空比(%) */

void Light_Control(uint8_t on)
{
    PWM_SetDuty(PWM_CH_LIGHT, on ? LIGHT_DUTY_ON : 0);
}

/* 读实际状态:占空比大于 0 就认为灯是亮的(读 PWM 实际输出,不维护软件状态) */
uint8_t Light_IsOn(void)
{
    return (PWM_GetDuty(PWM_CH_LIGHT) > 0) ? 1 : 0;
}
