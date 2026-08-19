#include "fan.h"
#include "bsp_pwm.h"
#include "bsp_hal_counter.h"

/*
  风扇器件驱动(Divece 层)

  器件层职责:知道"风扇这个器件"怎么用(转速 = PWM 占空比),
  不知道也不关心 TIM3 的寄存器细节(那是 BSP 层的事)。
  PID 算出的目标转速(0~100)直接喂给 Fan_Control 即可。

  转速换算(器件特性):磁铁 1 个极对 → 每圈 1 个霍尔脉冲。
  Fan_UpdateRpm 每 500ms 调一次(采集任务里):
    delta = 窗口内脉冲数
    RPM   = delta × 2(每秒脉冲)× 60(每分钟)= delta × 120
*/

/* 差分换算用的上次累计值 + 当前转速/占空比缓存 */
static uint32_t last_pulses = 0;
static uint16_t rpm = 0;
static uint8_t cur_duty = 0;

void Fan_Control(uint8_t percent)
{
    /* 占空比 = 转速:0% 停,100% 全速 */
    cur_duty = percent;                 /* 记录实际输出(显示/调试用) */
    PWM_SetDuty(PWM_CH_FAN, percent);
    /* TODO:DRV8833 方向引脚控制(风机需反转时),引脚待确认后补 */
}

/* 转速更新:500ms 窗口差分 → RPM */
void Fan_UpdateRpm(void)
{
    uint32_t now = HAL_Counter_GetPulses();   /* 当前累计脉冲 */
    uint32_t delta = now - last_pulses;       /* 窗口内新脉冲数(无符号回绕也安全) */
    last_pulses = now;

    rpm = (uint16_t)(delta * 120);            /* ×2(每秒)×60(每分)= ×120 */
}

/* 读当前转速 */
uint16_t Fan_GetRpm(void)
{
    return rpm;
}

/* 读当前实际输出占空比 */
uint8_t Fan_GetDuty(void)
{
    return cur_duty;
}
