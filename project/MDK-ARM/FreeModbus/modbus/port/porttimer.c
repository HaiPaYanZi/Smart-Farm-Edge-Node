/*
 * FreeModbus 移植层:porttimer.c(TIM2 做 3.5T 字符超时)
 *
 * 原理:RTU 规定字符间隔超过 3.5T(9600bps 下 ≈4ms)判定"帧结束"。
 *   协议栈每收到一个字节自动调 vMBPortTimersEnable(清零重启);
 *   4ms 内没新字节 → TIM2 超时中断 → pxMBPortCBTimerExpired() 通知协议栈收完一帧。
 *
 * 核心点(时间换算):
 *   定时器时钟 72MHz(APB1=36MHz×2)
 *   PSC=71 → 1MHz(1 tick = 1μs)
 *   ARR = timeout_50us × 50:协议栈把 3.5T 换算成 50μs 单位传入
 *   例:9600bps → 3.5T≈4ms → 协议栈传 80 → ARR=4000 → 4000μs 超时
 *
 * 中断路径:it.c 的 TIM2_IRQHandler → HAL_TIM_IRQHandler → 本文件的
 *   HAL_TIM_PeriodElapsedCallback(weak 回调,无需改 it.c)
 */

#include "port.h"      /* 必须最先:定义 BOOL/UCHAR 等类型 */
#include "mb.h"
#include "mbport.h"
#include "tim.h"

static USHORT timeout_50us;   /* 超时值,单位 50μs(协议栈 eMBInit 时按波特率算好传入) */

/**
  * 函    数：xMBPortTimersInit
  * 功    能：定时器初始化(协议栈启动时调用一次)
  * 参    数：timeout 超时值,单位 50μs(协议栈自动算出,如 9600→80)
  * 返 回 值：TRUE 成功
  */
BOOL xMBPortTimersInit(USHORT timeout)
{
    timeout_50us = timeout;                                      /* 记住超时值 */

    __HAL_TIM_SET_PRESCALER(&htim2, 71);                             /* 72MHz/72 = 1MHz = 1μs/tick */
    __HAL_TIM_SET_AUTORELOAD(&htim2, (uint32_t)timeout_50us * 50);   /* 超时值换算成 μs 计数 */
    __HAL_TIM_SET_COUNTER(&htim2, 0);                                /* 计数清零 */
    return TRUE;
}

/**
  * 函    数：vMBPortTimersEnable
  * 功    能：启动超时(协议栈每收到一个字节就调一次 = 清零重启)
  * 参    数：无
  * 返 回 值：无
  * 说    明：字节一个接一个来,定时器一直被清零,永远走不到头;
  *           帧发完了没人撞它,它走到 4ms → 中断 = 帧结束
  */
void vMBPortTimersEnable(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);            /* 清零,从头计时 */
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);  /* 开更新中断 */
    __HAL_TIM_ENABLE(&htim2);                    /* 启动计数 */
}

/**
  * 函    数：vMBPortTimersDisable
  * 功    能：停止超时(帧处理完/超时已触发时调用)
  * 参    数：无
  * 返 回 值：无
  */
void vMBPortTimersDisable(void)
{
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);  /* 关更新中断 */
    __HAL_TIM_DISABLE(&htim2);                    /* 停止计数 */
}

/**
  * 函    数：vMBPortTimersDelay
  * 功    能：延时(仅 ASCII 模式使用,本项目 RTU 不需要)
  * 参    数：delay_ms 延时毫秒
  * 返 回 值：无
  */
void vMBPortTimersDelay(USHORT delay_ms)
{
    (void)delay_ms;                              /* 空实现 */
}

/* HAL_TIM_PeriodElapsedCallback 在 main.c 里统一实现(全工程只能有一份,
   main.c 里已有 TIM4 做 HAL 系统心跳)。TIM2 超时分支已合并进 main.c 的
   USER CODE Callback 0 区域——改这里时记得两边同步。 */
