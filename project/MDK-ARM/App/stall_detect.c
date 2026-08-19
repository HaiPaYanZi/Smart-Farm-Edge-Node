#include "stall_detect.h"
#include "bsp_hal_counter.h"

/*
  堵转检测(文档《03》§4.1,故障管理的一部分)

  状态机:
    NORMAL ── 输出占空比>30% ──→ MONITORING(启动 3s 检测窗口)
    MONITORING ── 窗口内霍尔脉冲 ≥5 ──→ NORMAL(转起来了,解除)
    MONITORING ── 3s 内脉冲 <5 ──→ FAULT(堵转:停机+报警)
    FAULT ── 10s 后自动重试 ──→ MONITORING(重新观察)
    FAULT ── 重试再堵转 ──→ FAULT_SEVERE(停止自动重试,人工干预)
    任一状态 ── 目标占空比 ≤30% ──→ NORMAL(不需要担心堵转)

  每 100ms 调用一次(控制任务),用霍尔累计脉冲做窗口差分判断,
  不依赖转速换算——脉冲计数在 ISR 里累加,不丢数。
*/

/* 参数(§4.1) */
#define STALL_DUTY_THRESHOLD    30      /* 占空比>30% 才启动检测 */
#define STALL_WINDOW_TICKS      30      /* 3s 检测窗口(100ms/tick) */
#define STALL_MIN_PULSES        5       /* 窗口内最少脉冲数 */
#define STALL_RETRY_TICKS       100     /* 10s 后自动重试 */

static stall_state_t state = STALL_NORMAL;
static uint32_t win_start_pulses;       /* 检测窗口起始的累计脉冲 */
static uint16_t tick_cnt;               /* 当前状态下的 tick 计数 */
static uint8_t retried;                 /* 是否已重试过一次 */
static uint8_t out_duty;                /* 实际输出占空比缓存 */

/* 初始化 */
void StallDetect_Init(void)
{
    state = STALL_NORMAL;
    tick_cnt = 0;
    retried = 0;
    out_duty = 0;
}

/* 堵转检测节拍:返回实际应输出的占空比 */
uint8_t StallDetect_Tick(uint8_t fan_duty)
{
    switch (state)
    {
        case STALL_NORMAL:
            if (fan_duty > STALL_DUTY_THRESHOLD)
            {
                /* 大占空比才担心堵转:启动检测窗口,记脉冲起点 */
                state = STALL_MONITORING;
                win_start_pulses = HAL_Counter_GetPulses();
                tick_cnt = 0;
            }
            out_duty = fan_duty;
            break;

        case STALL_MONITORING:
            if (fan_duty <= STALL_DUTY_THRESHOLD)
            {
                /* 目标降到阈值以下:无需检测,回正常 */
                state = STALL_NORMAL;
                out_duty = fan_duty;
                break;
            }
            tick_cnt++;
            if (HAL_Counter_GetPulses() - win_start_pulses >= STALL_MIN_PULSES)
            {
                /* 窗口内脉冲够 → 风扇转起来了,解除监测 */
                state = STALL_NORMAL;
                out_duty = fan_duty;
            }
            else if (tick_cnt >= STALL_WINDOW_TICKS)
            {
                /* 3s 窗口内脉冲不足 → 堵转:停机,按是否重试过分级 */
                state = retried ? STALL_FAULT_SEVERE : STALL_FAULT;
                tick_cnt = 0;
                out_duty = 0;
            }
            else
            {
                out_duty = fan_duty;    /* 还在观察期,照常输出 */
            }
            break;

        case STALL_FAULT:
            tick_cnt++;
            if (tick_cnt >= STALL_RETRY_TICKS)
            {
                /* 10s 后自动重试:标记已重试,重新进入观察 */
                retried = 1;
                state = STALL_MONITORING;
                win_start_pulses = HAL_Counter_GetPulses();
                tick_cnt = 0;
            }
            out_duty = 0;               /* 故障期间保持停机 */
            break;

        case STALL_FAULT_SEVERE:
            out_duty = 0;               /* 严重:永久停机,等手动复位 */
            break;
    }

    return out_duty;
}

/* 查询状态 */
stall_state_t StallDetect_GetState(void)
{
    return state;
}

/* 是否堵转故障(驱动蜂鸣器/状态字) */
uint8_t StallDetect_IsStalled(void)
{
    return (state == STALL_FAULT || state == STALL_FAULT_SEVERE) ? 1 : 0;
}

/* 手动复位(严重故障后人工干预) */
void StallDetect_Reset(void)
{
    StallDetect_Init();
}
