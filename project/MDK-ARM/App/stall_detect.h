#ifndef __STALL_DETECT_H__
#define __STALL_DETECT_H__

#include <stdint.h>

/* 堵转检测状态(文档《03》§4.1) */
typedef enum {
    STALL_NORMAL = 0,       /* 正常:未检测(占空比低或转速正常) */
    STALL_MONITORING,       /* 监测中:大占空比下 3s 窗口看脉冲 */
    STALL_FAULT,            /* 堵转:已停机,10s 后自动重试一次 */
    STALL_FAULT_SEVERE,     /* 重试再次堵转:停止自动重试,需手动干预 */
} stall_state_t;

/* 初始化:复位到正常态(上电/手动复位时调用) */
void StallDetect_Init(void);

/* 堵转检测节拍:每 100ms 调用一次(控制任务内)
   参数 fan_duty:期望的风机占空比(0~100,如 PID 输出)
   返回值:实际应输出的占空比——堵转判定后强制返回 0(停机) */
uint8_t StallDetect_Tick(uint8_t fan_duty);

/* 查询当前状态 */
stall_state_t StallDetect_GetState(void);

/* 是否处于堵转故障(STALL_FAULT / STALL_FAULT_SEVERE 为 1)→ 驱动蜂鸣器 */
uint8_t StallDetect_IsStalled(void);

/* 手动复位(严重故障后人工干预:清状态、允许重新检测) */
void StallDetect_Reset(void);

#endif
