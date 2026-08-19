/*
 * FreeModbus 移植层:portevent.c(事件队列)
 *
 * 职责:协议栈与硬件中断之间的"信件投递"。
 *   串口 ISR 收完帧/发完帧 → vMBPortEventPost 投信(事件)
 *   Modbus 任务 eMBPoll → xMBPortEventGet 取信处理
 *
 * 核心点(并发安全,两种上下文用两种临界区):
 *   - vMBPortEventPost 在 ISR 里调用 → 必须用 FromISR 版临界区
 *     (portSET_INTERRUPT_MASK_FROM_ISR 保存+关中断+恢复,支持嵌套)
 *     taskENTER_CRITICAL 在 ISR 里用会触发 FreeRTOS 断言,不可用!
 *   - xMBPortEventGet 在任务里调用 → taskENTER_CRITICAL 即可(挂调度器)
 *
 * 队列满:丢弃新事件返回 FALSE,协议栈当无事发生,等下一轮。
 */

#include "port.h"      /* 必须最先:定义 BOOL/UCHAR 等类型 */
#include "mb.h"
#include "mbport.h"

#define QUEUE_SIZE  4     /* 队列深度:从机一次处理一帧,4 足够 */

static eMBEventType queue[QUEUE_SIZE];   /* 事件队列数组(环形使用) */
static uint8_t read_index = 0;           /* 读指针:下一个要取的位置 */
static uint8_t write_index = 0;          /* 写指针:下一个要放的位置 */
static uint8_t count = 0;                /* 队列里现有的事件数 */

/**
  * 函    数：xMBPortEventInit
  * 功    能：事件队列初始化(清空)
  * 参    数：无
  * 返 回 值：TRUE 永远成功
  */
BOOL xMBPortEventInit(void)
{
    read_index = 0;      /* 读指针归零 */
    write_index = 0;     /* 写指针归零 */
    count = 0;           /* 事件数清零 */
    return TRUE;
}

/**
  * 函    数：xMBPortEventPost
  * 功    能：发布一个事件(串口/定时器中断里调用)
  * 参    数：event 事件类型(EV_READY 帧收完 / EV_FRAME_SENT 帧发完 / EV_ERROR 错误)
  * 返 回 值：TRUE 投递成功 / FALSE 队列满丢弃
  */
BOOL xMBPortEventPost(eMBEventType event)
{
    BOOL ok = FALSE;                                                   /* 默认失败 */
    UBaseType_t saved = portSET_INTERRUPT_MASK_FROM_ISR();             /* ISR 版临界区:保存中断状态并关中断 */

    if (count < QUEUE_SIZE)                                            /* 队列没满才收 */
    {
        queue[write_index] = event;                                    /* 事件放入队尾 */
        write_index = (write_index + 1) % QUEUE_SIZE;                  /* 写指针前进,到头绕回 */
        count++;                                                       /* 事件数 +1 */
        ok = TRUE;                                                     /* 投递成功 */
    }
    /* 队列满:丢弃新事件(从机单帧处理,不会发生) */

    portCLEAR_INTERRUPT_MASK_FROM_ISR(saved);                          /* 恢复之前的中断状态 */
    return ok;
}

/**
  * 函    数：xMBPortEventGet
  * 功    能：取一个事件(Modbus 任务里调用,取不到返回 FALSE)
  * 参    数：event 出参,取到的事件类型
  * 返 回 值：TRUE 取到 / FALSE 队列空
  */
BOOL xMBPortEventGet(eMBEventType *event)
{
    BOOL ok = FALSE;                                                   /* 默认失败 */

    taskENTER_CRITICAL();                                              /* 任务级临界区:防 ISR 同时写队列 */
    if (count > 0)                                                     /* 队列里有事件才取 */
    {
        *event = queue[read_index];                                    /* 取队头事件 */
        read_index = (read_index + 1) % QUEUE_SIZE;                    /* 读指针前进,到头绕回 */
        count--;                                                       /* 事件数 -1 */
        ok = TRUE;                                                     /* 取成功 */
    }
    taskEXIT_CRITICAL();                                               /* 恢复调度 */

    return ok;
}
