/*
 * FreeModbus 移植层:port.h
 * 平台:STM32F103 + FreeRTOS
 *
 * 职责:① 给协议栈提供类型别名(BOOL/UCHAR 等,换平台只改这里)
 *      ② 定义临界区宏(协议栈内部共享数据保护用)
 *
 * 核心点:临界区用 taskENTER_CRITICAL(挂调度器,不关中断),
 *         中断上下文的事件发布由 portevent.c 用 ISR 版宏单独处理。
 */
#ifndef _PORT_H
#define _PORT_H

#include <stdint.h>
#include <assert.h>     /* 断言:协议栈内部用 assert 检查状态机合法性 */
#include "FreeRTOS.h"
#include "task.h"

#define INLINE                      inline
#define PR_BEGIN_EXTERN_C           extern "C" {
#define PR_END_EXTERN_C             }

/* 任务级临界区:挂起调度器;中断照常响应(比关中断友好) */
#define ENTER_CRITICAL_SECTION( )   taskENTER_CRITICAL( )
#define EXIT_CRITICAL_SECTION( )    taskEXIT_CRITICAL( )

/* 类型别名:协议栈内部统一用这些名字 */
typedef uint8_t   BOOL;
typedef unsigned char UCHAR;
typedef char      CHAR;
typedef uint16_t  USHORT;
typedef int16_t   SHORT;
typedef uint32_t  ULONG;
/* LONG 必须与 FatFs integer.h 的 typedef long LONG 类型完全一致,
   否则两者被同一 .c 包含时会 #256 重定义报错(ARM 上 int32_t 是 int,与 long 算不同类型) */
typedef long      LONG;

#ifndef TRUE
#define TRUE            1
#endif
#ifndef FALSE
#define FALSE           0
#endif

#endif
