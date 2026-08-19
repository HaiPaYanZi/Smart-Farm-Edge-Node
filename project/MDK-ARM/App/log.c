/**
  ******************************************************************************
  * @file    log.c
  * @brief   简易日志系统实现(app 层)
  *
  * 工作流程(log_output 内部):
  *   1. 级别过滤(运行时兜底,编译期已被宏过滤)
  *   2. 拿互斥锁 → 防止多任务日志穿插
  *   3. 格式化进本地缓冲:[tick毫秒][级别] 消息\r\n
  *   4. 一次性串口发送
  *   5. 释放互斥锁
  *
  * 依赖:usart.c 的 huart1(USART1 句柄)+ FreeRTOS 互斥锁
  ******************************************************************************
  */
#include "usart.h"      /* huart1: 日志输出使用的串口句柄 */
#include "cmsis_os.h"   /* osMutex 系列: 日志互斥锁 */
#include <stdio.h>      /* vsnprintf/snprintf: 格式化日志内容 */
#include <stdarg.h>     /* va_list: 处理可变参数 */
#include "log.h"
/* 互斥锁句柄: 多任务同时打日志时,保证每条日志完整输出、不互相穿插 */
static osMutexId_t log_mutex;

/* 级别 → 字母,与 log.h 枚举顺序一一对应: D=DEBUG I=INFO W=WARN E=ERROR */
static const char *const level_str[] = {"D", "I", "W", "E"};

/**
  * @brief  初始化日志系统
  * @note   必须在调度器启动前调用一次(建议放 MX_FREERTOS_Init 的 USER CODE 区)
  *         原因:FreeRTOS 对象(互斥锁)要在调度器启动前创建好
  *         失败返回 NULL(堆不足),log_output 里已做判空保护,不会崩溃
  */
void log_init(void)
{
  log_mutex = osMutexNew(NULL);//创建互斥锁
}

/**
  * @brief  日志输出核心函数
  * @param  level 日志级别,取 log.h 的枚举值(LOG_LEVEL_xxx)
  * @param  fmt   格式化字符串(printf 语法;MicroLIB 不支持 %f)
  * @note   业务代码请用 LOG_DEBUG/LOG_INFO/LOG_WARN/LOG_ERROR 宏
  *         不要在中打断里调用(内部是阻塞式串口发送,会卡死中断)
  */
void log_output(log_level_t level, const char *fmt, ...)
{
  char buf[128];        /* 单条日志缓冲(最长 127 字符) */
  int len = 0;          /* 当前已写入 buf 的字节数 */
  int n;                /* vsnprintf 返回值 */
  va_list ap;           /* 可变参数列表 */

  /* 第 1 步: 级别过滤(运行时兜底)。
     编译期的宏判断已过滤掉大多数调用,这里防的是绕过宏直接调 log_output */
  if (level < LOG_THRESHOLD)
  {
    return;
  }

  /* 防御: 调用方传了非法级别(大于枚举上限)时按 ERROR 处理,
     防止下面 level_str[level] 数组越界 */
  if (level > LOG_LEVEL_ERROR)
  {
    level = LOG_LEVEL_ERROR;
  }

  /* 第 2 步: 获取互斥锁。
     其他任务正在打印时,这里会阻塞等它打印完,再输出本条日志,
     保证一条日志在串口上是一段完整内容,不会被拆得七零八落 */
  if (log_mutex != NULL)
  {
    osMutexAcquire(log_mutex, osWaitForever);//获取互斥锁
  }

  /* 第 3 步: 格式化进缓冲。
     先写时间戳(系统 tick,单位 ms)+  */
  unsigned long  ms = (unsigned long)HAL_GetTick();
  len = snprintf(buf, sizeof(buf), "[%02lu:%02lu:%02lu][%s] ",
                 ms/3600000, (ms/60000)%60, (ms/1000)%60, level_str[level]);
  //snprintf 返回写入的字节数,不包含结尾的 \0
  /* 再拼接用户传入的格式化消息(变参展开) */
  va_start(ap, fmt); // 把 ap 指向"fmt 之后的第一个参数"
  n = vsnprintf(buf + len, sizeof(buf) - (size_t)len, fmt, ap);// 把 ap 里的一串参数按 fmt 格式化成字符串
  va_end(ap);//结束可变参数列表
  if (n < 0)
  {
    n = 0;              /* 格式化出错(格式串非法等)返回负值,按 0 处理 */
  }
  len += n;
  if (len >= (int)sizeof(buf))
  {
    len = (int)sizeof(buf) - 1;   /* 消息超长:截断,保证 buf[len] 永远合法 */
  }

  /* 第 4 步: 结尾补 \r\n,串口助手按行显示。
     注意先检查剩余空间(2 字节),空间不够就不加了 */
  if (len <= (int)sizeof(buf) - 2)
  {
    buf[len++] = '\r';
    buf[len++] = '\n';
  }
  buf[len] = '\0';      /* 保险: 补字符串结束符,方便调试器里查看 */

  /* 第 5 步: 整条消息一次性发送。
     不逐字符走 fputc,减少函数调用开销;
     超时 1000ms——128 字节 @115200 波特率只需约 11ms,绰绰有余 */
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 1000);

  /* 第 6 步: 释放互斥锁,放行其他任务的日志 */
  if (log_mutex != NULL)
  {
    osMutexRelease(log_mutex);//释放互斥锁
  }
}
