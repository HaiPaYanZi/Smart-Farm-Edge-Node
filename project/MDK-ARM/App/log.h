/**
  ******************************************************************************
  * @file    log.h
  * @brief   简易日志系统(app 层)—— 分级打印到串口(USART1)
  *
  * ==================== 使用示例 ====================
  * 任务代码里这样用:
  *   LOG_DEBUG("ADC 值 = %d", adc_val);         // 调试:开发期看细节
  *   LOG_INFO("传感器初始化完成");                // 信息:正常流程节点
  *   LOG_WARN("传感器读取失败,重试中");           // 警告:异常但可恢复
  *   LOG_ERROR("参数校验失败, code=%d", code);   // 错误:需要立刻关注
  *
  * 发布时:把 LOG_THRESHOLD 改成 LOG_LEVEL_WARN
  *   → DEBUG/INFO 的日志调用代码在编译期被整体消除,不占 Flash、不耗 CPU
  * ================================================
  *
  * 设计要点:
  *   1. 四个级别,数值越大越严重: DEBUG(0) < INFO(1) < WARN(2) < ERROR(3)
  *   2. 编译期阈值裁剪:宏里带常量判断,低于阈值的调用不生成代码
  *   3. 多任务安全:log_output 内部用互斥锁,日志不会互相穿插
  *   4. 注意:Keil MicroLIB 不支持 %f,浮点数请用整数/定点数打印
  ******************************************************************************
  */
#ifndef __LOG_H__
#define __LOG_H__

#include <stdint.h>   /* 标准整数类型(方便业务代码使用) */
#include <stdarg.h>   /* 可变参数(va_list),log_output 声明需要 */

/* ==================== 日志级别 ====================
   过滤规则: level >= LOG_THRESHOLD 才输出
     THRESHOLD = DEBUG → 全部输出(开发期推荐)
     THRESHOLD = WARN  → 只留 WARN 和 ERROR(发布期推荐)
     THRESHOLD = ERROR → 只留 ERROR(仅保留严重错误) */
typedef enum {
    LOG_LEVEL_DEBUG = 0,   /* 调试:详细过程信息,仅开发期使用 */
    LOG_LEVEL_INFO  = 1,   /* 信息:正常流程的节点记录 */
    LOG_LEVEL_WARN  = 2,   /* 警告:异常但可恢复(传感器一次失败、SD 写失败) */
    LOG_LEVEL_ERROR = 3,   /* 错误:需要关注(堵转、参数损坏) */
} log_level_t;

/* ==================== 编译期阈值 ====================
   开发期保持 DEBUG,发布时改成 WARN。
   低于阈值的日志调用整段不编译(由下方宏里的常量判断实现),
   而不是运行时判断 —— 这就是"不占 Flash"的实现原理。 */
#ifndef LOG_THRESHOLD
#define LOG_THRESHOLD LOG_LEVEL_DEBUG
#endif

/* ==================== 核心函数(业务代码不直接调用) ==================== */
void log_init(void);                                    /* 初始化,调度器启动前调用一次 */
void log_output(log_level_t level, const char *fmt, ...); /* 输出一条日志 */

/* ==================== 调用宏(业务代码只用这四个) ====================
   do{...}while(0) 写法:让宏像普通函数一样用,放在 if 后面也不会出错
   if(常量判断) 在编译期即可确定:
     - 满足 → 保留 log_output 调用
     - 不满足 → 整块被编译器消除,不占 Flash */
    //__VA_ARGS__ 可变参数宏,替换为实际传入的参数列表
#define LOG_DEBUG(...) do { if (LOG_LEVEL_DEBUG >= LOG_THRESHOLD) { log_output(LOG_LEVEL_DEBUG, __VA_ARGS__); } } while(0)
#define LOG_INFO(...)  do { if (LOG_LEVEL_INFO  >= LOG_THRESHOLD) { log_output(LOG_LEVEL_INFO,  __VA_ARGS__); } } while(0)
#define LOG_WARN(...)  do { if (LOG_LEVEL_WARN  >= LOG_THRESHOLD) { log_output(LOG_LEVEL_WARN,  __VA_ARGS__); } } while(0)
#define LOG_ERROR(...) do { if (LOG_LEVEL_ERROR >= LOG_THRESHOLD) { log_output(LOG_LEVEL_ERROR, __VA_ARGS__); } } while(0)

#endif /* __LOG_H__ */
