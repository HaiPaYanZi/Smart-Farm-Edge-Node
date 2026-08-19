#ifndef __BOOT_PROTO_H__
#define __BOOT_PROTO_H__

#include <stdint.h>
#include "ota_common.h"     /* 帧格式/命令/分区三方共享定义 */

/* 进入升级模式:初始化协议状态机,等待 PC 工具握手。
   active_part:当前活动分区 'A' 或 'B',升级目标自动取对侧分区 */
void boot_proto_start(uint8_t active_part);

/* 协议主循环:每圈调用一次(主循环里),内部做三件事:
   1. 从串口环形缓冲解析帧(字节流状态机)
   2. 帧 CRC 校验通过后按命令分发处理
   3. 检查 5s 收包超时 → 回滚原分区并软复位(文档《07》§3.3) */
void boot_proto_poll(void);

/* 协议模块引用的毫秒节拍(由 main.c 的 SysTick 中断累加):
   超时判断与应答重传都用它 */
extern volatile uint32_t g_tick_ms;

/* 读升级标志区(0x08003000)当前内容到结构体(Flash 随机读) */
void boot_flag_read(ota_flag_t *flag);

/* 写升级标志区:擦页+编程(掉电保持)。调用方先读旧值改字段再写回 */
uint8_t boot_flag_write(const ota_flag_t *flag);

#endif /* __BOOT_PROTO_H__ */
