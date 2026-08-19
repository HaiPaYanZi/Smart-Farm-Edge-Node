#ifndef __OLED_PAGE_H__
#define __OLED_PAGE_H__

#include <stdint.h>   /* uint8_t 定义 */

/* OLED 三页界面框架(1.1 显示任务用)
   页 1 传感器页:温度/湿度/光照/土壤
   页 2 执行器页:风机占空比转速/补光/水泵/堵转
   页 3 系统页:  模式/运行时间/故障/版本
   切换:KEY1(PC9)按下 → 下一页(循环),防抖 300ms */

/* 刷新当前页:500ms 调一次(读快照写显存 + OLED_Update)
   注意:由显示任务持 I2C 锁时调用 */
void OLED_Page_Show(void);

/* 按键轮询:100ms 调一次(防抖切页,不碰 I2C) */
void OLED_Page_KeyScan(void);

/* 直接切到指定页(0~2,调试命令用;越界忽略) */
void OLED_Page_JumpTo(uint8_t p);

#endif
