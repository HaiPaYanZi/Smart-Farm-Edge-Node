#ifndef __SD_LOG_H__
#define __SD_LOG_H__

#include <stdint.h>

/* ==================== 状态字位定义(CSV "状态字"列) ====================
   文档《03》§6.2 样例:状态字 0x0020 表示 FAN_STALL(风机堵转),
   故 bit5 固定为堵转位,其余位按系统现有状态补充。
   状态字由 sd_log.c 内部在每条快照行时实时合成,业务代码不用管 */
#define STW_MANUAL     0x0001   /* bit0:手动模式(Modbus 写 40005 切换) */
#define STW_STANDBY    0x0002   /* bit1:待机模式 */
#define STW_TEMP_FAIL  0x0004   /* bit2:温度传感器失效(读回 SENSOR_INVALID) */
#define STW_LIGHT_ON   0x0008   /* bit3:补光灯当前实际输出为开 */
#define STW_PUMP_ON    0x0010   /* bit4:水泵当前实际输出为开 */
#define STW_FAN_STALL  0x0020   /* bit5:风机堵转报警中(对齐文档样例位) */
#define STW_STALL_SEV  0x0040   /* bit6:严重堵转(10s 重试仍堵转,需人工干预) */

/* 上电初始化:创建日志队列与事件互斥锁(必须在调度器启动前调用,
   建议与 log_init() 并排放 MX_FREERTOS_Init 的 USER CODE Init 区) */
void SDLog_Init(void);

/* 推一条传感器快照 CSV 行进队列(采集任务每 500ms 调用一次,对应 F12 用例
   "每采集周期一条记录")。队列满 16 条时丢弃本条,历史数据优先保留 */
void SDLog_Snapshot(void);

/* 登记一条事件:内容会填进"下一条快照行"的事件列(消费后自动清空)。
   事件名用英文大写(如 "FAN_STALL"),控制任务在状态跳变时调用 */
void SDLog_Event(const char *event_name);

/* 日志落盘任务:挂载 SD 卡 → 攒批写 CSV 文件 → 超限滚动新建。
   作为自创建任务运行(创建代码放 freertos.c 的 RTOS_THREADS USER CODE 区) */
void SDLog_Task(void *argument);

#endif /* __SD_LOG_H__ */
