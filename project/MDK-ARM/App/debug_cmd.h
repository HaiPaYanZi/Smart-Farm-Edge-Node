#ifndef __DEBUG_CMD_H__
#define __DEBUG_CMD_H__

/* 串口调试命令(1.1 调试口交互)
   命令集(回车/换行结尾):
     help                    列出命令
     temp / humi / light / soil   查询传感器快照
     fan <0-100>             设风机目标占空比
     rpm                     查当前转速
     pump on|off             手动开关水泵(规则引擎下次评估会覆盖,仅调试)
     light on|off            手动开关补光(同上)
     stall                   查堵转状态
     stall-reset             堵转手动复位
     page <0-2>              切 OLED 页 */

/* 启动 USART1 中断接收(初始化时调用一次) */
void DebugCmd_Init(void);

/* 命令处理任务体(独立任务,低优先级) */
void DebugCmd_Task(void *argument);

#endif
