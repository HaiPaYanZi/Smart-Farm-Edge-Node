#include "debug_cmd.h"
#include "log.h"
#include "usart.h"
#include "MySensor.h"
#include "rule_engine.h"
#include "fan.h"
#include "pump.h"
#include "light.h"
#include "stall_detect.h"
#include "oled_page.h"
#include "param.h"
#include "PID.h"
#include "ota.h"
#include "version.h"
#include <string.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"   /* osDelay 等 CMSIS-RTOS2 API */

/*
  串口调试命令(App 层)

  结构:USART1 中断收字节 → 缓冲攒整行(遇 \n 置就绪标志)
       → 命令任务取行解析执行 → LOG_INFO 回显
  共享数据保护:rx_buf/rx_len 被 ISR 写、任务读,取行时进临界区拷贝,
      避免任务处理期间新字节插入导致半行错乱。

  注意:%f 打印依赖 log 系统的串口输出(当前未开 MicroLIB,可用);
      手动 pump/light 会被规则引擎 500ms 评估覆盖——正式手动模式在 1.6 Modbus 实现。
*/

#define RX_BUF_SIZE  64                /* 命令行长上限 */

/* 接收缓冲(ISR 写,任务读,临界区保护) */
static uint8_t rx_buf[RX_BUF_SIZE];
static uint16_t rx_len = 0;
static uint8_t rx_byte;         /* 接收单字节缓存(非 volatile:HAL 回调机制已保证可见性) */
static volatile uint8_t cmd_ready = 0;
static volatile uint32_t last_rx_tick = 0;  /* 最后收到字节的时刻(tick):无换行超时判帧用 */

extern volatile uint8_t g_fan_duty;    /* 风机目标占空比(freertos.c 定义) */

/* 启动中断接收 */
void DebugCmd_Init(void)
{
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);   /* 中断模式收单字节 */
}

/* 接收完成回调(每字节一次):攒行,遇换行置就绪 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (rx_len < RX_BUF_SIZE - 1)
        {
            rx_buf[rx_len] = rx_byte;
            rx_len++;
            last_rx_tick = osKernelGetTickCount();  /* 记收字节时刻,任务侧判超时 */
        }
        if (rx_byte == '\n' || rx_byte == '\r' || rx_len >= RX_BUF_SIZE - 1)
        {
            cmd_ready = 1;                      /* 整行就绪,任务侧处理 */
        }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);  /* 继续收下一个字节 */
    }
}

/* 命令执行(纯逻辑,可读快照/查状态/操作执行器) */
static void cmd_exec(char *cmd)
{
    if (strcmp(cmd, "help") == 0)
    {
        LOG_INFO("cmds:help temp humi light soil fan<n> rpm pump<on/off> light<on/off> stall stall-reset page<n> pid pidwatch mode<n> ota version");
    }
    else if (strcmp(cmd, "pid") == 0)
    {
        /* pid:查询 PID 当前参数与运行状态(整定观测用) */
        LOG_INFO("PID: Kp=%.2f Ki=%.2f Kd=%.2f target=%.1f C", Param_GetKp(), Param_GetKi(), Param_GetKd(), Param_GetTarget());
        LOG_INFO("     temp=%.1f C out=%.0f%% int=%.0f mode=%d", MySensor_GetValue(SENSOR_TEMP), g_pid.Out, g_pid.ErrorInt, Param_GetMode());
    }
    else if (strncmp(cmd, "pid ", 4) == 0)
    {
        /* pid <kp> <ki> <kd>:远程整定(与 Modbus 40001~40003 同效果,走参数区) */
        float kp_gain = (float)atof(cmd + 4);              /* 第一个参数:比例系数 */
        char *first_space = strchr(cmd + 4, ' ');          /* 第一个空格位置(参数分隔符) */
        float ki_gain = first_space ? (float)atof(first_space + 1) : 0.0f;   /* 第二个参数:积分系数 */
        char *second_space = first_space ? strchr(first_space + 1, ' ') : NULL; /* 第二个空格位置 */
        float kd_gain = second_space ? (float)atof(second_space + 1) : 0.0f;  /* 第三个参数:微分系数 */
        Param_SetKp(kp_gain); Param_SetKi(ki_gain); Param_SetKd(kd_gain);    /* 写入参数区,控制任务下周期加载 */
        LOG_INFO("pid -> Kp=%.2f Ki=%.2f Kd=%.2f (next 100ms loaded)", kp_gain, ki_gain, kd_gain);
    }
    else if (strcmp(cmd, "pidwatch") == 0)
    {
        /* pidwatch:PID 连续观测,每 500ms 打一行共 20 行(10 秒)。
           观察 temp(输入) → out(占空比输出)的响应:哈气加温/拿走传感器都能看到变化 */
        for (int i = 0; i < 20; i++)
        {
            LOG_INFO("watch %02d: temp=%.1f out=%.0f%% int=%.0f", i, MySensor_GetValue(SENSOR_TEMP), g_pid.Out, g_pid.ErrorInt);
            osDelay(500);
        }
    }
    else if (strncmp(cmd, "mode ", 5) == 0)
    {
        int mode_value = atoi(cmd + 5);                    /* 目标模式:0自动 1手动 2待机 */
        if (mode_value >= 0 && mode_value <= 2)            /* 范围检查:越界忽略 */
        {
            Param_SetMode((uint8_t)mode_value);            /* 写参数区,控制任务下拍生效 */
            LOG_INFO("mode -> %d (0=auto 1=manual 2=standby)", mode_value);
        }
    }
    else if (strcmp(cmd, "temp") == 0)
    {
        LOG_INFO("T: %.1f C", MySensor_GetValue(SENSOR_TEMP));
    }
    else if (strcmp(cmd, "humi") == 0)
    {
        LOG_INFO("H: %.1f %%", MySensor_GetValue(SENSOR_HUMI));
    }
    else if (strcmp(cmd, "light") == 0)
    {
        LOG_INFO("L: %.0f lx", MySensor_GetValue(SENSOR_LIGHT));
    }
    else if (strcmp(cmd, "soil") == 0)
    {
        LOG_INFO("S: %.1f %%", MySensor_GetValue(SENSOR_SOIL));
    }
    else if (strncmp(cmd, "fan ", 4) == 0)
    {
        int v = atoi(cmd + 4);
        if (v < 0)   v = 0;
        if (v > 100) v = 100;
        g_fan_duty = (uint8_t)v;
        LOG_INFO("fan duty -> %d%%", v);
    }
    else if (strcmp(cmd, "rpm") == 0)
    {
        LOG_INFO("rpm: %d", Fan_GetRpm());
    }
    else if (strcmp(cmd, "pump on") == 0)
    {
        Pump_Control(1);
        LOG_INFO("pump on (temp, rule will override)");
    }
    else if (strcmp(cmd, "pump off") == 0)
    {
        Pump_Control(0);
        LOG_INFO("pump off (temp)");
    }
    else if (strcmp(cmd, "light on") == 0)
    {
        Light_Control(1);
        LOG_INFO("light on (temp)");
    }
    else if (strcmp(cmd, "light off") == 0)
    {
        Light_Control(0);
        LOG_INFO("light off (temp)");
    }
    else if (strcmp(cmd, "stall") == 0)
    {
        LOG_INFO("stall state: %d (0=ok 1=check 2=fault 3=severe)", StallDetect_GetState());
    }
    else if (strcmp(cmd, "stall-reset") == 0)
    {
        StallDetect_Reset();
        LOG_INFO("stall reset");
    }
    else if (strncmp(cmd, "page ", 5) == 0)
    {
        int p = atoi(cmd + 5);
        if (p >= 0 && p <= 2)
        {
            OLED_Page_JumpTo((uint8_t)p);
            LOG_INFO("page -> %d", p);
        }
    }
    else if (strcmp(cmd, "version") == 0)
    {
        /* version:打印固件版本(与固件头 16B 里的一致) */
        LOG_INFO("app version %s (major=%d minor=%d patch=%d)",
                 APP_VERSION_STR, APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_VERSION_PATCH);
    }
    else if (strcmp(cmd, "ota") == 0)
    {
        /* ota:触发升级(写标志区后 200ms 软复位进 Bootloader,串口通道转给升级工具) */
        LOG_INFO("OTA: upgrade requested, rebooting to bootloader in 200ms...");
        OTA_RequestUpgrade();
    }
    else
    {
        LOG_INFO("unknown cmd: %s (help)", cmd);
    }
}

/* 命令处理任务 */
void DebugCmd_Task(void *argument)
{
    char line[RX_BUF_SIZE];             /* 任务侧工作缓冲(临界区拷入) */
    uint16_t len;

    for (;;)
    {
        len = 0;

        /* 无换行兜底:串口助手未勾"发送新行"时,发来的字节不带 \r\n。
           距最后一个字节超过 200ms 没有新字节,视作一行结束——
           与 Modbus 3.5T 字符间隔判帧同理。正常整行发送(含换行)不受影响 */
        if (!cmd_ready && rx_len > 0 &&
            (osKernelGetTickCount() - last_rx_tick) >= 200)
        {
            taskENTER_CRITICAL();
            if (!cmd_ready && rx_len > 0)
            {
                cmd_ready = 1;
            }
            taskEXIT_CRITICAL();
        }

        /* 临界区:拷走整行,防 ISR 写入竞争 */
        taskENTER_CRITICAL();
        if (cmd_ready)
        {
            memcpy(line, rx_buf, rx_len);
            len = rx_len;
            cmd_ready = 0;
            rx_len = 0;
        }
        taskEXIT_CRITICAL();

        if (len > 0)
        {
            /* 去掉行尾 \r\n */
            while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r'))
            {
                len--;
            }
            line[len] = 0;
            cmd_exec(line);
        }

        osDelay(10);
    }
}
