/**
  ******************************************************************************
  * @file    sd_log.c
  * @brief   SD 卡 CSV 日志模块(节点 1.6 收尾,实现文档《03》§6.2)
  *
  * 数据流(两个生产者、一个消费者):
  *   采集任务(500ms)──SDLog_Snapshot──> 日志队列(16 条) ──> SDLog_Task 攒批落盘
  *   控制任务(事件)──SDLog_Event──> 事件暂存区 ──> 下一条快照行的"事件"列
  *
  * 落盘策略(文档要求):
  *   1. 队列攒满 16 条 → 拼接成一批,一次性 f_write + f_sync
  *   2. 10s 内未攒满也强制落盘(防数据滞留内存,掉电丢太多)
  *   3. 单文件超过 512KB → 关闭,滚动到下一个编号文件(LOG00001.CSV → LOG00002.CSV)
  *   4. 上电时从 LOG00001.CSV 向上找:未超限的文件以追加方式续写,不丢历史
  *
  * 时间戳说明:板子未启用 RTC(PC13 被运行灯占用),CSV 时间戳用上电运行时间
  *   [hh:mm:ss](与串口日志 log.c 同款格式)。需要真实日历时间时再接入 RTC。
  *   文件名/表头/事件名全部用英文:Keil 下汉字字符串字面量必须 GBK 编码,
  *   为避坑本模块代码内不写汉字字符串(注释是 UTF-8,不参与编译无影响)。
  ******************************************************************************
  */
#include "sd_log.h"
#include "main.h"           /* HAL_GetTick: 上电运行毫秒数(时间戳来源) */
#include "fatfs.h"          /* FatFs 接口: f_mount/f_open/f_write/f_sync */
#include "cmsis_os.h"       /* FreeRTOS CMSIS API: osMessageQueue/osMutex */
#include <stdio.h>          /* snprintf: 格式化 CSV 行 */
#include <string.h>         /* strlen/memcpy: 行长度与批缓冲拼接 */
#include "log.h"            /* LOG_ERROR: SD 卡异常时从串口报错(调试可见) */
#include "MySensor.h"       /* MySensor_GetValue: 读传感器快照 */
#include "fan.h"            /* Fan_GetRpm: 风机实际转速 */
#include "param.h"          /* Param_GetMode: 当前运行模式 */
#include "stall_detect.h"   /* StallDetect_GetState: 堵转状态机状态 */
#include "light.h"          /* Light_IsOn: 补光灯实际输出 */
#include "pump.h"           /* Pump_IsOn: 水泵实际输出 */

/* 风机目标占空比(0~100):freertos.c 定义,自动模式由 PID 写、手动模式由 Modbus 写。
   日志"target_duty"列用它——文档样例的"目标转速"列本实现无目标转速概念,用占空比替代 */
extern volatile uint8_t g_fan_duty;

/* ==================== 模块常量 ==================== */
#define LOG_QUEUE_LEN   16                  /* 队列条数:文档"攒 16 条一次性 f_write" */
#define LOG_LINE_MAX    96                  /* 单条 CSV 行缓冲(实际约 75 字节,留余量) */
#define LOG_BATCH_SIZE  (LOG_QUEUE_LEN * LOG_LINE_MAX) /* 批写缓冲:16 条拼接一次写入 */
#define LOG_FILE_MAX    (512UL * 1024UL)    /* 单文件上限:超过滚动新建(文档 §6.2) */
#define LOG_SYNC_TICKS  10000               /* 落盘兜底超时 10s(单位 ms) */
#define LOG_RETRY_TICKS 1000                /* 挂载/打开失败重试间隔 1s */

/* CSV 表头行:列名用英文(避 Keil 汉字字符串 GBK 编码坑),
   列顺序与文档《03》§6.2 一致:时间戳/温度/湿度/光照/土壤/实际转速/目标/模式/状态字/事件 */
static const char s_csv_header[] =
  "timestamp,temp(C),humi(%),light(lx),soil(%),rpm,target_duty(%),mode,status,event\r\n";

/* ==================== 模块内部变量 ==================== */
static osMessageQueueId_t s_log_queue = NULL;       /* 日志队列句柄:快照行按值拷贝入队(96B/条) */
static uint8_t s_batch_buffer[LOG_BATCH_SIZE];      /* 批写缓冲:落盘前把队列内容拼在这里,一次 f_write */
static char s_pending_event[24];                    /* 待填事件名:SDLog_Event 写入,下一条快照行消费后清空 */
static osMutexId_t s_event_mutex = NULL;            /* 事件暂存区互斥锁:控制任务写、采集任务读 */
static FATFS s_fatfs;                               /* FatFs 工作区(仅日志任务使用,无需锁) */
static FIL s_log_file;                              /* 当前日志文件句柄(仅日志任务使用) */
static uint8_t s_file_open = 0;                     /* 文件打开标志:1=已打开可写,0=待重开 */
static uint16_t s_file_number = 0;                  /* 当前文件编号(LOG00001.CSV 中的 1) */
static uint8_t s_reopen_error_reported = 0;         /* 重开失败是否已报过:连续失败只报一次,恢复后复位 */

/* ==================== 内部函数 ==================== */

/**
  * @brief  上电运行时间 → "[hh:mm:ss]" 字符串(与串口日志 log.c 同款格式)
  * @param  dest      输出缓冲(至少 16 字节)
  * @param  dest_size 缓冲大小
  * @note   HAL_GetTick 每 49.7 天回绕,本项目跑机周期远小于此,不处理回绕
  */
static void build_timestamp(char *dest, uint16_t dest_size)
{
  uint32_t elapsed_ms = HAL_GetTick();                  /* 上电以来毫秒数 */
  uint32_t hours      = elapsed_ms / 3600000UL;         /* 小时数 */
  uint32_t minutes    = (elapsed_ms / 60000UL) % 60UL;  /* 分钟数 */
  uint32_t seconds    = (elapsed_ms / 1000UL) % 60UL;   /* 秒数 */

  snprintf(dest, dest_size, "[%02lu:%02lu:%02lu]", hours, minutes, seconds);
}

/**
  * @brief  模式代号 → CSV 用的大写模式名(与 OLED 页3 显示一致)
  * @param  current_mode 模式值(Param_GetMode 返回值)
  * @retval 大写模式名字符串
  */
static const char *mode_name(uint8_t current_mode)
{
  if (current_mode == MODE_MANUAL) { return "MANUAL"; }   /* 手动模式 */
  if (current_mode == MODE_STANDBY) { return "STANDBY"; } /* 待机模式 */
  return "AUTO";                                           /* 其余按自动(默认) */
}

/**
  * @brief  合成当前系统状态字(16 位位图,位定义见 sd_log.h)
  * @retval 状态字:模式/执行器实际输出/堵转状态按位或合成
  * @note   补光/水泵取"实际输出"(IsOn 读 BSP 层),手动模式线圈直接操作时同样真实
  */
static uint16_t build_status_word(void)
{
  uint16_t status_word = 0;                          /* 状态字初始全 0 */
  uint8_t current_mode = Param_GetMode();            /* 当前运行模式 */

  if (current_mode == MODE_MANUAL)  { status_word |= STW_MANUAL; }  /* 手动模式置位 */
  if (current_mode == MODE_STANDBY) { status_word |= STW_STANDBY; } /* 待机模式置位 */

  if (MySensor_GetValue(SENSOR_TEMP) == SENSOR_INVALID)  /* 温度失效:读回无效值 */
  {
    status_word |= STW_TEMP_FAIL;
  }
  if (Light_IsOn()) { status_word |= STW_LIGHT_ON; }  /* 补光灯实际开着 */
  if (Pump_IsOn())  { status_word |= STW_PUMP_ON; }   /* 水泵实际开着 */

  stall_state_t stall_state = StallDetect_GetState(); /* 堵转状态机当前状态 */
  if (stall_state == STALL_FAULT)        { status_word |= STW_FAN_STALL; } /* 普通堵转 */
  if (stall_state == STALL_FAULT_SEVERE) { status_word |= STW_STALL_SEV; } /* 严重堵转 */

  return status_word;
}

/**
  * @brief  定位本次要写的日志文件(滚动规则,上电续写)
  * @retval 1=文件已打开可写  0=失败(SD 卡错误,调用方稍后重试)
  * @note   从 LOG00001.CSV 开始向上找:
  *         存在且 <512KB → 追加续写(指针移到文件尾,掉电不丢历史);
  *         存在且 ≥512KB → 关闭,编号 +1 继续找;
  *         不存在 → 用该编号新建并写表头
  */
static uint8_t open_current_log_file(void)
{
  FRESULT open_result;        /* f_open 返回值(FR_OK 为成功) */
  uint16_t try_number;        /* 当前尝试的文件编号(1~999) */
  char file_path[24];         /* 文件名缓冲:"LOG00001.CSV" 共 13 字符 */

  for (try_number = 1; try_number <= 999; try_number++)
  {
    snprintf(file_path, sizeof(file_path), "LOG%05u.CSV", try_number);

    /* 先按"文件已存在"打开(读+写),成功说明这个编号的历史文件在 */
    open_result = f_open(&s_log_file, file_path, FA_OPEN_EXISTING | FA_READ | FA_WRITE);
    if (open_result == FR_OK)
    {
      if (f_size(&s_log_file) < LOG_FILE_MAX)   /* 未超限:追加续写 */
      {
        f_lseek(&s_log_file, f_size(&s_log_file)); /* 写指针移到文件尾 */
        break;                                     /* 找到可写文件,结束查找 */
      }
      f_close(&s_log_file);                        /* 已超限:关闭,编号 +1 继续找 */
    }
    else if (open_result == FR_NO_FILE)            /* 该编号不存在:新建 */
    {
      open_result = f_open(&s_log_file, file_path, FA_CREATE_NEW | FA_WRITE);
      if (open_result != FR_OK)
      {
        return 0;                                  /* 新建失败(卡错误):返回待重试 */
      }
      UINT header_written = 0;                     /* 表头实际写入字节数 */
      f_write(&s_log_file, s_csv_header, strlen(s_csv_header), &header_written);
      break;                                       /* 新建成功,结束查找 */
    }
    else
    {
      return 0;                                    /* 其他错误(卡未就绪等):返回待重试 */
    }
  }

  if (try_number > 999)                            /* 999 个文件全满(极端情况) */
  {
    return 0;
  }

  s_file_number = try_number;                      /* 记录当前编号(调试用) */
  s_file_open = 1;                                 /* 置打开标志 */
  return 1;
}

/* ==================== 对外接口 ==================== */

/**
  * @brief  上电初始化:创建日志队列与事件互斥锁
  * @note   必须在调度器启动前调用(FreeRTOS 对象需在 osKernelStart 前创建好),
  *         创建失败(堆不足)时句柄为 NULL,SDLog_Snapshot/Event 内部已做判空保护
  */
void SDLog_Init(void)
{
  s_log_queue = osMessageQueueNew(LOG_QUEUE_LEN, LOG_LINE_MAX, NULL);  /* 16 条 × 96B 队列 */
  s_event_mutex = osMutexNew(NULL);                                    /* 事件暂存区互斥锁 */
}

/**
  * @brief  推一条传感器快照 CSV 行进队列(采集任务每 500ms 调用)
  * @note   对应 F12 用例"每采集周期一条记录"。
  *         队列满时丢弃本条(历史数据优先保留,SD 卡跟不上时宁缺毋滥);
  *         传感器失效时数值列为 -1.0(即 SENSOR_INVALID),可读性优先于好看
  */
void SDLog_Snapshot(void)
{
  char csv_line[LOG_LINE_MAX];  /* 本行 CSV 缓冲 */
  char timestamp_str[16];       /* 时间戳字符串 "[hh:mm:ss]" */
  char event_str[24] = "-";     /* 事件列内容,默认无事件 */

  if (s_log_queue == NULL)      /* 队列未创建(初始化未调用):直接返回 */
  {
    return;
  }

  /* 取走待填事件(有则填入本行,无则保持 "-"):控制任务可能刚登记了事件 */
  if (s_event_mutex != NULL &&
      osMutexAcquire(s_event_mutex, osWaitForever) == osOK)
  {
    if (s_pending_event[0] != '\0')               /* 有待填事件 */
    {
      snprintf(event_str, sizeof(event_str), "%s", s_pending_event); /* 拷进事件列 */
      s_pending_event[0] = '\0';                  /* 消费掉:一条事件只填一行 */
    }
    osMutexRelease(s_event_mutex);
  }

  build_timestamp(timestamp_str, sizeof(timestamp_str));

  /* 拼 CSV 行:时间戳,温度,湿度,光照,土壤,实际转速,目标占空比,模式,状态字,事件 */
  snprintf(csv_line, sizeof(csv_line),
           "%s,%.1f,%.1f,%.0f,%.1f,%u,%u,%s,0x%04X,%s\r\n",
           timestamp_str,
           MySensor_GetValue(SENSOR_TEMP),   /* 温度(℃) */
           MySensor_GetValue(SENSOR_HUMI),   /* 湿度(%) */
           MySensor_GetValue(SENSOR_LIGHT),  /* 光照(lx,整数) */
           MySensor_GetValue(SENSOR_SOIL),   /* 土壤(%) */
           (unsigned)Fan_GetRpm(),           /* 风机实际转速(RPM) */
           (unsigned)g_fan_duty,             /* 目标占空比(0~100) */
           mode_name(Param_GetMode()),       /* 模式名(AUTO/MANUAL/STANDBY) */
           build_status_word(),              /* 状态字(位定义见 sd_log.h) */
           event_str);                       /* 事件(默认 "-") */

  /* 入队(按值拷贝 96 字节):队列满返回 osErrorResource,本条直接丢弃 */
  osMessageQueuePut(s_log_queue, csv_line, 0, 0);
}

/**
  * @brief  登记一条事件:填入下一条快照行的"事件"列
  * @param  event_name 事件名(英文大写,如 "FAN_STALL"),仅保存最近一条
  * @note   调用方在控制任务(状态跳变时),消费方在采集任务(Snapshot 内),
  *         两者不同任务,故用互斥锁保护暂存区
  */
void SDLog_Event(const char *event_name)
{
  if (s_event_mutex == NULL)      /* 互斥锁未创建:直接返回 */
  {
    return;
  }
  if (osMutexAcquire(s_event_mutex, osWaitForever) == osOK)
  {
    snprintf(s_pending_event, sizeof(s_pending_event), "%s", event_name);
    osMutexRelease(s_event_mutex);
  }
}

/**
  * @brief  日志落盘任务(自创建任务,与 Modbus_Task 同款约定)
  * @param  argument 未使用
  * @note   流程:挂载 SD 卡(带重试) → 定位续写文件 → 主循环攒批落盘。
  *         只有本任务访问 FatFs(_FS_REENTRANT=0 够用,勿再开其他任务碰 SD 卡);
  *         写盘失败时关闭文件、串口报错,下一批重新尝试打开
  */
void SDLog_Task(void *argument)
{
  /* ---- 第 1 步:挂载 SD 卡(带重试,卡未插好/上电未就绪时每 1s 试一次) ---- */
  FRESULT mount_result;                       /* f_mount 返回值 */
  uint8_t mount_error_reported = 0;           /* 挂载错误是否已报过:只报一次,防卡未插时串口每秒刷屏 */
  do
  {
    mount_result = f_mount(&s_fatfs, "0:", 1);  /* "0:" 盘符,第二参数 1=立即挂载 */
    if (mount_result != FR_OK)
    {
      if (mount_error_reported == 0)            /* 首次失败:串口报一次(带错误码) */
      {
        LOG_ERROR("sd mount fail: %u, retrying...", (unsigned)mount_result);
        mount_error_reported = 1;               /* 置已报标志:后续失败静默重试,不刷屏 */
      }
      osDelay(LOG_RETRY_TICKS);                 /* 1s 后重试 */
    }
  } while (mount_result != FR_OK);
  LOG_INFO("sd mounted");                       /* 挂载成功:串口提示 */

  /* ---- 第 2 步:定位本次要写的文件(滚动规则见函数注释) ---- */
  while (open_current_log_file() == 0)          /* 打开失败:重试 */
  {
    osDelay(LOG_RETRY_TICKS);
  }
  LOG_INFO("sd log file: LOG%05u.CSV", s_file_number); /* 串口告知当前文件名 */

  /* ---- 第 3 步:主循环:收队列攒批 → 滚动检查 → 一次性 f_write ---- */
  for (;;)
  {
    uint32_t batch_bytes = 0;                   /* 本批已收集字节数 */
    char one_line[LOG_LINE_MAX];                /* 本批取出的单条 CSV 行 */

    /* 收队列凑批:满 16 条自动退出;10s 未收到新数据也退出(落盘兜底) */
    while (batch_bytes + LOG_LINE_MAX <= sizeof(s_batch_buffer))
    {
      osStatus_t get_status = osMessageQueueGet(s_log_queue, one_line, NULL, LOG_SYNC_TICKS);
      if (get_status != osOK)                   /* 超时或队列错误:结束收集 */
      {
        break;
      }
      uint16_t line_length = (uint16_t)strlen(one_line); /* 本条行字节数(含\r\n) */
      memcpy(&s_batch_buffer[batch_bytes], one_line, line_length); /* 拼进批缓冲 */
      batch_bytes += line_length;
    }

    if (batch_bytes == 0)                       /* 什么都没收到:回循环头继续等 */
    {
      continue;
    }

    /* 滚动检查:当前文件超过 512KB → 关闭重开(下一个编号) */
    if (f_size(&s_log_file) + batch_bytes > LOG_FILE_MAX)
    {
      f_close(&s_log_file);                     /* 关闭超限文件 */
      s_file_open = 0;
    }

    /* 确保文件打开(新建/续写失败后的重开路径) */
    if (s_file_open == 0 && open_current_log_file() == 0)
    {
      if (s_reopen_error_reported == 0)         /* 连续失败只报一次:防卡坏时每批刷屏 */
      {
        LOG_ERROR("sd log reopen fail");        /* 重开失败:丢弃本批,下批再试 */
        s_reopen_error_reported = 1;            /* 置已报标志 */
      }
      continue;
    }
    s_reopen_error_reported = 0;                /* 重开成功:复位标志,下次失败再报 */

    /* 一次性 f_write 整批 + f_sync 落盘(CTRL_SYNC 已实现,防掉电丢数据) */
    UINT batch_written = 0;                     /* 实际写入字节数 */
    FRESULT write_result = f_write(&s_log_file, s_batch_buffer, (UINT)batch_bytes, &batch_written);
    if (write_result == FR_OK)
    {
      f_sync(&s_log_file);                      /* 冲刷到物理介质(掉电安全) */
    }
    else
    {
      LOG_ERROR("sd log write fail: %u", (unsigned)write_result); /* 串口报错误码 */
      f_close(&s_log_file);                     /* 关闭,下批重新打开 */
      s_file_open = 0;
    }
  }
}
