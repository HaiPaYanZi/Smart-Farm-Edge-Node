/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "log.h"
#include "MYOLED.h"
#include "OLED_Data.h"
#include "soil_moisture.h"
#include "BH1750.h"
#include "AHT20.h"
#include "rule_engine.h"
#include "MySensor.h"
#include "light.h"
#include "pump.h"
#include "fan.h"
#include "dsp_soft_IIC.h"
#include "bsp_hal_counter.h"
#include "stall_detect.h"
#include "buzzer.h"
#include "oled_page.h"
#include "debug_cmd.h"
#include "mb.h"
#include "modbus_reg.h"
#include "param.h"
#include "PID.h"
#include "nvs.h"
#include "sd_log.h"
#include "ota.h"
extern void stack_overflow_check(void);   /* 栈金丝雀周期检查(main.c USER CODE 区定义) */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* 猫动画帧表:按播放顺序排列(指针数组,零拷贝,仅占 8*4=32 字节) */
const uint8_t *const cat_frames[] = {
	bitmap_1,  bitmap_3,  bitmap_6,  bitmap_8,
	bitmap_10, bitmap_13, bitmap_15, bitmap_18
};
#define CAT_FRAME_COUNT  (sizeof(cat_frames) / sizeof(cat_frames[0]))	/* 自动计算帧数,加帧删帧不用改循环 */

/* 风机目标占空比(0~100):调试命令(手动模式)可修改,温度 PID 接入后由 PID 写 */
volatile uint8_t g_fan_duty = 0;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for task_collect */
osThreadId_t task_collectHandle;
const osThreadAttr_t task_collect_attributes = {
  .name = "task_collect",
  .stack_size = 192 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for task_control */
osThreadId_t task_controlHandle;
const osThreadAttr_t task_control_attributes = {
  .name = "task_control",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal4,
};
/* Definitions for task_show */
osThreadId_t task_showHandle;
const osThreadAttr_t task_show_attributes = {
  .name = "task_show",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal2,
};
/* Definitions for I2C_mutex */
osMutexId_t I2C_mutexHandle;
const osMutexAttr_t I2C_mutex_attributes = {
  .name = "I2C_mutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void DebugCmd_Task(void *argument);   /* 调试命令任务体(自创建任务,原型必须放 USER CODE 区,防被 CubeMX 清掉) */
void Modbus_Task(void *argument);     /* Modbus 从机任务(自创建,同 debug_cmd 约定) */
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTaskCollect(void *argument);
void StartTaskControl(void *argument);
void StartTaskShow(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* 栈溢出钩子:某任务栈写穿了才会进到这里(开发期必须开着)
      停死在这里,调试器看调用栈即可定位是哪个任务、哪一层调用 */
   __disable_irq();          /* 关中断,防止其他任务继续破坏现场 */
   while (1);
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  log_init();   /* 初始化日志系统（必须在调度器启动前调用） */
  SDLog_Init(); /* 初始化 SD 卡日志队列与事件锁（必须在调度器启动前调用） */
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of I2C_mutex */
  I2C_mutexHandle = osMutexNew(&I2C_mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of task_collect */
  task_collectHandle = osThreadNew(StartTaskCollect, NULL, &task_collect_attributes);

  /* creation of task_control */
  task_controlHandle = osThreadNew(StartTaskControl, NULL, &task_control_attributes);

  /* creation of task_show */
  task_showHandle = osThreadNew(StartTaskShow, NULL, &task_show_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* 调试命令任务:串口命令解析(低优先级,响应慢无所谓) */
  const osThreadAttr_t debugCmd_attributes = {
    .name = "debug_cmd",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };
  osThreadNew(DebugCmd_Task, NULL, &debugCmd_attributes);

  /* Modbus 从机任务:RS485 轮询(中优先级,文档 02 任务⑥) */
  const osThreadAttr_t modbus_attributes = {
    .name = "modbus",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };
  osThreadNew(Modbus_Task, NULL, &modbus_attributes);

  /* SD 卡日志任务:挂载 + 攒批落盘(最低优先级,写盘耗时不抢占任何控制/采集任务)。
     栈给 2KB:f_mount/f_open 栈消耗大,别学其他任务只给 1KB */
  const osThreadAttr_t sdlog_attributes = {
    .name = "sd_log",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityBelowNormal1,
  };
  osThreadNew(SDLog_Task, NULL, &sdlog_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);//设置PA8为低电平,检查一下芯片运行状态
  LOG_INFO("runing");       /* 示例:信息级日志,自动带时间戳 */
  MyI2C_Init();                 /* 软件 I2C(PB6/PB7) */
  OLED_Init();                  /* OLED 初始化 + 清屏 */
  AHT20_Init();                 /*温湿度传感器初始化*/
  BH1750_Init();                /*光照传感器初始化*/
  SoilMoisture_Init();         /*土壤湿度传感器初始化*/
  HAL_Counter_Init();          /* 霍尔脉冲计数清零(PB1/EXTI1) */
  StallDetect_Init();          /* 堵转检测状态机复位 */
  DebugCmd_Init();             /* 启动串口命令接收(USART1 中断) */
  OLED_Clear();
  osDelay(35);
  float Light;
  BH1750_Read(&Light);
  LOG_INFO("Light: %f \n", Light);
  float Moisture;
  SoilMoisture_Read(&Moisture);
  LOG_INFO("Moisture: %f \n", Moisture);
  float Temperature, Humidity;
  AHT20_Read(&Temperature, &Humidity);//往后面放，AHT20上电到采集数据响应比较慢
  LOG_INFO("Temperature: %f, Humidity: %f \n", Temperature, Humidity);
  /* 猫动画已交给 task_show,本任务保留做测试(如需 I2C 访问记得加锁) */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTaskCollect */
/**
* @brief Function implementing the task_collect thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskCollect */
void StartTaskCollect(void *argument)
{
  /* USER CODE BEGIN StartTaskCollect */
  /* 采集任务:每 500ms 刷新一次传感器快照
     软件 I2C(PB6/PB7)是共享总线,读传感器前必须拿 I2C 锁,
     与显示任务(OLED 同总线)互斥,防止两个任务同时敲总线 */
  for(;;)
  {
    osMutexAcquire(I2C_mutexHandle, osWaitForever); /* 拿 I2C 总线锁 */
    MySensor_Update();                              /* 统一刷新快照(温湿度/光照/土壤) */
    osMutexRelease(I2C_mutexHandle);                /* 释放 I2C 总线锁 */

    Fan_UpdateRpm();                                /* 500ms 差分换算转速(霍尔计数,不走 I2C 无需锁) */

    SDLog_Snapshot();                               /* 推一条快照 CSV 行(读快照不碰 I2C,日志任务负责落盘) */

    osDelay(500);
  }
  /* USER CODE END StartTaskCollect */
}

/* USER CODE BEGIN Header_StartTaskControl */
/**
* @brief Function implementing the task_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskControl */
void StartTaskControl(void *argument)
{
  /* USER CODE BEGIN StartTaskControl */
  /* 控制任务:100ms 精确节拍(osDelayUntil 防周期漂移,PID 积分项依赖它)
     温度 PID 每拍都算(文档《03》§2.2:控制周期 100ms,用最新快照);
     每 5 拍(500ms)评估一次规则引擎(文档:规则引擎 500ms 周期)。
     输出映射:规则开/关 → 补光 80%或 0%、水泵继电器;PID → 风机占空比
     注意:本任务不碰 I2C,只读快照、写执行器,无需 I2C 锁 */
  const uint32_t period = 100;           /* 节拍 100ms */
  uint32_t lastWake = osKernelGetTickCount();
  uint8_t tickCnt = 0;                   /* 拍数计数,5 拍 = 500ms */
  uint8_t fan_out;                       /* 堵转检测后的实际输出占空比 */
  uint8_t last_mode = MODE_AUTO;         /* 上一拍模式:检测模式切换(写 40005 后) */
  stall_state_t last_stall = STALL_NORMAL; /* 上一拍堵转状态:检测堵转跳变(推 SD 日志事件) */
  float current_temperature;             /* 本拍温度快照(取一次给 PID 用) */

  Param_Init();                          /* 参数区默认值 + PID 实例工作参数 */
  NVS_Init();                            /* 上电读 Flash:有保存过的参数则覆盖默认值 */

  for(;;)
  {
    stack_overflow_check();                   /* 栈金丝雀:MSP 中断栈溢出印记检查(仅溢出时打日志) */
    uint8_t current_mode = Param_GetMode();   /* 本拍模式(Modbus 写 40005 后下拍生效) */

    Param_Sync();                        /* Modbus 改过 PID 参数 → 刷进实例并清积分 */
    NVS_Tick();                          /* 参数变更防抖 2s → 落盘 Flash(掉电不丢) */
    OTA_Poll();                          /* 升级触发后 200ms 复位倒计时(Modbus 40020/debug ota 触发) */

    if (current_mode != last_mode)       /* 模式切换:旧模式的执行器状态和积分作废 */
    {
      last_mode = current_mode;
      RuleEngine_Init();                 /* 规则引擎状态表清零(文档:模式切换时调用) */
      PID_Reset(&g_pid);                 /* 清积分,防止切换期间攒的误差突跳输出 */

      /* 模式切换事件写入 SD 日志(事件列),便于事后查"什么时候切的手动" */
      if (current_mode == MODE_MANUAL)  { SDLog_Event("MODE_MANUAL"); }
      else if (current_mode == MODE_STANDBY) { SDLog_Event("MODE_STANDBY"); }
      else                              { SDLog_Event("MODE_AUTO"); }
    }

    if (current_mode == MODE_AUTO)       /* 自动:规则引擎 + PID 全自动 */
    {
      if (++tickCnt >= 5)                /* 每 500ms 评估一次规则 */
      {
        tickCnt = 0;

        RuleEngine_Tick(MySensor_GetValue);              /* 规则引擎评估(读快照) */

        /* 执行器输出映射(文档 §3.3):规则状态 → 器件驱动 */
        Light_Control(RuleEngine_GetActuator(ACT_LIGHT) == ACT_STATE_ON);
        Pump_Control(RuleEngine_GetActuator(ACT_PUMP)   == ACT_STATE_ON);
      }

      /* 温度 PID(每拍 100ms) → g_fan_duty */
      current_temperature = MySensor_GetValue(SENSOR_TEMP);   /* 取本拍温度快照(采集任务 500ms 刷新,这里用最新值) */
      if (current_temperature == SENSOR_INVALID)   /* 温度传感器失效:安全态固定 30%(文档《03》§4.2) */
      {
        g_fan_duty = 30;
      }
      else if (current_temperature < g_pid.Target - Param_GetFanHyst())
      {                                  /* R1 滞回:温度 < 目标-温差 → 停转 */
        PID_Reset(&g_pid);               /* 停转区间积分不许继续往上顶,回温后平稳起步 */
        g_fan_duty = 0;
      }
      else
      {
        g_pid.Actual = current_temperature;  /* 把本拍温度喂给 PID */
        PID_Update(&g_pid);              /* 位置式 PID 计算,输出已限幅 0~100 */
        g_fan_duty = (uint8_t)g_pid.Out; /* PID 输出 → 风机目标占空比 */
      }
    }
    else if (current_mode == MODE_MANUAL)
    {
      /* 手动:规则引擎与 PID 全停,执行器由 Modbus 直接控制(文档《02》状态图)
         g_fan_duty 已在 modbus_reg.c 写 40006 时更新;线圈直接操作水泵/补光 */
    }
    else                                 /* 待机:全部执行器停止输出 */
    {
      g_fan_duty = 0;
      Light_Control(ACT_STATE_OFF);
      Pump_Control(ACT_STATE_OFF);
    }

    /* 堵转检测(100ms 节拍):堵转时强制停机,返回实际输出 */
    fan_out = StallDetect_Tick(g_fan_duty);
    Fan_Control(fan_out);
    /* 蜂鸣器:堵转/严重故障时报警 */
    Buzzer_Control(StallDetect_IsStalled());

    /* 堵转状态跳变 → SD 日志事件(只在跳变沿发,不重复刷屏) */
    stall_state_t current_stall = StallDetect_GetState();   /* 本拍堵转状态 */
    if (current_stall != last_stall)
    {
      last_stall = current_stall;
      if (current_stall == STALL_FAULT)        { SDLog_Event("FAN_STALL"); }        /* 堵转报警 */
      else if (current_stall == STALL_FAULT_SEVERE) { SDLog_Event("FAN_STALL_SEVERE"); } /* 严重堵转 */
      else if (current_stall == STALL_NORMAL)  { SDLog_Event("FAN_STALL_CLEAR"); }  /* 故障解除 */
    }

    lastWake += period;                  /* 绝对节拍递增(防周期漂移) */
    osDelayUntil(lastWake);              /* 睡到该绝对时刻,单参数版 */
  }
  /* USER CODE END StartTaskControl */
}

/* USER CODE BEGIN Header_StartTaskShow */
/**
* @brief Function implementing the task_show thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskShow */
void StartTaskShow(void *argument)
{
  /* USER CODE BEGIN StartTaskShow */
  /* 显示任务:三页界面(低优先级)
     按键扫描 100ms(防抖切页),页面刷新 500ms
     OLED 走软件 I2C 同一条总线,写屏前必须拿 I2C 锁 */
  uint8_t showCnt = 0;                   /* 页刷新节拍:5×100ms = 500ms */

  for(;;)
  {
    osMutexAcquire(I2C_mutexHandle, osWaitForever);  /* 拿 I2C 总线锁 */
    OLED_Page_KeyScan();                             /* 按键扫描+切页(KEY1) */
    if (++showCnt >= 5)                              /* 每 500ms 刷一次页 */
    {
      showCnt = 0;
      OLED_Page_Show();                              /* 刷新当前页(读快照) */
    }
    osMutexRelease(I2C_mutexHandle);                 /* 释放 I2C 总线锁 */

    osDelay(100);
  }
  /* USER CODE END StartTaskShow */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* Modbus 从机任务:RTU,从站地址 4,9600bps 8N1(文档《06》)
   初始化 eMBInit/eMBEnable 放任务里(需要调度器已启动),
   轮询周期 1ms:RTU 帧靠 3.5T 超时(TIM2)判定结束,轮询及时处理 */
void Modbus_Task(void *argument)
{
   
    eMBInit(MB_RTU, Param_GetSlaveAddr(), 0, 9600, MB_PAR_NONE);   /* 从站地址:参数区(默认 4,写 40012 后重启生效) */
    eMBEnable();                                 /* 使能协议栈 */

    for (;;)
    {
        eMBPoll();      /* 协议栈轮询:取事件、处理帧、回响应 */
        osDelay(1);
    }
}
/* USER CODE END Application */

