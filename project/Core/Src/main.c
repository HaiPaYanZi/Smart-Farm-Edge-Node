/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "fatfs.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "log.h"
#include "mb.h"   /* Modbus 协议栈头(内含 port.h 类型定义 + mbport.h 接口声明) */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 栈金丝雀:栈大小必须与 startup 文件 Stack_Size 一致(App:0x800=2KB,
   2026-08-18 从 1KB 加大——HAL 初始化+printf 链实际用量比 1KB 紧张,
   实测打印曾出现扫描越界,先给足余量再谈检测);
   栈底 = 栈顶(链接器符号 __initial_sp) - 栈大小,不硬编码地址
   (代码改动会移动栈位置,硬编码必错) */
#define STACK_SIZE_BYTES  (2048u)
#define STACK_BOTTOM      ((uint32_t)&__initial_sp - STACK_SIZE_BYTES)
#define CANARY_WORD       0xA5A5A5A5u   /* 哨兵模式 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 诊断:填哨兵时的初始化栈用量(栈顶-当时SP)。定义在 USER CODE BEGIN 4,
   此处提前声明——BEGIN 2 的 LOG_INFO 要打印它(调用早于定义会隐式声明报错) */
static uint32_t g_canary_sp_depth;
extern uint32_t __initial_sp;   /* 栈顶(startup 文件导出的标号)。提前声明:BEGIN 1
                                   的 canary_init 展开 STACK_BOTTOM 宏要用(定义在 BEGIN 4) */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
static void stack_canary_init(void);      /* 栈金丝雀:填哨兵(实现见 USER CODE BEGIN 4) */
static uint32_t stack_used_peak(void);    /* 栈金丝雀:历史最深使用,已用字节(实现见 USER CODE BEGIN 4) */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* OTA 双分区:向量表由 Boot 跳转前设置(Boot/main.c jump_to_app:
     SCB->VTOR = 目标分区地址+0x200,住 A 设 0x08010200、住 B 设 0x08020200),
     App 不再自设 VTOR。
     2026-08-19 修复:原代码 SCB->VTOR = Image$$ER_IROM1$$Base 是编译时常量
     (恒=0x08010200,旧注释"随烧录位置跟随"是错的)——从 B 区运行时会把 Boot
     设好的正确值改回 A 区,中断向量指 A 区旧固件(靠旧固件碰巧兼容才没炸,
     A 区损坏则 B 区运行时中断必 HardFault)。删掉后双分区都真正正确。 */
  /* 栈金丝雀:填哨兵(必须在 HAL_Init 之前!实测 2026-08-18:放 BEGIN 2 太晚,
     HAL 初始化期的栈使用不被哨兵覆盖,used 只显示 8B 假象;
     放这里才能让哨兵覆盖初始化期,扫描才看得到真实峰值——与 Boot 同款时机) */
  stack_canary_init();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  log_init();   /* 初始化日志系统（必须在调度器启动前调用） */
  /* 打印 MSP 栈真实使用:used=已用字节/总字节;init_sp=填哨兵时刻的栈深度
     (填哨兵已提前到 BEGIN 1,此时扫描才看得到 HAL 初始化期的真实用量;
     used 接近总大小说明初始化链栈紧张,栈还要加大) */
  LOG_INFO("stack: msp used=%luB/%uB (init_sp=%luB)",
           (unsigned long)stack_used_peak(), STACK_SIZE_BYTES,
           (unsigned long)g_canary_sp_depth);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* 注意：osKernelStart() 启动调度器后永不返回，
     用户代码必须写在任务函数（freertos.c 的 StartDefaultTask）里 */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* ==================== 栈金丝雀:MSP 栈运行时溢出检测 ==================== */
/* 说明:FreeRTOS 任务栈溢出已由 vApplicationStackOverflowHook(freertos.c
   USER CODE 区)负责;这里管 MSP 栈(2KB)——调度器启动前的 HAL 初始化、
   运行期中断嵌套都用它。F103 无 MPU 硬件拦截,金丝雀是裸机最便宜
   的运行时防线(哨兵被踩是永久印记,事后检查也能发现)。
   2026-08-18 修复:① 扫描补栈顶边界(原无边界检查,曾越界读);
   ② 返回语义改为"已用字节"(原返回的是"未用",打印标签却写 peak,
   实测 Boot 打印 1708 实为未用,App 打印 1024 实为扫描越界——误导) */

extern uint32_t __initial_sp;   /* 栈顶(startup 文件导出的标号) */

static uint8_t g_stack_overflowed = 0;   /* 溢出标志:只报一次 */

static uint32_t g_canary_sp_depth = 0;   /* 诊断:填哨兵时的初始化栈用量(栈顶-当时SP) */

static void stack_canary_init(void)      /* 填哨兵:只填 [栈底, 当前SP) 未用区 */
{
  uint32_t *p = (uint32_t *)STACK_BOTTOM;//栈底
  uint32_t *sp = (uint32_t *)__get_MSP();//当前SP
  g_canary_sp_depth = (uint32_t)&__initial_sp - (uint32_t)sp; /* 记录初始化阶段栈用量 */
  while (p < sp)
  {
    *p++ = CANARY_WORD;
  }
}

static uint32_t stack_used_peak(void)    /* 已用字节 = 栈顶 - 最深水位(第一个非哨兵) */
{
  uint32_t *p = (uint32_t *)STACK_BOTTOM;
  uint32_t *stack_top = (uint32_t *)&__initial_sp;  /* 栈顶:扫描不许越过(栈外是.bss等) */
  while (*p == CANARY_WORD && p < stack_top)//遍历栈底向上,找第一个被踩的哨兵
  {
    p++;
  }
  return (uint32_t)stack_top - (uint32_t)p;  /* 栈顶-最深水位 = 已用字节 */
}

void stack_overflow_check(void)          /* 跨文件调用(控制任务每 100ms 一拍) */
{
  if (g_stack_overflowed)
  {
    return;
  }
  if (*(uint32_t *)STACK_BOTTOM != CANARY_WORD)
  {
    g_stack_overflowed = 1;
    LOG_WARN("stack overflow: MSP bottom canary destroyed!");
  }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM2)     /* Modbus 3.5T 字符超时:帧收完(porttimer.c) */
  {
    vMBPortTimersDisable();       /* 单次超时,停表 */
    pxMBPortCBTimerExpired();     /* 通知协议栈:这一帧收完了 */
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
