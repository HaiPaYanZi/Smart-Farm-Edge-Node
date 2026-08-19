/**
  ******************************************************************************
  * @file    main.c
  * @brief   Bootloader 主程序(寄存器级,整个 Boot 控制在 8KB 内)
  *
  * 职责:
  *   1. 初始化:CRC 查表 → USART1(调试+升级通道) → SysTick 1ms 节拍
  *   2. 读升级标志区(0x08003000)三态判定(文档《07》§3.5):
  *      upgrade_req=0 → 正常启动:验活动区固件头跳转;
  *                       活动区无效试对侧区;都无效进升级模式
  *      upgrade_req=1 → 升级请求:直接进升级模式(5s 无握手自动回滚,
  *                       覆盖"传输 50% 断电"用例 E07:复位后回旧分区)
  *      upgrade_req=2 → 待验证:全片 CRC32 复验新分区
  *                       通过 → 切活动分区跳新;
  *                       失败 → 回旧分区并置故障码 0x0302(回滚路径)
  *   3. 跳转 App:关外设中断 → VTOR 指向分区+0x200 → 装 SP/PC → 执行
  *   4. 升级模式:boot_proto 状态机死循环(唯一的阻塞点)
  *
  * 时钟说明:启动文件先调 SystemInit(CMSIS 库函数,已配 VTOR=0x08000000、
  *           HSE 8MHz*9 PLL=72MHz、Flash 2 等待周期),本文件不重复配时钟省 ROM。
  ******************************************************************************
  */
#include "stm32f1xx.h"       /* 寄存器定义/SystemInit/SysTick_Config */
#include "boot_uart.h"       /* USART1 收发 */
#include "boot_flash.h"      /* Flash 擦写 */
#include "boot_crc.h"        /* CRC16/32 */
#include "boot_proto.h"      /* 协议状态机 + 标志区读写 */

/* 毫秒节拍:SysTick 中断累加,协议层 5s 超时用(声明在 boot_proto.h) */
volatile uint32_t g_tick_ms = 0;

/* ==================== 内部函数前置声明 ==================== */
static void clock_init(void);
static void stack_canary_init(void);       /* 栈金丝雀:未用区填哨兵 */
static uint32_t stack_used_peak(void);     /* 栈金丝雀:历史最深使用(峰值) */
static void stack_overflow_check(void);    /* 栈金丝雀:溢出检测(仅打印一次) */
static void stack_report(void);            /* 栈金丝雀:启动峰值报告 */
static uint8_t firmware_valid(uint32_t part_addr);
static uint8_t vector_table_looks_valid(uint32_t part_addr);
static uint8_t firmware_crc_check(uint32_t part_addr, uint32_t length, uint32_t expect_crc);
static void jump_to_app(uint32_t part_addr);
static void enter_upgrade_mode(uint8_t active_part);

/**
  * @brief  时钟配置:复位默认 HSI 8MHz → HSE 8MHz×PLL9 = 72MHz(寄存器级)
  * @note   F1 的 Cube CMSIS SystemInit 只设 VTOR 不配时钟,必须在这里手动配,
  *         否则芯片跑 HSI 8MHz,BRR=625 的串口实际只有 12800 波特率。
  *          配置顺序不能乱:
  *           开 HSE → 等就绪 → Flash 2 等待周期(72MHz 必须,先于切频) →
  *           写 CFGR(PLL 源 HSE/×9/APB1 2 分频=36MHz,此时 SW 位=0 仍是 HSI) →
  *           开 PLL → 等锁定 → 切 SW=PLL → 等 SWS 确认切换完成。
  */
static void clock_init(void)
{
  RCC->CR |= RCC_CR_HSEON;                    /* 开 HSE 晶振 */
  while ((RCC->CR & RCC_CR_HSERDY) == 0) {}   /* 等 HSE 起振就绪 */

  FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;  /* 预取+2等待周期(72MHz 要求) */

  RCC->CFGR = RCC_CFGR_PLLSRC                /* PLL 源=HSE(不分频) */
            | RCC_CFGR_PLLMULL9              /* ×9 = 72MHz */
            | RCC_CFGR_PPRE1_DIV2;           /* APB1 2 分频=36MHz(F103 外设上限) */

  RCC->CR |= RCC_CR_PLLON;
  while ((RCC->CR & RCC_CR_PLLRDY) == 0) {}  /* 等 PLL 锁定 */

  RCC->CFGR |= RCC_CFGR_SW_PLL;              /* 系统时钟切到 PLL */
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}  /* 等切换完成 */

  SystemCoreClock = 72000000;                /* 同步全局变量(SysTick 1ms 节拍用) */
}

/* ==================== 栈金丝雀:运行时栈溢出检测 ==================== */

extern uint32_t __initial_sp;               /* 栈顶地址(startup 文件导出的标号,最高地址) */

/* 栈大小必须与 startup 文件 Stack_Size 一致(本工程 0x800=2KB);
   栈底 = 栈顶 - 栈大小。必须用链接器符号推导、不能硬编码地址:
   .bss 内容变化会整体移动栈位置(实例:flush_data 改 static 后,
   栈底从 0x20001458 移到 0x20001c58),硬编码必错 */
#define STACK_SIZE_BYTES  (2048u)
#define STACK_BOTTOM      ((uint32_t)&__initial_sp - STACK_SIZE_BYTES)
#define CANARY_WORD       0xA5A5A5A5u       /* 哨兵模式 */

static uint8_t g_stack_overflowed = 0;      /* 溢出标志:置 1 后不再重复打印 */

static uint32_t g_canary_sp_depth = 0;      /* 诊断:填哨兵时的初始化栈用量(栈顶-当时SP) */

/**
  * @brief  初始化栈金丝雀:把"栈当前未用区"全部填上哨兵模式
  * @note   必须在 main() 最开头调用(越早越好,此时栈最浅)。
  *         只填 [栈底, 当前SP) 区间——SP 以上是正在使用的调用帧,
  *         填了会把现场冲掉,绝对禁止。
  *         原理:栈向下生长,运行中用多深,哨兵被踩多深;
  *         若溢出穿越栈底,连栈底哨兵都被踩坏(此时正在踩 .bss)。
  *         F103 无 MPU 硬件拦截,这是裸机最便宜的运行时防线。
  *         坑例:page_flush 局部数组 2KB 曾溢出踩 page_buffer,
  *         症状只有 CRC 失败、无任何报错——金丝雀可当场现形
  */
static void stack_canary_init(void)
{
  uint32_t *p = (uint32_t *)STACK_BOTTOM;   /* 从栈底开始填 */
  uint32_t *sp = (uint32_t *)__get_MSP();   /* 当前 SP:只填它以下未用区 */
  g_canary_sp_depth = (uint32_t)&__initial_sp - (uint32_t)sp; /* 记录初始化栈用量 */
  while (p < sp)
  {
    *p++ = CANARY_WORD;
  }
}

/**
  * @brief  报告栈的历史最深使用(已用字节)
  * @retval 已用字节 = 栈顶 - 最深水位(第一个非哨兵)
  * @note   从栈底向上扫:连续完好哨兵区结束的位置就是历史最深水位,
  *         栈顶减它 = 已用多少(栈从栈顶向下长,栈底到最深水位之间
  *         是自始至终没碰过的区域)。
  *         2026-08-18 修复:① 原返回"未用字节"却当"峰值"打印(误导);
  *         ② 无边界检查,曾扫过栈顶读到栈外(实测 App 打印 1024=全栈
  *         都是哨兵、越界停在栈顶)——现在扫描不许越过栈顶。
  *         若曾溢出,栈底哨兵已坏,本值无意义(此时看 g_stack_overflowed)
  */
static uint32_t stack_used_peak(void)
{
  uint32_t *p = (uint32_t *)STACK_BOTTOM;
  uint32_t *stack_top = (uint32_t *)&__initial_sp;   /* 栈顶:扫描上边界 */
  while (*p == CANARY_WORD && p < stack_top)
  {
    p++;
  }
  return (uint32_t)stack_top - (uint32_t)p;  /* 栈顶-最深水位 = 已用字节 */
}

/**
  * @brief  溢出检查:栈底 4 字节哨兵被踩 = 溢出穿越栈底(正在踩 .bss)
  * @note   周期调用,平时静默不打印;检测到溢出只打印一次防刷屏。
  *         哨兵被踩是"永久印记"——溢出发生时不在场,
  *         事后任何一次检查都能发现
  */
static void stack_overflow_check(void)
{
  if (g_stack_overflowed)
  {
    return;                                /* 已报过一次,不再重复 */
  }
  if (*(uint32_t *)STACK_BOTTOM != CANARY_WORD)
  {
    g_stack_overflowed = 1;
    boot_uart_send_string("stack: OVERFLOW! bottom canary destroyed\r\n");
  }
}

/**
  * @brief  启动报告:打印初始化阶段的栈用量(开发期看栈余量)
  * @note   used=已用字节(2KB 栈足够,16 位 hex 即可);init_sp=填哨兵时
  *         初始化链已用的栈(诊断:若接近栈总大小说明初始化栈紧张)
  */
static void stack_report(void)
{
  uint32_t used = stack_used_peak();
  boot_uart_send_string("stack: used=0x");
  boot_uart_send_hex((uint8_t)(used >> 8));    /* 16 位已用量逐字节打印 */
  boot_uart_send_hex((uint8_t)used);
  boot_uart_send_string("/0x0800 (init_sp=0x");
  boot_uart_send_hex((uint8_t)(g_canary_sp_depth >> 8));
  boot_uart_send_hex((uint8_t)g_canary_sp_depth);
  boot_uart_send_string(")\r\n");
}

/**
  * @brief  校验分区起始处的固件头:魔数+长度双检查(快速判定分区有效性)
  * @param  part_addr 分区起始地址(0x08010000 或 0x08020000)
  * @retval 1=有效固件 0=无效(可跳转前必查,防止跳到乱码区)
  * @note   头 16B 位于分区起始,向量表在分区+0x200(OTA_APP_OFFSET)。
  *         头的 crc32 字段是"固件数据 CRC"(文档《05》§3.2),不是头自身 CRC,
  *         这里不能拿它和头前 12 字节比较——曾经误比导致态 2 复验
  *         firmware_valid 永远失败、升级必回滚(实物抓到的坑)。
  *         全片完整性由升级 [5/6] 与态 2 firmware_crc_check 负责。
  */
static uint8_t firmware_valid(uint32_t part_addr)
{
  const ota_fw_header_t *header = (const ota_fw_header_t *)part_addr; /* 分区头指针 */

  if (header->magic != OTA_FW_MAGIC)        /* 魔数 "GWNM" 不符:非固件 */
  {
    return 0;
  }
  if (header->length == 0 || header->length > OTA_FW_MAX_SIZE)  /* 长度非法 */
  {
    return 0;
  }
  return 1;
}

/**
  * @brief  宽松验证:固件头无效时,检查向量表处的 SP/PC 是否合理(开发期兜底)
  * @param  part_addr 分区起始地址(向量表实际在 part_addr+0x200)
  * @retval 1=向量表看似有效(可跳) 0=无效
  * @note   首次用 Keil/ST-Link 直接烧录时,分区起始没有 16B 固件头
  *         (固件头只有 OTA 流程写对侧分区时才有),firmware_valid 必然失败。
  *         此时检查向量表[0]=初始 SP 应在 48KB RAM(0x20000000~0x2000C000)、
  *         向量表[1]=复位向量应在本分区范围内。空 Flash 全 0xFF →
  *         SP/PC=0xFFFFFFFF 必然不过,不会跳到乱码区,安全。
  */
static uint8_t vector_table_looks_valid(uint32_t part_addr)
{
  uint32_t app_vtor = part_addr + OTA_APP_OFFSET;                /* 向量表地址 */
  uint32_t stack_pointer = *(volatile uint32_t *)app_vtor;       /* 向量表[0]=初始 SP */
  uint32_t reset_handler = *(volatile uint32_t *)(app_vtor + 4); /* 向量表[1]=复位向量 */

  if (stack_pointer < 0x20000000UL || stack_pointer > 0x2000C000UL)  /* SP 不在 RAM 范围 */
  {
    return 0;
  }
  if (reset_handler < part_addr || reset_handler > part_addr + OTA_PART_SIZE) /* PC 不在本分区 */
  {
    return 0;
  }
  return 1;
}

/**
  * @brief  全片 CRC32 校验固件数据(待验证态 upgrade_req=2 专用)
  * @param  part_addr   分区起始地址
  * @param  length      固件长度(标志区记录的 new_length)
  * @param  expect_crc  期望 CRC32(标志区记录的 new_crc32)
  * @retval 1=校验通过 0=不通过
  * @note   分段 256B 读 Flash 计算,100KB 全片约几十毫秒,可接受
  */
static uint8_t firmware_crc_check(uint32_t part_addr, uint32_t length, uint32_t expect_crc)
{
  uint32_t crc_mid = OTA_CRC32_INIT;        /* CRC 中间值 */
  uint32_t remain = length;                 /* 剩余未算字节数 */
  uint32_t read_offset = 0;                 /* 已算偏移 */

  while (remain > 0)
  {
    uint32_t segment = remain > 256 ? 256 : remain;       /* 本段字节数(≤256) */
    uint8_t segment_buffer[256];                          /* 段缓冲(栈上) */
    const uint8_t *flash_ptr =
        (const uint8_t *)(part_addr + OTA_APP_OFFSET + read_offset); /* 段起始地址 */
    uint32_t index;                                       /* 段内下标 */
    for (index = 0; index < segment; index++)
    {
      segment_buffer[index] = flash_ptr[index];           /* Flash 直接读 */
    }
    crc_mid = boot_crc32_update(crc_mid, segment_buffer, segment);
    remain -= segment;
    read_offset += segment;
  }
  return (boot_crc32_final(crc_mid) == expect_crc) ? 1 : 0;
}

/**
  * @brief  跳转到指定分区的 App 固件(不返回)
  * @param  part_addr 分区起始地址(向量表实际在 part_addr+0x200)
  * @note   跳转前必须收拾干净 Boot 开过的资源:
  *         关 USART1 外设与其中断(否则 App 初始化前收到字节会进
  *         已失效的 Boot 中断向量)、停 SysTick、关全局中断。
  *         App 的启动代码自己会重开这些外设。
  */
static void jump_to_app(uint32_t part_addr)
{
  uint32_t app_vtor = part_addr + OTA_APP_OFFSET;         /* App 向量表地址 */
  uint32_t stack_pointer = *(volatile uint32_t *)app_vtor;        /* 向量表[0]=初始 SP */
  uint32_t reset_handler = *(volatile uint32_t *)(app_vtor + 4);  /* 向量表[1]=复位向量 */
  void (*app_entry)(void) = (void (*)(void))reset_handler;        /* 复位向量转函数指针 */

  USART1->CR1 &= ~USART_CR1_UE;             /* 关串口外设(先于关中断) */
  NVIC_DisableIRQ(USART1_IRQn);             /* 禁 USART1 中断 */
  SysTick->CTRL = 0;                        /* 停 SysTick 节拍 */
  __disable_irq();                          /* 关全局中断 */

  SCB->VTOR = app_vtor;                     /* 中断向量表重定向到 App */
  __set_MSP(stack_pointer);                 /* 装 App 栈指针(此后不再回 Boot 栈) */
  app_entry();                              /* 跳 App 复位向量,不再返回 */
}

/**
  * @brief  进入升级模式:初始化协议状态机后死循环轮询
  * @param  active_part 当前活动分区 'A'/'B'(握手应答与回滚用)
  * @note   boot_proto_poll 内部做 5s 无帧超时回滚:清升级请求并软复位,
  *         复位后走三态判定的正常启动分支回到 App
  */
static void enter_upgrade_mode(uint8_t active_part)
{
  boot_proto_start(active_part);            /* 打印提示并初始化协议上下文 */
  while (1)
  {
    stack_overflow_check();                 /* 金丝雀:溢出只打印一次(平时静默,不干扰协议) */
    boot_proto_poll();                      /* 帧解析+命令处理+超时检查 */
  }
}

/**
  * @brief  SysTick 中断服务:1ms 累加毫秒节拍
  * @note   只有"g_tick_ms++"一条指令,极短;覆盖启动文件弱定义
  */
void SysTick_Handler(void)
{
  g_tick_ms++;
}

/**
  * @brief  主函数:初始化 → 三态判定 → 跳转或升级模式
  * @retval 不返回(要么跳 App,要么死在升级模式循环)
  * @note   SystemInit 已由启动文件先调用(配好 VTOR/时钟/Flash 等待),
  *         全局中断默认关闭,初始化串口后需要 __enable_irq() 才能收帧
  */
int main(void)
{
  ota_flag_t flag;                          /* 升级标志区内容 */

  /* ---- 第 0 步:栈金丝雀(越早越好,必须在任何深栈调用之前) ---- */
  stack_canary_init();                      /* 栈未用区填哨兵,运行时溢出检测 */

  /* ---- 第 1 步:外设初始化 ---- */
  clock_init();                             /* 先配时钟 72MHz(F1 的 SystemInit 不配) */
  boot_crc_init();                          /* 运行时生成 CRC16/32 查表 */
  boot_uart_init();                         /* USART1:115200,中断接收 */
  SysTick_Config(SystemCoreClock / 1000);   /* 1ms 节拍(SystemCoreClock=72MHz) */
  __enable_irq();                           /* 开全局中断(收串口字节) */

  boot_uart_send_string("\r\nOTA Boot v1.0, ");
  boot_uart_send_string("active part=");
  /* 活动分区打印放三态判定后(此时还没读标志区) */
  boot_uart_send_newline();

  stack_report();                           /* 金丝雀:初始化阶段栈峰值(看余量) */

  /* ---- 第 2 步:读升级标志区 ---- */
  boot_flag_read(&flag);

  /* ---- 第 3 步:三态判定 ---- */

  /* 态 2:升级完成待验证(0x06 执行后软复位的落点) */
  if (flag.magic == OTA_FLAG_MAGIC && flag.upgrade_req == 2)
  {
    uint32_t new_addr = (flag.new_part == 'A') ? OTA_PART_A_ADDR : OTA_PART_B_ADDR; /* 新分区 */
    uint32_t old_addr = (flag.active_part == 'A') ? OTA_PART_A_ADDR : OTA_PART_B_ADDR; /* 旧分区 */

    boot_uart_send_string("OTA: verify new firmware...\r\n");
    if (firmware_valid(new_addr) &&
        firmware_crc_check(new_addr, flag.new_length, flag.new_crc32))
    {
      /* 验证通过:切换活动分区(文档《07》升级成功路径) */
      flag.active_part = flag.new_part;     /* 活动分区 = 新分区 */
      flag.upgrade_req = 0;                 /* 升级完成,清请求 */
      flag.fail_code = 0;                   /* 清故障码 */
      flag.seq++;                           /* 写入序号 +1 */
      boot_flag_write(&flag);               /* 落盘(掉电后仍从新分区启动) */
      boot_uart_send_string("OTA: verify ok, boot new part...\r\n");
      jump_to_app(new_addr);
    }
    else
    {
      /* 验证失败:回滚旧分区,置故障码 0x0302(文档《07》故障处理) */
      flag.upgrade_req = 0;                 /* 清请求 */
      flag.fail_code = OTA_FAIL_ROLLBACK;   /* 0x0302:升级校验失败回滚 */
      flag.seq++;
      boot_flag_write(&flag);
      boot_uart_send_string("OTA: verify fail, rollback old part...\r\n");
      jump_to_app(old_addr);
    }
  }

  /* 态 1:升级请求(App 写标志区后软复位落点,或传输中断电复位) */
  if (flag.magic == OTA_FLAG_MAGIC && flag.upgrade_req == 1)
  {
    boot_uart_send_string("OTA: upgrade requested, enter upgrade mode\r\n");
    enter_upgrade_mode(flag.active_part);
  }

  /* 态 0:正常启动(或无标志区的新芯片) */
  if (flag.magic == OTA_FLAG_MAGIC)
  {
    uint32_t active_addr = (flag.active_part == 'B') ? OTA_PART_B_ADDR : OTA_PART_A_ADDR; /* 活动区 */
    uint32_t other_addr  = (flag.active_part == 'B') ? OTA_PART_A_ADDR : OTA_PART_B_ADDR; /* 对侧区 */

    if (firmware_valid(active_addr))        /* 活动区有效:正常跳转 */
    {
      /* 启动日志:活动分区 + 上次升级故障码(0=成功,0x0302=回滚)。
         实物需求(2026-08-17):升级瞬间的 verify ok 打印只落在 PC 工具
         窗口里,助手看不到;这里在每次上电/复位时把升级结果重放出来 */
      boot_uart_send_string("OTA: part ");
      boot_uart_send_string((flag.active_part == 'B') ? "B" : "A");
      boot_uart_send_string(" valid, fail=");
      boot_uart_send_hex((uint8_t)(flag.fail_code >> 8));
      boot_uart_send_hex((uint8_t)(flag.fail_code & 0xFF));
      boot_uart_send_newline();
      jump_to_app(active_addr);
    }
    if (vector_table_looks_valid(active_addr))  /* 无固件头但向量表合理:开发期 Keil 直烧场景 */
    {
      boot_uart_send_string("OTA: no fw header, jump active part\r\n");
      jump_to_app(active_addr);
    }
    boot_uart_send_string("OTA: active part invalid, try other part\r\n");
    if (firmware_valid(other_addr))         /* 活动区坏了:试对侧区(容错) */
    {
      jump_to_app(other_addr);
    }
    if (vector_table_looks_valid(other_addr))   /* 对侧区同样允许无头跳转 */
    {
      boot_uart_send_string("OTA: no fw header, jump other part\r\n");
      jump_to_app(other_addr);
    }
    enter_upgrade_mode(flag.active_part);   /* 两区都无效:进升级模式等烧录 */
  }

  /* 标志区魔数不符:全新芯片首次上电,按 A → B 顺序找固件 */
  boot_uart_send_string("OTA: flag area empty, first boot\r\n");
  if (firmware_valid(OTA_PART_A_ADDR))
  {
    jump_to_app(OTA_PART_A_ADDR);
  }
  if (vector_table_looks_valid(OTA_PART_A_ADDR))  /* 首次 Keil 直烧无固件头:向量表合理即跳 */
  {
    boot_uart_send_string("OTA: no fw header, jump part A\r\n");
    jump_to_app(OTA_PART_A_ADDR);
  }
  if (firmware_valid(OTA_PART_B_ADDR))
  {
    jump_to_app(OTA_PART_B_ADDR);
  }
  if (vector_table_looks_valid(OTA_PART_B_ADDR))
  {
    boot_uart_send_string("OTA: no fw header, jump part B\r\n");
    jump_to_app(OTA_PART_B_ADDR);
  }
  enter_upgrade_mode('A');                  /* 默认以 A 为活动分区进升级模式 */

  while (1) {}                              /* 兜底死循环(理论走不到) */
}
