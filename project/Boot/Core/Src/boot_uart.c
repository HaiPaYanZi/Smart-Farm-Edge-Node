/**
  ******************************************************************************
  * @file    boot_uart.c
  * @brief   Bootloader USART1 收发(寄存器级,中断接收 + 环形缓冲)
  *
  * 为什么用中断+环形缓冲而不是轮询:
  *   PC 工具 115200 波特率发 256B 数据包约需 22ms;Boot 擦 Flash 一页要
  *   20~40ms。若轮询收字节,擦页期间串口数据直接丢失 → 升级必失败。
  *   中断把每个字节搬进环形缓冲(1KB,可容纳约 2 帧),主循环忙完再解析。
  *
  * 引脚:PA9=TX(复用推挽 50MHz)/PA10=RX(浮空输入)——板载 CH340 直连 PC。
  * 本模块只认 USART1,升级通道即文档勘误后的 USART1(USART2 留给 ESP8266)。
  ******************************************************************************
  */
#include "stm32f1xx.h"      /* RCC/GPIO/USART/NVIC 寄存器定义 */
#include "boot_uart.h"

/* ==================== 环形缓冲 ==================== */
#define BOOT_UART_BUF_SIZE  1024          /* 环形缓冲 1KB:最大帧 522B,可容约 2 帧 */
static volatile uint8_t s_rx_buffer[BOOT_UART_BUF_SIZE]; /* 接收缓冲(中断写/主循环读) */
static volatile uint32_t s_rx_head = 0;   /* 写指针:ISR 收到字节后 +1 */
static volatile uint32_t s_rx_tail = 0;   /* 读指针:主循环取字节后 +1 */

/* 波特率寄存器值:USARTDIV = 72MHz / 115200 = 625,整数写入 BRR 即可 */
#define BOOT_UART_BRR       (72000000UL / 115200UL)

/**
  * @brief  初始化 USART1 的 GPIO/外设/中断(调用前系统时钟已配 72MHz)
  */
void boot_uart_init(void)
{
  /* ---- 第 1 步:开 GPIOA 与 USART1 的时钟(都挂 APB2 总线) ---- */
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;       /* 开 GPIOA 时钟 */
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;     /* 开 USART1 时钟 */

  /* ---- 第 2 步:配置 PA9(TX)复用推挽 50MHz ---- */
  GPIOA->CRH &= ~(0xF << (9 - 8) * 4);      /* 清 PA9 原配置(CRH 管引脚 8~15) */
  GPIOA->CRH |= (0xB << (9 - 8) * 4);       /* 0xB = 复用推挽输出,50MHz */

  /* ---- 第 3 步:配置 PA10(RX)浮空输入(串口外设直接采样引脚) ---- */
  GPIOA->CRH &= ~(0xF << (10 - 8) * 4);     /* 清 PA10 原配置 */
  GPIOA->CRH |= (0x4 << (10 - 8) * 4);      /* 0x4 = 浮空输入 */

  /* ---- 第 4 步:配置 USART1:115200 8N1,收发使能,开接收中断 ---- */
  USART1->BRR = BOOT_UART_BRR;              /* 波特率 115200 */
  USART1->CR1 = USART_CR1_UE                /* 串口总使能 */
              | USART_CR1_TE                /* 发送使能 */
              | USART_CR1_RE                /* 接收使能 */
              | USART_CR1_RXNEIE;           /* 接收缓冲非空中断:每收 1 字节进一次 ISR */

  /* ---- 第 5 步:使能 USART1 中断(抢占优先级 0,升级流程对时序敏感) ---- */
  NVIC_SetPriority(USART1_IRQn, 0);         /* 最高抢占优先级:ISR 只搬 1 字节,极短 */
  NVIC_EnableIRQ(USART1_IRQn);
}

/**
  * @brief  取环形缓冲中可读字节数(主循环轮询用)
  * @retval 可读字节数 0 ~ BOOT_UART_BUF_SIZE-1
  */
uint32_t boot_uart_available(void)
{
  return (s_rx_head - s_rx_tail) & (BOOT_UART_BUF_SIZE - 1); /* 无符号差自动处理回绕 */
}

/**
  * @brief  读一个字节(缓冲空返回 -1)
  */
int32_t boot_uart_read_byte(void)
{
  uint8_t byte_value;                       /* 读出的字节 */

  if (s_rx_head == s_rx_tail)               /* 头尾相等:缓冲空 */
  {
    return -1;
  }
  byte_value = s_rx_buffer[s_rx_tail];      /* 取当前字节 */
  s_rx_tail = (s_rx_tail + 1) & (BOOT_UART_BUF_SIZE - 1); /* 读指针前移(回绕) */
  return byte_value;
}

/**
  * @brief  发送原始字节块(轮询 TXE,阻塞)
  * @param  buffer 数据缓冲
  * @param  size   字节数
  */
void boot_uart_send(const uint8_t *buffer, uint32_t size)
{
  uint32_t index;                           /* 数据下标 */

  for (index = 0; index < size; index++)
  {
    while ((USART1->SR & USART_SR_TXE) == 0) {}   /* TXE=1:发送数据寄存器空,可写入 */
    USART1->DR = buffer[index];                   /* 写入数据,硬件自动移位发出 */
  }
}

/**
  * @brief  发送 C 字符串(调试打印用)
  * @note   字符串必须是 ASCII 英文:Keil 下汉字字符串字面量必须 GBK 编码,
  *         Boot 工程按 UTF-8 保存源码,含汉字会编译报错(见项目编码坑记录)
  */
void boot_uart_send_string(const char *string)
{
  while (*string != '\0')                   /* 逐字符发送直到串尾 */
  {
    boot_uart_send((const uint8_t *)string, 1);
    string++;
  }
}

/**
  * @brief  发送一个字节的十六进制文本(调试打印数值)
  * @param  byte_value 要显示的字节值
  * @note   输出格式 "XX "(两位大写十六进制+空格),如 0x3F 显示 "3F "
  */
void boot_uart_send_hex(uint8_t byte_value)
{
  static const char hex_digits[] = "0123456789ABCDEF"; /* 十六进制字符表 */
  char text[4];                             /* "XX " + 结尾 */

  text[0] = hex_digits[byte_value >> 4];    /* 高半字节 */
  text[1] = hex_digits[byte_value & 0x0F];  /* 低半字节 */
  text[2] = ' ';
  text[3] = '\0';
  boot_uart_send_string(text);
}

/**
  * @brief  发送换行(CR LF)
  */
void boot_uart_send_newline(void)
{
  static const char newline[] = "\r\n";
  boot_uart_send_string(newline);
}

/**
  * @brief  USART1 中断服务函数(接收非空中断:每收 1 字节进一次)
  * @note   覆盖启动文件的弱定义;只做"搬字节进环形缓冲"这一件极短的事,
  *         帧解析全部放主循环,ISR 里禁止碰 Flash(擦页期间读 Flash 会挂死总线)
  */
void USART1_IRQHandler(void)
{
  if ((USART1->SR & USART_SR_RXNE) != 0)    /* 接收缓冲非空 */
  {
    uint32_t next_head = (s_rx_head + 1) & (BOOT_UART_BUF_SIZE - 1); /* 下一写位置 */
    if (next_head != s_rx_tail)             /* 缓冲未满(满了丢新字节,防覆盖未解析数据) */
    {
      s_rx_buffer[s_rx_head] = (uint8_t)USART1->DR; /* 读 DR 即清 RXNE 标志 */
      s_rx_head = next_head;
    }
    else
    {
      (void)USART1->DR;                     /* 缓冲满:读走丢弃,清标志防止死等 */
    }
  }
}
