#ifndef __BOOT_UART_H__
#define __BOOT_UART_H__

#include <stdint.h>

/* 初始化 USART1:PA9=TX/PA10=RX,115200 8N1,RXNE 中断收字节进环形缓冲。
   调用前系统时钟必须已配置为 72MHz(波特率按 72M/115200=625 计算) */
void boot_uart_init(void);

/* 取环形缓冲中可读字节数(主循环轮询用) */
uint32_t boot_uart_available(void);

/* 读一个字节:缓冲空返回 -1,否则返回 0~255 */
int32_t boot_uart_read_byte(void);

/* 发送原始字节块(轮询 TXE,阻塞直到全部送进发送寄存器) */
void boot_uart_send(const uint8_t *buffer, uint32_t size);

/* 发送 C 字符串(调试打印用,内容必须是 ASCII 英文——Keil 汉字字符串 GBK 坑) */
void boot_uart_send_string(const char *string);

/* 发送字节的十六进制文本(调试打印数值用,如 "3F ") */
void boot_uart_send_hex(uint8_t byte_value);

/* 发送换行(CR LF,串口助手上一条一行) */
void boot_uart_send_newline(void);

#endif /* __BOOT_UART_H__ */
