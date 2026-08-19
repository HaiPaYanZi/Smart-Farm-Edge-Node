#ifndef __BOOT_FLASH_H__
#define __BOOT_FLASH_H__

#include <stdint.h>

/* 解锁 Flash 控制寄存器(擦写前必须调用,写错密钥会锁死到复位) */
void boot_flash_unlock(void);

/* 锁定 Flash 控制寄存器(擦写完成后调用,防误写) */
void boot_flash_lock(void);

/* 擦除指定页(高密度器件页大小 2KB,页地址内部自动对齐)。
   返回值:0=成功 1=失败。擦除耗时约 20~40ms,期间不要读 Flash */
uint8_t boot_flash_erase_page(uint32_t page_address);

/* 半字编程:向已擦除地址写 16bit。
   返回值:0=成功 1=失败。F1 编程最小单位是半字,地址需 2 字节对齐 */
uint8_t boot_flash_program_halfword(uint32_t address, uint16_t data);

/* 按缓冲区连续编程(内部拆半字,奇数长度末尾补 0xFF)。
   address 所在页必须先擦除。返回值:0=成功 1=失败 */
uint8_t boot_flash_program(uint32_t address, const uint8_t *buffer, uint32_t size);

#endif /* __BOOT_FLASH_H__ */
