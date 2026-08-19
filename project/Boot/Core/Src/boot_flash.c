/**
  ******************************************************************************
  * @file    boot_flash.c
  * @brief   Bootloader 内部 Flash 编程驱动(寄存器级,不依赖 HAL)
  *
  * 提供三个原子操作,全部基于 FLASH 外设寄存器:
  *   - 整页擦除(2KB/页,高密度器件,擦一页约 20~40ms,期间 Flash 不可读)
  *   - 半字编程(16bit,编程前地址必须处于已擦除状态)
  *   - 解锁/上锁(擦写前后必须配对调用,防止误写)
  *
  * 注意:擦除/编程进行中若读同一 Flash 会总线挂起等待(CPU 停顿),
  *       所以本驱动只做"发起→轮询 BSY 标志",不依赖中断。
  ******************************************************************************
  */
#include "stm32f1xx.h"      /* FLASH_TypeDef/FLASH 寄存器定义 */
#include "boot_flash.h"

/**
  * @brief  等待 Flash 内部操作完成(擦除/编程共用)
  * @retval 0=操作成功完成  1=发生编程错误(写入已编程区/电压异常)
  * @note   轮询 FLASH_SR 的 BSY 位,BSY=1 表示内部操作还在进行
  */
static uint8_t flash_wait_done(void)
{
  uint32_t timeout_counter = 0x00FFFFFF;    /* 超时计数:擦一页最坏 40ms,该值足够大 */

  while ((FLASH->SR & FLASH_SR_BSY) != 0)   /* BSY=1:操作未完成 */
  {
    if (--timeout_counter == 0)             /* 计数耗尽:硬件异常,返回失败 */
    {
      return 1;
    }
  }
  /* 编程错误检查:向未擦除区写数据会置 PGERR。之前只等 BSY 不看错误位,
     写失败被当成成功吞掉,升级全片 CRC 必失败(实物抓到的坑)。置位即报失败 */
  if ((FLASH->SR & FLASH_SR_PGERR) != 0)
  {
    FLASH->SR = FLASH_SR_PGERR;             /* 写 1 清标志 */
    return 1;
  }
  return 0;                                 /* BSY=0:操作完成 */
}

/**
  * @brief  解锁 Flash 控制寄存器(擦写前必须调用)
  * @note   依次写入两个固定密钥到 FLASH_KEYR,写错顺序则硬件锁定直到复位
  */
void boot_flash_unlock(void)
{
  FLASH->KEYR = 0x45670123;                 /* 密钥 1 */
  FLASH->KEYR = 0xCDEF89AB;                 /* 密钥 2 */
}

/**
  * @brief  锁定 Flash 控制寄存器(擦写完成后调用,防误写)
  */
void boot_flash_lock(void)
{
  FLASH->CR |= FLASH_CR_LOCK;               /* 置 LOCK 位:擦写寄存器失效 */
}

/**
  * @brief  擦除指定页(高密度器件页大小 2KB)
  * @param  page_address 页内任意地址(内部对齐到 2KB 页起始)
  * @retval 0=擦除成功  1=擦除失败
  * @note   调用前必须已解锁;擦除会把整页写成 0xFF
  */
uint8_t boot_flash_erase_page(uint32_t page_address)
{
  uint8_t result;                            /* 操作结果 */

  FLASH->CR |= FLASH_CR_PER;                 /* 使能页擦除模式 */
  FLASH->AR = page_address;                  /* 写入页地址(硬件自动取 2KB 边界) */
  FLASH->CR |= FLASH_CR_STRT;                /* 置启动位:开始擦除 */
  result = flash_wait_done();                /* 等 BSY 清零 */
  FLASH->CR &= ~FLASH_CR_PER;                /* 退出页擦除模式 */
  return result;
}

/**
  * @brief  半字编程(向已擦除地址写 16bit 数据)
  * @param  address 目标地址(必须 2 字节对齐且已擦除为 0xFFFF)
  * @param  data    要写入的半字
  * @retval 0=写入成功  1=写入失败
  * @note   STM32F1 的 Flash 编程最小单位是半字,不能按字节写
  */
uint8_t boot_flash_program_halfword(uint32_t address, uint16_t data)
{
  uint8_t result;                            /* 操作结果 */

  FLASH->CR |= FLASH_CR_PG;                  /* 使能编程模式 */
  *(volatile uint16_t *)address = data;      /* 向目标地址写入半字,硬件自动开始编程 */
  result = flash_wait_done();                /* 等 BSY 清零 */
  FLASH->CR &= ~FLASH_CR_PG;                 /* 退出编程模式 */
  return result;
}

/**
  * @brief  按缓冲区连续编程(内部拆成半字写,常用:写固件数据/标志区)
  * @param  address 起始地址(2 字节对齐,所在页已擦除)
  * @param  buffer  数据缓冲
  * @param  size    字节数(奇数时最后补一字节 0xFF)
  * @retval 0=全部写入成功  1=中途失败
  */
uint8_t boot_flash_program(uint32_t address, const uint8_t *buffer, uint32_t size)
{
  uint32_t offset;                           /* 缓冲内偏移 */
  uint16_t halfword;                         /* 本次写入的半字 */

  for (offset = 0; offset < size; offset += 2)
  {
    halfword = buffer[offset];               /* 低字节 */
    if (offset + 1 < size)
    {
      halfword |= (uint16_t)buffer[offset + 1] << 8;  /* 高字节(够长才取) */
    }
    else
    {
      halfword |= 0xFF00;                    /* 奇数长度:末尾补 0xFF */
    }
    if (boot_flash_program_halfword(address + offset, halfword) != 0)
    {
      return 1;                              /* 编程失败立即返回 */
    }
  }
  return 0;
}
