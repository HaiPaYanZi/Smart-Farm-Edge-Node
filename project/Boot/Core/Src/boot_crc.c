/**
  ******************************************************************************
  * @file    boot_crc.c
  * @brief   Bootloader CRC 计算(CRC16-MODBUS + CRC32,软件查表,无 HAL)
  *
  * 两个校验的用途(文档《07》):
  *   - CRC16-MODBUS:每帧传输校验(帧尾 2B),与 FreeModbus 同款多项式 0xA001
  *   - CRC32:固件全片校验(0x05 命令),与 PC 工具 Python 的 zlib.crc32 对齐
  *
  * 查表法原理:把"逐位异或移位"的 8 次迭代预计算成 256 项表,
  *   每字节只需 1 次查表+移位,速度约 4~6 周期/字节。
  *   表在 boot_crc_init 里运行时生成(256×4B×2 表 ≈ 1.5KB RAM,
  *   比放 Flash 省 1.5KB ROM——Boot 总共才 8KB)。
  *
  * 注意 CRC32 参数必须与 Python zlib.crc32 完全一致:
  *   多项式 0xEDB88320(反射形式),初值 0xFFFFFFFF,结果异或 0xFFFFFFFF。
  ******************************************************************************
  */
#include "boot_crc.h"

/* 查表缓冲(运行时生成,不用初始化值) */
static uint16_t s_crc16_table[256];   /* CRC16 表:256 项 × 2B = 512B */
static uint32_t s_crc32_table[256];   /* CRC32 表:256 项 × 4B = 1KB */

/**
  * @brief  生成两张 CRC 查表(上电调用一次,各迭代 256 次)
  * @note   查表原理:表中第 i 项 = 数值 i 单独做"8 次移位异或"后的结果
  */
void boot_crc_init(void)
{
  uint32_t index;                       /* 表下标 0~255 */
  uint32_t bit;                         /* 逐位迭代 8 次 */

  for (index = 0; index < 256; index++)
  {
    uint16_t crc16_value = (uint16_t)index;    /* CRC16:本项初值 */
    for (bit = 0; bit < 8; bit++)
    {
      /* 多项式 0xA001:最低位为 1 则右移并异或,否则只右移 */
      crc16_value = (crc16_value & 1) ? (crc16_value >> 1) ^ 0xA001 : (crc16_value >> 1);
    }
    s_crc16_table[index] = crc16_value;

    uint32_t crc32_value = index;              /* CRC32:本项初值 */
    for (bit = 0; bit < 8; bit++)
    {
      /* 多项式 0xEDB88320:反射算法,异或的是"右移后的低位" */
      crc32_value = (crc32_value & 1) ? (crc32_value >> 1) ^ 0xEDB88320UL : (crc32_value >> 1);
    }
    s_crc32_table[index] = crc32_value;
  }
}

/**
  * @brief  计算 CRC16-MODBUS(逐段可续算,帧校验用)
  * @param  init_value 初值(新数据用 OTA_CRC16_INIT=0xFFFF,续算传上次结果)
  * @param  buffer     数据缓冲
  * @param  size       字节数
  * @retval CRC16 结果(小端放帧尾,低字节在前)
  */
uint16_t boot_crc16_update(uint16_t init_value, const uint8_t *buffer, uint32_t size)
{
  uint16_t crc_value = init_value;      /* CRC 累计值 */
  uint32_t index;                       /* 数据下标 */

  for (index = 0; index < size; index++)
  {
    /* 查表法一步:高字节异或数据字节作为表下标,低字节左移补 0 */
    crc_value = (crc_value >> 8) ^ s_crc16_table[(crc_value ^ buffer[index]) & 0xFF];
  }
  return crc_value;
}

/**
  * @brief  计算 CRC32(zlib 标准,逐段可续算,固件全片校验用)
  * @param  init_value 初值(新数据用 OTA_CRC32_INIT=0xFFFFFFFF,续算传上次结果)
  * @param  buffer     数据缓冲
  * @param  size       字节数
  * @retval CRC32 中间值(最终结果需再异或 0xFFFFFFFF,见 boot_crc32_final)
  * @note   与 Python zlib.crc32 的中间值完全一致,PC 端可直接用
  */
uint32_t boot_crc32_update(uint32_t init_value, const uint8_t *buffer, uint32_t size)
{
  uint32_t crc_value = init_value;      /* CRC 累计值 */
  uint32_t index;                       /* 数据下标 */

  for (index = 0; index < size; index++)
  {
    crc_value = (crc_value >> 8) ^ s_crc32_table[(crc_value ^ buffer[index]) & 0xFF];
  }
  return crc_value;
}

/**
  * @brief  CRC32 收尾:异或 0xFFFFFFFF 得到最终值
  * @param  crc_mid boot_crc32_update 的返回值
  * @retval 最终 CRC32(zlib.crc32 的直接结果)
  */
uint32_t boot_crc32_final(uint32_t crc_mid)
{
  return crc_mid ^ 0xFFFFFFFFUL;
}

/**
  * @brief  一次性算整块数据的 CRC32(常用包装:update+final)
  * @param  buffer 数据缓冲
  * @param  size   字节数
  * @retval 最终 CRC32(可直接与固件头里存的 crc32 比较)
  */
uint32_t boot_crc32(const uint8_t *buffer, uint32_t size)
{
  return boot_crc32_final(boot_crc32_update(OTA_CRC32_INIT, buffer, size));
}
