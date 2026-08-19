#ifndef __BOOT_CRC_H__
#define __BOOT_CRC_H__

#include <stdint.h>

/* CRC 初值常量(与协议约定一致) */
#define OTA_CRC16_INIT  0xFFFF          /* CRC16-MODBUS 初值 */
#define OTA_CRC32_INIT  0xFFFFFFFFUL    /* CRC32(zlib 标准)初值 */

/* 上电调用一次:运行时生成两张查表(省 1.5KB ROM) */
void boot_crc_init(void);

/* CRC16-MODBUS 逐段更新:初值传 OTA_CRC16_INIT,续算传上次结果 */
uint16_t boot_crc16_update(uint16_t init_value, const uint8_t *buffer, uint32_t size);

/* CRC32 逐段更新(中间值,与 zlib.crc32 内部一致):初值传 OTA_CRC32_INIT */
uint32_t boot_crc32_update(uint32_t init_value, const uint8_t *buffer, uint32_t size);

/* CRC32 收尾:update 的返回值异或 0xFFFFFFFF 得最终值 */
uint32_t boot_crc32_final(uint32_t crc_mid);

/* 整块数据一次性 CRC32(update+final 包装),与固件头 crc32 字段直接比较 */
uint32_t boot_crc32(const uint8_t *buffer, uint32_t size);

#endif /* __BOOT_CRC_H__ */
